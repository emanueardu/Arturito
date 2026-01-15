import os, re, json, time, threading, serial
from serial import SerialException
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String, Int16
from sensor_msgs.msg import BatteryState, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

def _convert_env(value: str, default):
    if isinstance(default, bool):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    if isinstance(default, int):
        try:
            return int(value.strip())
        except ValueError:
            return default
    if isinstance(default, float):
        try:
            return float(value.strip())
        except ValueError:
            return default
    return value


def p(node, name, default):
    env = os.getenv(name.upper())
    if env is not None:
        value = _convert_env(env, default)
    else:
        value = default
    return node.declare_parameter(name, value).value

LINE_RE = re.compile(rb'\{.*\}\s*$')

class ESP32SerialBridge(Node):
    def __init__(self):
        super().__init__('esp32_serial_bridge')
        self.port       = p(self, 'port', '/dev/ttyAMA0')
        self.baud       = int(p(self, 'baud', 115200))
        self.poll_hz    = float(p(self, 'poll_hz', 10.0))
        self._max_pwm_limit = int(float(p(self, 'max_pwm', 255.0)))
        self._current_max_pwm = self._max_pwm_limit
        self.max_vx     = float(p(self, 'max_vx', 0.5))
        self.max_wz     = float(p(self, 'max_wz', 1.5))
        self.base_width = float(p(self, 'base_width', 0.28))
        self._bump_recovery_enabled = bool(p(self, 'bump_recovery_enabled', True))
        self._bump_recovery_pwm = int(float(p(self, 'bump_recovery_pwm', 180)))
        self._bump_recovery_duration = float(p(self, 'bump_recovery_duration', 0.4))
        self._obstacle_avoid_enabled = bool(p(self, 'obstacle_avoid_enabled', True))
        self._obstacle_distance_threshold = float(p(self, 'obstacle_distance_threshold', 0.35))
        self._obstacle_reverse_pwm = int(float(p(self, 'obstacle_reverse_pwm', 120)))
        self._obstacle_reverse_duration = float(p(self, 'obstacle_reverse_duration', 0.3))
        self._obstacle_turn_pwm = int(float(p(self, 'obstacle_turn_pwm', 160)))
        self._obstacle_turn_duration = float(p(self, 'obstacle_turn_duration', 0.45))
        self._distance_request_period = float(p(self, 'distance_request_period', 1.0))

        self.pub_batt   = self.create_publisher(BatteryState, '/battery_state', 10)
        self.pub_imu    = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_diag   = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.pub_bump_l = self.create_publisher(Bool, '/events/bump_left', 10)
        self.pub_bump_r = self.create_publisher(Bool, '/events/bump_right', 10)
        self.pub_cliff  = self.create_publisher(Bool, '/events/cliff', 10)
        self.pub_status = self.create_publisher(String, '/esp32/status', 10)

        self._send_lock = threading.Lock()
        self._recovering = False
        self._recovery_thread: Optional[threading.Thread] = None
        self._last_twist: Tuple[float, float] = (0.0, 0.0)
        self._queued_twist: Optional[Tuple[float, float]] = None
        self._last_distance: Optional[float] = None
        self._last_obstacle_time = 0.0
        self._last_bump_left = False
        self._last_bump_right = False

        self.sub_twist  = self.create_subscription(Twist,  '/cmd_vel', self.cb_twist, 10)
        self.sub_tilt   = self.create_subscription(Float32,'/head/tilt', self.cb_tilt, 10)
        self.sub_vac    = self.create_subscription(Bool, '/vacuum/enabled', self.cb_vac, 10)
        self.sub_brush  = self.create_subscription(Bool, '/brush/enabled',  self.cb_brush,10)
        self.sub_pwm    = self.create_subscription(Int16, '/motors/max_pwm', self.cb_max_pwm, 10)

        self.ser = None
        self.rx_buf = bytearray()
        self._stop = False
        self._connect_serial()
        self.rx_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.rx_thread.start()

        self.poll_dt = max(0.02, 1.0/float(self.poll_hz))
        self.timer = self.create_timer(self.poll_dt, self.request_sens)
        self.distance_timer = None
        if self._distance_request_period > 0.0:
            self.distance_timer = self.create_timer(self._distance_request_period, self.request_dis)

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
            self.get_logger().info(f"Serial abierto en {self.port} @ {self.baud}")
        except SerialException as exc:
            self.ser = None
            self.get_logger().warn(f"No se pudo abrir {self.port}: {exc}")

    def cb_twist(self, msg: Twist):
        vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        wz = max(-self.max_wz, min(self.max_wz, msg.angular.z))
        self._last_twist = (vx, wz)
        if self._recovering:
            self._queued_twist = (vx, wz)
            return
        self._apply_velocity_pwm(vx, wz)

    def _apply_velocity_pwm(self, vx: float, wz: float) -> None:
        if self._current_max_pwm <= 0:
            l = 0
            r = 0
        else:
            max_vx = self.max_vx if abs(self.max_vx) > 1e-6 else 1e-6
            max_wz = self.max_wz if abs(self.max_wz) > 1e-6 else 1e-6
            l = (vx / max_vx - (wz / max_wz)) * self._current_max_pwm
            r = (vx / max_vx + (wz / max_wz)) * self._current_max_pwm
            l = int(max(-self._current_max_pwm, min(self._current_max_pwm, l)))
            r = int(max(-self._current_max_pwm, min(self._current_max_pwm, r)))
        self.send_cmd(f"M{l} {r}")

    def cb_tilt(self, msg: Float32):
        deg = max(-30.0, min(90.0, float(msg.data)))
        self.send_cmd(f"t{int(round(deg))}")

    def cb_vac(self, msg: Bool):
        self.send_cmd(f"VAC{1 if msg.data else 0}")

    def cb_brush(self, msg: Bool):
        self.send_cmd(f"BRUSH{1 if msg.data else 0}")

    def cb_max_pwm(self, msg: Int16):
        desired = int(msg.data)
        if desired <= 0:
            new_pwm = self._max_pwm_limit
        else:
            new_pwm = max(0, min(self._max_pwm_limit, desired))
        if new_pwm != self._current_max_pwm:
            self._current_max_pwm = new_pwm
            self.get_logger().info(f"Max PWM actualizado a {self._current_max_pwm}")
        if self._recovering:
            # Asegura que la secuencia de recuperación use el límite actualizado.
            self._queued_twist = self._last_twist

    def _handle_bump_state(self, side: str, pressed: bool, force_trigger: bool = False) -> None:
        if side == 'left':
            previous = self._last_bump_left
            self._last_bump_left = pressed
        else:
            previous = self._last_bump_right
            self._last_bump_right = pressed
        if pressed and (force_trigger or not previous):
            self._trigger_bump_recovery(side)

    def _trigger_bump_recovery(self, side: str) -> None:
        if not self._bump_recovery_enabled:
            return
        if self._recovering:
            return
        if self._recovery_thread and self._recovery_thread.is_alive():
            return
        self.get_logger().info(f"Bumper {side} activado, iniciando giro de evasión.")
        self._recovery_thread = threading.Thread(
            target=self._run_bump_recovery, args=(side,), daemon=True
        )
        self._recovery_thread.start()

    def _run_bump_recovery(self, side: str) -> None:
        self._begin_recovery()
        pwm = min(self._bump_recovery_pwm, self._current_max_pwm)
        if pwm <= 0 or self._bump_recovery_duration <= 0.0:
            self._finish_recovery()
            return
        if side == 'left':
            left_pwm = pwm
            right_pwm = -pwm
        else:
            left_pwm = -pwm
            right_pwm = pwm
        self.send_cmd(f"M{left_pwm} {right_pwm}")
        time.sleep(self._bump_recovery_duration)
        self._finish_recovery()

    def _start_obstacle_avoidance(self) -> None:
        if not self._obstacle_avoid_enabled:
            return
        if self._recovering:
            return
        if self._recovery_thread and self._recovery_thread.is_alive():
            return
        self.get_logger().info("Obstáculo frontal detectado, iniciando maniobra de evasión.")
        self._last_obstacle_time = time.monotonic()
        self._recovery_thread = threading.Thread(
            target=self._run_obstacle_avoidance,
            daemon=True,
        )
        self._recovery_thread.start()

    def _run_obstacle_avoidance(self) -> None:
        self._begin_recovery()
        back_pwm = min(self._obstacle_reverse_pwm, self._current_max_pwm)
        turn_pwm = min(self._obstacle_turn_pwm, self._current_max_pwm)

        if back_pwm > 0 and self._obstacle_reverse_duration > 0.0:
            self.send_cmd(f"M{-back_pwm} {-back_pwm}")
            time.sleep(self._obstacle_reverse_duration)

        if turn_pwm > 0 and self._obstacle_turn_duration > 0.0:
            self.send_cmd(f"M{turn_pwm} {-turn_pwm}")
            time.sleep(self._obstacle_turn_duration)

        self._finish_recovery()

    def _begin_recovery(self) -> None:
        self._recovering = True
        self._queued_twist = None
        self.send_cmd("M0 0")
        time.sleep(0.05)

    def _finish_recovery(self) -> None:
        self.send_cmd("M0 0")
        self._recovering = False
        resume = self._queued_twist if self._queued_twist is not None else self._last_twist
        self._queued_twist = None
        self._recovery_thread = None
        self._apply_velocity_pwm(resume[0], resume[1])

    def _handle_distance_from_obj(self, obj: dict) -> None:
        distance = self._extract_distance(obj)
        if distance is None:
            return
        self._last_distance = distance
        self._check_obstacle_distance(distance)

    def _extract_distance(self, obj: dict) -> Optional[float]:
        keys_m = ('distance_m', 'ultrasonic_m', 'front_distance', 'distance')
        for key in keys_m:
            if key in obj:
                try:
                    return float(obj[key])
                except (TypeError, ValueError):
                    continue

        if 'distance_cm' in obj:
            try:
                return float(obj['distance_cm']) / 100.0
            except (TypeError, ValueError):
                pass
        if 'distance_mm' in obj:
            try:
                return float(obj['distance_mm']) / 1000.0
            except (TypeError, ValueError):
                pass

        if 'dis' in obj:
            try:
                raw = float(obj['dis'])
                # Heurística: si parece una medida grande asumimos centímetros.
                return raw / 100.0 if raw > 5.0 else raw
            except (TypeError, ValueError):
                pass
        return None

    def _check_obstacle_distance(self, distance: float) -> None:
        if not self._obstacle_avoid_enabled:
            return
        if distance <= 0.0:
            return
        if distance > self._obstacle_distance_threshold:
            return
        now = time.monotonic()
        if now - self._last_obstacle_time < 1.0:
            return
        self._start_obstacle_avoidance()

    def read_loop(self):
        while not self._stop:
            if self.ser is None or not self.ser.is_open:
                self._connect_serial()
                time.sleep(1.0)
                continue
            try:
                chunk = self.ser.read(256)
                if chunk:
                    self.rx_buf.extend(chunk)
                    while b'\n' in self.rx_buf or b'\r' in self.rx_buf:
                        for sep in (b'\n', b'\r'):
                            if sep in self.rx_buf:
                                i = self.rx_buf.find(sep)
                                line = self.rx_buf[:i].strip()
                                del self.rx_buf[:i+1]
                                break
                        if not line:
                            continue
                        if LINE_RE.match(line):
                            self.parse_json_line(line.decode(errors='ignore'))
                        else:
                            txt = line.decode(errors='ignore')
                            if txt:
                                self.pub_status.publish(String(data=txt))
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(0.5)

    def parse_json_line(self, s: str):
        try:
            obj = json.loads(s)
        except Exception as e:
            self.get_logger().warn(f"JSON invalido: {s} ({e})")
            return

        self._handle_distance_from_obj(obj)

        if 'status' in obj:
            self.pub_status.publish(String(data=obj.get('status','')))
            return
        if 'err' in obj:
            self.pub_status.publish(String(data=f"ERR:{obj['err']}"))
            return

        if 'vbat' in obj or 'ibat' in obj or 'yaw' in obj or 'roll' in obj or 'pitch' in obj:
            b = BatteryState()
            b.voltage = float(obj.get('vbat', 0.0))
            b.current = float(obj.get('ibat', 0.0))
            b.percentage = -1.0
            b.present = True
            self.pub_batt.publish(b)

            imu = Imu()
            if all(k in obj for k in ('ax','ay','az','gx','gy','gz')):
                imu.linear_acceleration.x = float(obj['ax'])
                imu.linear_acceleration.y = float(obj['ay'])
                imu.linear_acceleration.z = float(obj['az'])
                imu.angular_velocity.x    = float(obj['gx'])
                imu.angular_velocity.y    = float(obj['gy'])
                imu.angular_velocity.z    = float(obj['gz'])
            else:
                imu.angular_velocity.z    = float(obj.get('yaw',   0.0))
                imu.linear_acceleration.y = float(obj.get('pitch', 0.0))
                imu.linear_acceleration.x = float(obj.get('roll',  0.0))
                imu.angular_velocity_covariance = [0.2,0,0,0,0.2,0,0,0,0.2]
                imu.linear_acceleration_covariance = [0.5,0,0,0,0.5,0,0,0,0.5]
            imu.orientation_covariance[0] = -1.0
            self.pub_imu.publish(imu)

            if 'bL' in obj:
                left_pressed = bool(obj['bL'])
                self.pub_bump_l.publish(Bool(data=left_pressed))
                self._handle_bump_state('left', left_pressed)
            if 'bR' in obj:
                right_pressed = bool(obj['bR'])
                self.pub_bump_r.publish(Bool(data=right_pressed))
                self._handle_bump_state('right', right_pressed)
            if 'cliff' in obj:
                self.pub_cliff.publish(Bool(data=bool(obj['cliff'])))
            return

        if obj.get('event') == 'bumpL':
            pressed = bool(obj.get('pressed', False))
            self.pub_bump_l.publish(Bool(data=pressed))
            self._handle_bump_state('left', pressed, force_trigger=True)
            return
        if obj.get('event') == 'bumpR':
            pressed = bool(obj.get('pressed', False))
            self.pub_bump_r.publish(Bool(data=pressed))
            self._handle_bump_state('right', pressed, force_trigger=True)
            return
        if obj.get('event') == 'cliff':
            self.pub_cliff.publish(Bool(data=bool(obj.get('detected', False))))
            return

        if 'L' in obj or 'R' in obj or 'tilt' in obj or 'vac' in obj or 'brush' in obj:
            di = DiagnosticArray()
            st = DiagnosticStatus()
            st.level = DiagnosticStatus.OK
            st.name = 'esp32/telemetry'
            for k,v in obj.items():
                st.values.append(KeyValue(key=str(k), value=str(v)))
            di.status.append(st)
            self.pub_diag.publish(di)

    def request_sens(self):
        self.send_cmd("SENS")

    def request_dis(self):
        self.send_cmd("DIS")

    def send_cmd(self, cmd: str):
        try:
            payload = (cmd + '\n').encode()
            with self._send_lock:
                if self.ser is None or not self.ser.is_open:
                    raise SerialException(f'Serial no disponible para enviar "{cmd}"')
                self.ser.write(payload)
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.close()
        except:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = ESP32SerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
