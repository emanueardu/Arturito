import json
import math
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import BatteryState, Imu, Range
from std_msgs.msg import Bool, Int16, String
from std_srvs.srv import SetBool, Trigger

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - handled at runtime
    serial = None  # type: ignore
    SerialException = Exception  # type: ignore


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class ArturitoUARTBridge(Node):
    def __init__(self) -> None:
        super().__init__('arturito_uart_bridge')

        if serial is None:
            raise RuntimeError('pyserial is required to use ArturitoUARTBridge')

        qos = QoSProfile(depth=10)
        self._port = self.declare_parameter('port', '/dev/serial0').value
        self._baudrate = int(self.declare_parameter('baudrate', 115200).value)
        self._read_timeout = float(self.declare_parameter('read_timeout_sec', 0.05).value)
        self._status_request_period = float(
            self.declare_parameter('status_request_period_sec', 0.5).value
        )
        self._auto_request_status = bool(
            self.declare_parameter('auto_request_status', True).value
        )
        self._max_wheel_pwm = int(self.declare_parameter('max_wheel_pwm', 255).value)
        self._max_wheel_speed = float(
            self.declare_parameter('max_wheel_speed_mps', 0.25).value
        )
        # Deadband: el motor real no responde por debajo de este PWM. Si el
        # comando es no-cero pero su |PWM| < min_wheel_pwm, lo elevamos a
        # min_wheel_pwm conservando el signo. Con cmd=0 → PWM=0 (frena).
        self._min_wheel_pwm = int(self.declare_parameter('min_wheel_pwm', 200).value)
        self._base_width = float(self.declare_parameter('base_width_m', 0.28).value)
        self._imu_frame = self.declare_parameter('imu_frame_id', 'imu_link').value
        self._range_frame = self.declare_parameter('range_frame_id', 'ultrasonic_link').value
        self._battery_frame = self.declare_parameter('battery_frame_id', 'base_link').value
        self._imu_accel_lsb_per_g = float(
            self.declare_parameter('imu_accel_lsb_per_g', 16384.0).value
        )
        self._imu_gyro_lsb_per_dps = float(
            self.declare_parameter('imu_gyro_lsb_per_dps', 131.0).value
        )
        self._ultra_fov = math.radians(
            float(self.declare_parameter('ultrasonic_fov_deg', 30.0).value)
        )
        self._ultra_min = float(
            self.declare_parameter('ultrasonic_min_range_m', 0.05).value
        )
        self._ultra_max = float(
            self.declare_parameter('ultrasonic_max_range_m', 2.0).value
        )
        self._status_prefix = self.declare_parameter('status_prefix', 'SENS').value
        self._event_prefix = self.declare_parameter('event_prefix', 'EVENT').value
        self._swap_bumpers = bool(self.declare_parameter('swap_bumpers', False).value)
        self._wake_topic = str(
            self.declare_parameter('wake_topic', '/wake_word/detected').value
        )
        self._movement_topic = str(
            self.declare_parameter('movement_topic', '/movement_cmds').value
        )
        self._cmd_vel_topic = str(self.declare_parameter('cmd_vel_topic', '/cmd_vel').value)
        self._wander_cmd_vel_topic = str(
            self.declare_parameter('wander_cmd_vel_topic', 'arturito/cmd_vel_wander').value
        )
        self._wander_status_topic = str(
            self.declare_parameter('wander_status_topic', '/robot_web/wander_status').value
        )
        self._clean_cmd_vel_topic = str(
            self.declare_parameter('clean_cmd_vel_topic', 'arturito/cmd_vel_clean').value
        )
        self._clean_status_topic = str(
            self.declare_parameter('clean_status_topic', '/robot_web/clean_status').value
        )
        if not self._cmd_vel_topic.startswith('/'):
            self._cmd_vel_topic = f'/{self._cmd_vel_topic}'
        if not self._wander_cmd_vel_topic.startswith('/'):
            self._wander_cmd_vel_topic = f'/{self._wander_cmd_vel_topic}'
        if not self._wander_status_topic.startswith('/'):
            self._wander_status_topic = f'/{self._wander_status_topic}'
        if not self._clean_cmd_vel_topic.startswith('/'):
            self._clean_cmd_vel_topic = f'/{self._clean_cmd_vel_topic}'
        if not self._clean_status_topic.startswith('/'):
            self._clean_status_topic = f'/{self._clean_status_topic}'
        self._enabled = bool(self.declare_parameter('start_enabled', True).value)

        self._serial_lock = threading.Lock()
        self._serial: Optional[serial.Serial] = None
        self._closing = False
        self._last_serial_error_log = 0.0
        self._vacuum_state: Optional[bool] = None
        self._brush_state: Optional[bool] = None
        self._last_pwm_left = 0
        self._last_pwm_right = 0
        self._wander_active = False
        self._clean_active = False

        self._pub_status_raw = self.create_publisher(String, 'arturito/status_raw', qos)
        self._pub_events_raw = self.create_publisher(String, 'arturito/events_raw', qos)
        self._pub_battery = self.create_publisher(BatteryState, 'arturito/battery', qos)
        self._pub_range = self.create_publisher(Range, 'arturito/ultrasonic', qos)
        self._pub_imu = self.create_publisher(Imu, 'arturito/imu', qos)
        self._pub_bumper_left = self.create_publisher(Bool, 'arturito/bumper_left', qos)
        self._pub_bumper_right = self.create_publisher(Bool, 'arturito/bumper_right', qos)
        self._pub_cliff = self.create_publisher(Bool, 'arturito/cliff', qos)
        self._pub_vacuum_state = self.create_publisher(Bool, 'arturito/vacuum_state', qos)
        self._pub_brush_state = self.create_publisher(Bool, 'arturito/brush_state', qos)
        self._pub_left_pwm = self.create_publisher(Int16, 'arturito/wheel_left_pwm', qos)
        self._pub_right_pwm = self.create_publisher(Int16, 'arturito/wheel_right_pwm', qos)

        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, qos)
        self.create_subscription(Twist, self._wander_cmd_vel_topic, self._on_wander_cmd_vel, qos)
        self.create_subscription(Twist, self._clean_cmd_vel_topic, self._on_clean_cmd_vel, qos)
        self.create_subscription(String, 'arturito/raw_cmd', self._on_raw_cmd, qos)
        self.create_subscription(String, self._movement_topic, self._on_movement_cmd, qos)
        self.create_subscription(Bool, self._wake_topic, self._on_wake_signal, qos)
        self.create_subscription(Bool, self._wander_status_topic, self._on_wander_status, qos)
        self.create_subscription(Bool, self._clean_status_topic, self._on_clean_status, qos)

        self.create_service(Trigger, 'arturito/request_status', self._srv_request_status)
        self.create_service(Trigger, 'arturito/stop', self._srv_stop)
        self.create_service(SetBool, 'arturito/set_vacuum', self._srv_set_vacuum)
        self.create_service(SetBool, 'arturito/set_brush', self._srv_set_brush)
        self.create_service(Trigger, 'arturito/request_distance', self._srv_request_distance)

        if self._auto_request_status and self._status_request_period > 0.0:
            self.create_timer(self._status_request_period, self._request_status_timer_cb)

        self._open_serial()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            f'Arturito UART bridge ready on {self._port} @ {self._baudrate} bps'
        )
        if not self._enabled:
            self.get_logger().info('UART bridge arrancó deshabilitado y esperará wake word.')

    # region Serial helpers -------------------------------------------------
    def _open_serial(self) -> None:
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                return
            try:
                self._serial = serial.Serial(
                    port=self._port,
                    baudrate=self._baudrate,
                    timeout=self._read_timeout,
                    write_timeout=0.2,
                )
                self.get_logger().info('Serial port opened successfully')
            except SerialException as exc:
                self._serial = None
                now = time.monotonic()
                if now - self._last_serial_error_log > 5.0:
                    self.get_logger().warn(f'Unable to open serial port {self._port}: {exc}')
                    self._last_serial_error_log = now

    def _send_line(self, line: str) -> bool:
        payload = (line.strip() + '\n').encode('utf-8')
        with self._serial_lock:
            ser = self._serial
            if not ser or not ser.is_open:
                self.get_logger().warn('Cannot send command, serial port unavailable')
                return False
            try:
                ser.write(payload)
                return True
            except SerialException as exc:
                self.get_logger().warn(f'Write error: {exc}')
                try:
                    ser.close()
                except Exception:  # pragma: no cover - serial close best effort
                    pass
                self._serial = None
                return False

    def _reader_loop(self) -> None:
        buffer = bytearray()
        while not self._closing:
            with self._serial_lock:
                ser = self._serial
            if ser is None or not ser.is_open:
                self._open_serial()
                time.sleep(1.0)
                continue
            try:
                chunk = ser.read(256)
                if chunk:
                    buffer.extend(chunk)
                    while True:
                        newline_idx = buffer.find(b'\n')
                        carriage_idx = buffer.find(b'\r')
                        if newline_idx == -1 or (0 <= carriage_idx < newline_idx):
                            newline_idx = carriage_idx
                        if newline_idx == -1:
                            break
                        raw = buffer[:newline_idx].decode('utf-8', 'ignore').strip()
                        del buffer[: newline_idx + 1]
                        if raw:
                            self._handle_line(raw)
                else:
                    time.sleep(0.01)
            except SerialException as exc:
                self.get_logger().warn(f'Read error: {exc}')
                with self._serial_lock:
                    try:
                        if self._serial:
                            self._serial.close()
                    except Exception:
                        pass
                    self._serial = None
                time.sleep(1.0)

    # endregion -------------------------------------------------------------

    # region ROS callbacks --------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        if not self._enabled or self._clean_active or self._wander_active:
            return
        self._send_twist(msg)

    def _on_wander_cmd_vel(self, msg: Twist) -> None:
        if not self._enabled or self._clean_active or not self._wander_active:
            return
        self._send_twist(msg)

    def _on_clean_cmd_vel(self, msg: Twist) -> None:
        if not self._enabled or not self._clean_active:
            return
        self._send_twist(msg)

    def _on_wander_status(self, msg: Bool) -> None:
        self._wander_active = bool(msg.data)

    def _on_clean_status(self, msg: Bool) -> None:
        self._clean_active = bool(msg.data)

    def _send_twist(self, msg: Twist) -> None:
        v = clamp(msg.linear.x, -self._max_wheel_speed, self._max_wheel_speed)
        w = clamp(
            msg.angular.z,
            -2.0 * self._max_wheel_speed / max(self._base_width, 1e-3),
            2.0 * self._max_wheel_speed / max(self._base_width, 1e-3),
        )
        half_width = self._base_width * 0.5
        left_speed = v - w * half_width
        right_speed = v + w * half_width

        scale = self._max_wheel_pwm / max(self._max_wheel_speed, 1e-6)
        left_pwm = int(clamp(round(left_speed * scale), -self._max_wheel_pwm, self._max_wheel_pwm))
        right_pwm = int(clamp(round(right_speed * scale), -self._max_wheel_pwm, self._max_wheel_pwm))

        # Deadband compensation: si la rueda quiere moverse pero su PWM cae
        # debajo del umbral mecánico del motor, elevamos a min_wheel_pwm.
        min_pwm = self._min_wheel_pwm
        if min_pwm > 0:
            if 0 < left_pwm < min_pwm:
                left_pwm = min_pwm
            elif -min_pwm < left_pwm < 0:
                left_pwm = -min_pwm
            if 0 < right_pwm < min_pwm:
                right_pwm = min_pwm
            elif -min_pwm < right_pwm < 0:
                right_pwm = -min_pwm

        cmd = f'M {left_pwm} {right_pwm}'
        if self._send_line(cmd):
            self._publish_pwm(left_pwm, right_pwm)

    def _on_raw_cmd(self, msg: String) -> None:
        if not self._enabled:
            return
        payload = msg.data.strip()
        if payload:
            self._send_line(payload)

    def _on_movement_cmd(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            return
        if not self._enabled:
            self.get_logger().debug(
                f'Comando de movimiento ignorado (UART deshabilitada): {command}'
            )
            return
        if not self._send_line(command):
            self.get_logger().warn(f'No se pudo enviar comando de movimiento: {command}')

    def _on_wake_signal(self, msg: Bool) -> None:
        if msg.data and not self._enabled:
            self._enabled = True
            self.get_logger().info('UART bridge habilitado tras wake word.')

    def _srv_request_status(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        resp.success = self._send_line('SENS')
        resp.message = 'Requested status' if resp.success else 'Serial unavailable'
        return resp

    def _srv_request_distance(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        resp.success = self._send_line('DIS')
        resp.message = 'Requested distance' if resp.success else 'Serial unavailable'
        return resp

    def _srv_stop(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        resp.success = self._send_line('STOP')
        if resp.success:
            self._publish_pwm(0, 0)
        resp.message = 'Stop command sent' if resp.success else 'Serial unavailable'
        return resp

    def _srv_set_vacuum(self, req: SetBool.Request, resp: SetBool.Response) -> SetBool.Response:
        desired = bool(req.data)
        send = self._vacuum_state is None or self._vacuum_state != desired
        if send:
            send = self._send_line('V')
            resp.success = bool(send)
            if resp.success:
                self._vacuum_state = desired
                resp.message = f'Vacuum state set to {desired}'
            else:
                resp.message = 'Serial unavailable'
        else:
            resp.success = True
            resp.message = 'Vacuum already set as requested'
        return resp

    def _srv_set_brush(self, req: SetBool.Request, resp: SetBool.Response) -> SetBool.Response:
        desired = bool(req.data)
        send = self._brush_state is None or self._brush_state != desired
        if send:
            send = self._send_line('B')
            resp.success = bool(send)
            if resp.success:
                self._brush_state = desired
                resp.message = f'Brush state set to {desired}'
            else:
                resp.message = 'Serial unavailable'
        else:
            resp.success = True
            resp.message = 'Brush already set as requested'
        return resp

    def _request_status_timer_cb(self) -> None:
        self._send_line('SENS')

    # endregion -------------------------------------------------------------

    # region Parsing --------------------------------------------------------
    def _handle_line(self, line: str) -> None:
        prefix = None
        payload = line
        raw_json = False
        if line.startswith(self._status_prefix):
            prefix = self._status_prefix
        elif line.startswith(self._event_prefix):
            prefix = self._event_prefix
        elif line.startswith('ERR'):
            self.get_logger().warn(f'Error from firmware: {line}')
        elif line.startswith('{') and line.endswith('}'):
            # JSON crudo sin prefijo (lo manda el firmware actual). Lo tratamos
            # como status pero NO se le recortan caracteres del comienzo.
            prefix = self._status_prefix
            raw_json = True
        if prefix and not raw_json:
            _, _, payload = line.partition(' ')
            if not payload:
                payload = line[len(prefix) :].strip()

        if prefix == self._status_prefix or prefix is None:
            if payload.startswith('{'):
                self._pub_status_raw.publish(String(data=payload))
                self._handle_status_json(payload)
            else:
                self._pub_status_raw.publish(String(data=payload))
        elif prefix == self._event_prefix:
            if payload.startswith('{'):
                self._pub_events_raw.publish(String(data=payload))
                self._handle_event_json(payload)
            else:
                self._pub_events_raw.publish(String(data=payload))
        else:
            self._pub_status_raw.publish(String(data=line))

    def _handle_status_json(self, payload: str) -> None:
        try:
            data: Dict[str, Any] = json.loads(payload)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid JSON status: {exc}: {payload}')
            return

        now = self.get_clock().now().to_msg()

        voltage = self._get_float(data, ['vbat', 'battery_v', 'voltage'])
        current = self._get_float(data, ['ibat', 'battery_i', 'current'])
        if voltage is not None or current is not None:
            msg = BatteryState()
            msg.header.stamp = now
            msg.header.frame_id = self._battery_frame
            if voltage is not None:
                msg.voltage = voltage
            if current is not None:
                msg.current = current
            msg.present = True
            msg.percentage = float(data.get('battery_percent', -1.0))
            self._pub_battery.publish(msg)

        distance = self._get_distance_meters(data)
        if distance is not None:
            rng = Range()
            rng.header.stamp = now
            rng.header.frame_id = self._range_frame
            rng.radiation_type = Range.ULTRASOUND
            rng.field_of_view = self._ultra_fov
            rng.min_range = self._ultra_min
            rng.max_range = self._ultra_max
            rng.range = clamp(distance, self._ultra_min, self._ultra_max)
            self._pub_range.publish(rng)

        imu_msg = self._build_imu_message(data, now)
        if imu_msg is not None:
            self._pub_imu.publish(imu_msg)

        bumper_left_keys = ['bumper_left', 'bL', 'left_bumper']
        bumper_right_keys = ['bumper_right', 'bR', 'right_bumper']
        if self._swap_bumpers:
            bumper_left_keys, bumper_right_keys = bumper_right_keys, bumper_left_keys
        self._publish_bool_from_keys(data, self._pub_bumper_left, bumper_left_keys)
        self._publish_bool_from_keys(data, self._pub_bumper_right, bumper_right_keys)
        self._publish_bool_from_keys(data, self._pub_cliff, ['cliff', 'cliff_detected'])
        if self._publish_bool_from_keys(
            data, self._pub_vacuum_state, ['vacuum', 'vac', 'vacuum_on']
        ):
            self._vacuum_state = self._extract_bool(data, ['vacuum', 'vac', 'vacuum_on'])
        if self._publish_bool_from_keys(
            data, self._pub_brush_state, ['brush', 'brush_on']
        ):
            self._brush_state = self._extract_bool(data, ['brush', 'brush_on'])

        left_pwm = self._get_float(data, ['pwm_left', 'left_pwm', 'pwmL', 'L'])
        right_pwm = self._get_float(data, ['pwm_right', 'right_pwm', 'pwmR', 'R'])
        if left_pwm is not None or right_pwm is not None:
            self._publish_pwm(
                int(left_pwm if left_pwm is not None else self._last_pwm_left),
                int(right_pwm if right_pwm is not None else self._last_pwm_right),
            )

    def _handle_event_json(self, payload: str) -> None:
        try:
            data: Dict[str, Any] = json.loads(payload)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid JSON event: {exc}: {payload}')
            return
        event_name = str(data.get('event', '')).lower()
        pressed = bool(data.get('pressed', data.get('state', False)))
        left_events = ('bumpl', 'bumper_left', 'left_bumper')
        right_events = ('bumpr', 'bumper_right', 'right_bumper')
        if self._swap_bumpers:
            left_events, right_events = right_events, left_events
        if event_name in left_events:
            self._pub_bumper_left.publish(Bool(data=pressed))
        elif event_name in right_events:
            self._pub_bumper_right.publish(Bool(data=pressed))
        elif event_name in ('cliff', 'cliff_detected'):
            self._pub_cliff.publish(Bool(data=pressed))

    # endregion -------------------------------------------------------------

    def _publish_pwm(self, left: int, right: int) -> None:
        self._last_pwm_left = int(clamp(left, -self._max_wheel_pwm, self._max_wheel_pwm))
        self._last_pwm_right = int(clamp(right, -self._max_wheel_pwm, self._max_wheel_pwm))
        self._pub_left_pwm.publish(Int16(data=self._last_pwm_left))
        self._pub_right_pwm.publish(Int16(data=self._last_pwm_right))

    def _publish_bool_from_keys(
        self,
        data: Dict[str, Any],
        publisher,
        keys: list[str],
    ) -> bool:
        value = self._extract_bool(data, keys)
        if value is None:
            return False
        publisher.publish(Bool(data=value))
        return True

    def _extract_bool(self, data: Dict[str, Any], keys: list[str]) -> Optional[bool]:
        for key in keys:
            if key in data:
                val = data[key]
                if isinstance(val, bool):
                    return val
                if isinstance(val, (int, float)):
                    return bool(int(val))
                if isinstance(val, str):
                    val_l = val.strip().lower()
                    if val_l in {'1', 'true', 'yes', 'on'}:
                        return True
                    if val_l in {'0', 'false', 'no', 'off'}:
                        return False
        return None

    def _get_float(self, data: Dict[str, Any], keys: list[str]) -> Optional[float]:
        for key in keys:
            if key in data:
                try:
                    return float(data[key])
                except (TypeError, ValueError):
                    continue
        return None

    def _get_distance_meters(self, data: Dict[str, Any]) -> Optional[float]:
        distance = self._get_float(data, ['distance_m', 'ultrasonic_m', 'ultra_m'])
        if distance is not None:
            return distance
        # Firmware actual envía 'dist' en cm. Ojo: 0 significa "sin eco / fuera
        # de rango" — devolvemos None para que NO se publique Range fantasma
        # (sino el orchestrator dispararía startled pensando que hay algo a 5cm).
        distance = self._get_float(data, ['distance_cm', 'ultrasonic_cm', 'ultra_cm', 'dist'])
        if distance is not None:
            if distance <= 0.0:
                return None
            return distance / 100.0
        distance = self._get_float(data, ['distance_mm', 'ultrasonic_mm', 'ultra_mm'])
        if distance is not None:
            if distance <= 0.0:
                return None
            return distance / 1000.0
        distance = self._get_float(data, ['distance', 'ultrasonic'])
        return distance

    def _build_imu_message(self, data: Dict[str, Any], stamp) -> Optional[Imu]:
        accel_keys = ['ax', 'ay', 'az']
        gyro_keys = ['gx', 'gy', 'gz']
        has_raw = all(key in data for key in accel_keys + gyro_keys)
        accel_g_keys = ['ax_g', 'ay_g', 'az_g']
        gyro_dps_keys = ['gx_dps', 'gy_dps', 'gz_dps']
        if not has_raw and not (
            all(key in data for key in accel_g_keys) and all(key in data for key in gyro_dps_keys)
        ):
            return None

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._imu_frame
        if has_raw:
            imu.linear_acceleration.x = (
                float(data['ax']) / max(self._imu_accel_lsb_per_g, 1e-6) * 9.80665
            )
            imu.linear_acceleration.y = (
                float(data['ay']) / max(self._imu_accel_lsb_per_g, 1e-6) * 9.80665
            )
            imu.linear_acceleration.z = (
                float(data['az']) / max(self._imu_accel_lsb_per_g, 1e-6) * 9.80665
            )
            imu.angular_velocity.x = (
                float(data['gx']) / max(self._imu_gyro_lsb_per_dps, 1e-6) * math.pi / 180.0
            )
            imu.angular_velocity.y = (
                float(data['gy']) / max(self._imu_gyro_lsb_per_dps, 1e-6) * math.pi / 180.0
            )
            imu.angular_velocity.z = (
                float(data['gz']) / max(self._imu_gyro_lsb_per_dps, 1e-6) * math.pi / 180.0
            )
        else:
            imu.linear_acceleration.x = float(data['ax_g']) * 9.80665
            imu.linear_acceleration.y = float(data['ay_g']) * 9.80665
            imu.linear_acceleration.z = float(data['az_g']) * 9.80665
            imu.angular_velocity.x = float(data['gx_dps']) * math.pi / 180.0
            imu.angular_velocity.y = float(data['gy_dps']) * math.pi / 180.0
            imu.angular_velocity.z = float(data['gz_dps']) * math.pi / 180.0
        imu.orientation_covariance[0] = -1.0
        return imu

    def destroy_node(self) -> bool:
        self._closing = True
        self.get_logger().info('Shutting down Arturito UART bridge')
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        with self._serial_lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArturitoUARTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
