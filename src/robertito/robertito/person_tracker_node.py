import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2D, Detection2DArray


class ArturitoPersonTracker(Node):
    """Track a detected person, turn the robot to follow, and drive eye expressions."""

    def __init__(self) -> None:
        super().__init__('arturito_person_tracker')

        self._detection_topic = str(
            self.declare_parameter('detection_topic', 'arturito/camera/faces').value
        )
        self._expression_topic = str(
            self.declare_parameter('expression_topic', 'arturito/eyes_expression').value
        )
        self._follow_mode_topic = str(
            self.declare_parameter(
                'follow_mode_topic', '/assistant/mode/follow_person'
            ).value
        )
        self._cmd_vel_topic = str(
            self.declare_parameter('cmd_vel_topic', '/cmd_vel/person_track').value
        )
        if not self._cmd_vel_topic.startswith('/'):
            self._cmd_vel_topic = f'/{self._cmd_vel_topic}'
        self._tilt_topic = str(
            self.declare_parameter('tilt_topic', '/head/tilt').value
        )
        if not self._tilt_topic.startswith('/'):
            self._tilt_topic = '/' + self._tilt_topic

        self._frame_width = float(self.declare_parameter('frame_width', 640.0).value)
        self._frame_height = float(self.declare_parameter('frame_height', 480.0).value)

        self._angular_gain = float(self.declare_parameter('angular_gain', 1.5).value)
        self._max_angular_speed = float(
            self.declare_parameter('max_angular_speed', 0.9).value
        )
        self._deadband = float(self.declare_parameter('deadband', 0.05).value)
        self._use_yaw_control = bool(
            self.declare_parameter('use_yaw_control', True).value
        )
        self._camera_fov_deg = float(
            self.declare_parameter('camera_fov_deg', 60.0).value
        )
        self._yaw_kp = float(self.declare_parameter('yaw_kp', 2.0).value)

        self._neutral_tilt = float(self.declare_parameter('neutral_tilt_deg', 10.0).value)
        self._tilt_gain = float(self.declare_parameter('tilt_gain_deg', 30.0).value)
        self._tilt_min = float(self.declare_parameter('tilt_min_deg', -20.0).value)
        self._tilt_max = float(self.declare_parameter('tilt_max_deg', 45.0).value)
        self._tilt_epsilon = float(self.declare_parameter('tilt_epsilon_deg', 0.5).value)

        self._expression_on_track = str(
            self.declare_parameter('track_expression', 'happy').value
        ).lower()
        self._expression_idle = str(
            self.declare_parameter('idle_expression', 'normal').value
        ).lower()
        self._lost_timeout = float(self.declare_parameter('lost_timeout_sec', 1.5).value)
        self._control_period = float(self.declare_parameter('control_period_sec', 0.1).value)
        self._wake_topic = str(
            self.declare_parameter('wake_topic', '/wake_word/detected').value
        )
        self._activate_on_wake = bool(
            self.declare_parameter('activate_on_wake', False).value
        )
        self._active = bool(self.declare_parameter('start_active', False).value)
        self._movement_topic = str(
            self.declare_parameter('movement_topic', '/movement_cmds').value
        )
        self._command_topic = str(
            self.declare_parameter('command_topic', '/tracker_control').value
        )
        self._status_topic = str(
            self.declare_parameter('status_topic', '/tracker_status').value
        )
        self._search_angular_speed = float(
            self.declare_parameter('search_angular_speed', 0.35).value
        )
        self._search_tilt = float(
            self.declare_parameter('search_tilt_deg', 30.0).value
        )
        self._search_expression = str(
            self.declare_parameter('search_expression', 'focus').value
        ).lower()
        self._search_direction = float(
            self.declare_parameter('search_direction', 1.0).value
        )
        self._search_speed_command = str(
            self.declare_parameter('search_speed_command', 'v200').value
        )
        self._stop_speed_command = str(
            self.declare_parameter('stop_speed_command', 'S').value
        )
        self._search_tilt_command_prefix = str(
            self.declare_parameter('search_tilt_command_prefix', 't').value.lower()
        )
        self._salute_cycles = int(self.declare_parameter('salute_cycles', 2).value)
        self._salute_step_sec = float(self.declare_parameter('salute_step_sec', 0.35).value)
        self._salute_return_tilt = float(
            self.declare_parameter('salute_return_tilt_deg', 0.0).value
        )
        self._vacuum_service = str(
            self.declare_parameter('vacuum_service', 'arturito/set_vacuum').value
        )
        self._brush_service = str(
            self.declare_parameter('brush_service', 'arturito/set_brush').value
        )
        self._target_area = float(
            self.declare_parameter('target_bbox_area', 22000.0).value
        )
        self._area_tolerance = float(
            self.declare_parameter('target_bbox_tolerance', 6000.0).value
        )
        self._approach_linear_speed = float(
            self.declare_parameter('approach_linear_speed', 0.12).value
        )
        self._flip_horizontal = bool(
            self.declare_parameter('flip_horizontal', False).value
        )
        self._flip_vertical = bool(
            self.declare_parameter('flip_vertical', False).value
        )
        self._imu_topic = str(self.declare_parameter('imu_topic', 'arturito/imu').value)
        self._range_topic = str(
            self.declare_parameter('range_topic', 'arturito/ultrasonic').value
        )
        self._target_distance = float(
            self.declare_parameter('target_distance_m', 0.5).value
        )
        self._distance_tolerance = float(
            self.declare_parameter('distance_tolerance_m', 0.08).value
        )
        self._distance_kp = float(self.declare_parameter('distance_kp', 0.6).value)
        self._max_linear_speed = float(
            self.declare_parameter('max_linear_speed', 0.2).value
        )
        self._distance_timeout = float(
            self.declare_parameter('distance_timeout_sec', 0.8).value
        )

        qos = rclpy.qos.QoSProfile(depth=10)
        self.create_subscription(
            Detection2DArray, self._detection_topic, self._on_detections, qos
        )
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._tilt_pub = self.create_publisher(Float32, self._tilt_topic, 10)
        self._expr_pub = self.create_publisher(String, self._expression_topic, 10)
        self._status_pub = self.create_publisher(Bool, self._status_topic, 10)
        self._movement_pub = (
            self.create_publisher(String, self._movement_topic, 10)
            if self._movement_topic
            else None
        )
        self._vacuum_client = self.create_client(SetBool, self._vacuum_service)
        self._brush_client = self.create_client(SetBool, self._brush_service)
        self.create_subscription(Bool, self._wake_topic, self._on_wake_signal, 10)
        self.create_subscription(String, self._command_topic, self._on_command, 10)
        if self._follow_mode_topic:
            self.create_subscription(
                Bool, self._follow_mode_topic, self._on_follow_mode, 10
            )
        if self._imu_topic:
            self.create_subscription(Imu, self._imu_topic, self._on_imu, 20)
        if self._range_topic:
            self.create_subscription(Range, self._range_topic, self._on_range, 10)

        self._last_detection: Optional[Tuple[float, float]] = None
        self._last_detection_time = self.get_clock().now() - Duration(seconds=999.0)
        self._current_tilt = self._neutral_tilt
        self._last_expression_sent: Optional[str] = None
        self._control_timer = self.create_timer(self._control_period, self._control_tick)
        self._searching = False
        self._search_tilt_target = self._neutral_tilt
        self._last_area: Optional[float] = None
        self._saluting = False
        self._salute_thread: Optional[threading.Thread] = None
        self._salute_lock = threading.Lock()
        self._cleaning_enabled = False
        self._current_yaw = 0.0
        self._last_imu_time: Optional[Time] = None
        self._target_yaw: Optional[float] = None
        self._last_range: Optional[float] = None
        self._last_range_time: Optional[Time] = None
        self._tilt_anchor: Optional[float] = None

        # Initialize hardware with neutral commands.
        self._publish_expression(self._expression_idle)
        self._publish_tilt(self._neutral_tilt, force=True)
        self._publish_velocity(0.0, 0.0)
        self._publish_status()
        self.get_logger().info(
            'Arturito person tracker listo: escuchando detecciones en %s'
            % self._detection_topic
        )
        if not self._active:
            self.get_logger().info(
                'Seguimiento inactivo; esperando comando "seguir" o activacion web.'
            )

    def _on_wake_signal(self, msg: Bool) -> None:
        if msg.data and self._activate_on_wake:
            self._set_active(True, reason='wake word')

    def _on_detections(self, msg: Detection2DArray) -> None:
        if not self._active:
            return
        selection = self._select_detection(msg)
        if selection is None:
            return
        detection, area = selection

        center_x = float(detection.bbox.center.position.x)
        center_y = float(detection.bbox.center.position.y)
        self._last_detection = (center_x, center_y)
        self._last_detection_time = self.get_clock().now()
        self._last_area = float(area)
        if self._searching:
            self.get_logger().debug('Persona detectada; deteniendo modo búsqueda.')
            self._searching = False
            if self._tilt_anchor is None:
                self._tilt_anchor = self._current_tilt
        if self._use_yaw_control:
            yaw_offset = self._offset_to_yaw(offset_x=None, center_x=center_x)
            if yaw_offset is not None:
                self._target_yaw = self._wrap_angle(self._current_yaw + yaw_offset)

    def _on_command(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if not text:
            return
        if 'dejar de seguir' in text:
            if self._active:
                self._set_active(False, reason='comando de voz')
        elif 'seguir' in text and not self._active:
            self._set_active(True, reason='comando de voz')
        elif 'saludar' in text:
            self._start_salute()
        elif 'dejar de limpiar' in text or 'parar limpiar' in text or 'detener limpiar' in text:
            self._set_cleaning(False)
        elif 'limpiar' in text:
            self._set_cleaning(True)

    def _on_follow_mode(self, msg: Bool) -> None:
        if msg is None:
            return
        self._set_active(bool(msg.data), reason='modo de voz')

    def _select_detection(self, msg: Detection2DArray) -> Optional[Tuple[Detection2D, float]]:
        best: Optional[Detection2D] = None
        best_area = -1.0
        for det in msg.detections:
            area = float(det.bbox.size_x) * float(det.bbox.size_y)
            if area > best_area:
                best = det
                best_area = area
        if best is None:
            return None
        return best, best_area

    def _set_active(self, value: bool, *, reason: str = '') -> None:
        if value == self._active:
            if value and not self._searching and self._last_detection is None:
                self._begin_search()
            return
        self._active = value
        self._publish_status()
        if value:
            self.get_logger().info('Person tracker activado%s.' % (f' ({reason})' if reason else ''))
            self._begin_search()
        else:
            self.get_logger().info('Person tracker desactivado%s.' % (f' ({reason})' if reason else ''))
            self._searching = False
            self._last_detection = None
            self._last_area = None
            self._target_yaw = None
            self._tilt_anchor = None
            self._publish_expression(self._expression_idle)
            self._publish_velocity(0.0, 0.0)
            self._publish_tilt(self._neutral_tilt, force=True)
            self._send_movement_command(self._stop_speed_command)

    def _publish_status(self) -> None:
        if self._status_pub:
            self._status_pub.publish(Bool(data=bool(self._active)))

    def _begin_search(self) -> None:
        self._searching = True
        self._last_detection = None
        self._last_area = None
        self._target_yaw = None
        self._tilt_anchor = None
        target = max(self._tilt_min, min(self._tilt_max, self._search_tilt))
        self._search_tilt_target = target
        self._publish_tilt(target, force=True)
        self._publish_expression(self._search_expression or self._expression_idle)
        self._publish_velocity(0.0, self._clamp_search_speed(self._search_direction * self._search_angular_speed))
        self._send_movement_command(self._search_speed_command)

    def _search_behavior(self) -> None:
        self._publish_expression(self._search_expression or self._expression_idle)
        self._publish_tilt(self._search_tilt_target)
        self._publish_velocity(0.0, self._clamp_search_speed(self._search_direction * self._search_angular_speed))
        self._send_movement_command(self._search_speed_command)

    def _clamp_search_speed(self, value: float) -> float:
        if value == 0.0:
            return 0.0
        return math.copysign(self._max_angular_speed, value)

    def _control_tick(self) -> None:
        if not self._active:
            return
        now = self.get_clock().now()
        target_visible = (
            self._last_detection is not None
            and (now - self._last_detection_time) < Duration(seconds=self._lost_timeout)
        )

        if not target_visible:
            if not self._searching:
                self._begin_search()
            else:
                self._search_behavior()
            return
        if self._searching:
            self._searching = False
            self._publish_velocity(0.0, 0.0)
            self._send_movement_command(self._stop_speed_command)
            if self._tilt_anchor is None:
                self._tilt_anchor = self._current_tilt

        center_x, center_y = self._last_detection
        offset_x = (center_x - self._frame_width / 2.0) / max(self._frame_width / 2.0, 1.0)
        offset_y = (center_y - self._frame_height / 2.0) / max(self._frame_height / 2.0, 1.0)
        if self._flip_horizontal:
            offset_x *= -1.0
        if self._flip_vertical:
            offset_y *= -1.0

        angular = 0.0
        if self._use_yaw_control and self._target_yaw is not None and self._last_imu_time is not None:
            yaw_error = self._wrap_angle(self._target_yaw - self._current_yaw)
            if abs(offset_x) < self._deadband:
                angular = 0.0
            else:
                angular = self._yaw_kp * yaw_error
            angular = max(-self._max_angular_speed, min(self._max_angular_speed, angular))
        else:
            if abs(offset_x) < self._deadband:
                angular = 0.0
            else:
                angular = self._angular_gain * offset_x
            angular = max(-self._max_angular_speed, min(self._max_angular_speed, angular))
        if angular != 0.0:
            angular = math.copysign(self._max_angular_speed, angular)

        linear = 0.0
        range_distance = self._latest_range(now)
        if range_distance is not None:
            error = range_distance - self._target_distance
            if abs(error) > self._distance_tolerance:
                linear = self._distance_kp * error
                linear = max(-self._max_linear_speed, min(self._max_linear_speed, linear))
            else:
                linear = 0.0
        elif self._last_area is not None:
            if self._last_area < (self._target_area - self._area_tolerance):
                linear = self._approach_linear_speed
            elif self._last_area > (self._target_area + self._area_tolerance):
                linear = 0.0
        if linear != 0.0:
            linear = math.copysign(self._max_linear_speed, linear)

        if self._tilt_anchor is None:
            self._tilt_anchor = self._current_tilt
        if abs(offset_y) < self._deadband:
            self._tilt_anchor = self._current_tilt
        target_tilt = self._tilt_anchor - (self._tilt_gain * offset_y)
        target_tilt = max(self._tilt_min, min(self._tilt_max, target_tilt))

        self._publish_expression(self._expression_on_track)
        self._publish_velocity(linear, angular)
        self._publish_tilt(target_tilt)

    def _publish_velocity(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self._cmd_pub.publish(twist)

    def _publish_tilt(self, target: float, *, force: bool = False) -> None:
        if self._saluting and not force:
            return
        if force or abs(target - self._current_tilt) > self._tilt_epsilon:
            self._current_tilt = target
            self._tilt_pub.publish(Float32(data=float(target)))
            if self._movement_pub is not None and self._search_tilt_command_prefix:
                command = f"{self._search_tilt_command_prefix}{int(round(target))}"
                self._send_movement_command(command)

    def _publish_expression(self, expression: str) -> None:
        expression = expression.lower()
        if expression == self._last_expression_sent:
            return
        self._last_expression_sent = expression
        self._expr_pub.publish(String(data=expression))

    def _send_movement_command(self, command: str) -> None:
        if not command or self._movement_pub is None:
            return
        msg = String()
        msg.data = command
        self._movement_pub.publish(msg)
        self.get_logger().debug(f'Comando de movimiento enviado: {command}')

    def _on_imu(self, msg: Imu) -> None:
        if math.isnan(msg.angular_velocity.z):
            return
        stamp = Time.from_msg(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now()
        if self._last_imu_time is None:
            self._last_imu_time = stamp
            return
        dt = (stamp - self._last_imu_time).nanoseconds * 1e-9
        if dt <= 0.0:
            self._last_imu_time = stamp
            return
        self._last_imu_time = stamp
        self._current_yaw = self._wrap_angle(self._current_yaw + msg.angular_velocity.z * dt)

    def _on_range(self, msg: Range) -> None:
        if math.isnan(msg.range) or msg.range <= 0.0:
            return
        stamp = Time.from_msg(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now()
        self._last_range = float(msg.range)
        self._last_range_time = stamp

    def _latest_range(self, now: Time) -> Optional[float]:
        if self._last_range is None or self._last_range_time is None:
            return None
        if (now - self._last_range_time).nanoseconds * 1e-9 > self._distance_timeout:
            return None
        return self._last_range

    def _offset_to_yaw(self, offset_x: Optional[float] = None, center_x: Optional[float] = None) -> Optional[float]:
        if offset_x is None:
            if center_x is None:
                return None
            offset_x = (center_x - self._frame_width / 2.0) / max(self._frame_width / 2.0, 1.0)
        if self._flip_horizontal:
            offset_x *= -1.0
        if abs(offset_x) < self._deadband:
            return 0.0
        fov_rad = math.radians(self._camera_fov_deg)
        return offset_x * (fov_rad / 2.0)

    @staticmethod
    def _wrap_angle(value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    def _start_salute(self) -> None:
        with self._salute_lock:
            if self._salute_thread is not None and self._salute_thread.is_alive():
                return
            self._salute_thread = threading.Thread(
                target=self._salute_sequence, name='salute-sequence', daemon=True
            )
            self._salute_thread.start()

    def _salute_sequence(self) -> None:
        self._saluting = True
        min_tilt = self._tilt_min
        max_tilt = self._tilt_max
        return_tilt = max(self._tilt_min, min(self._tilt_max, self._salute_return_tilt))
        step_delay = max(0.05, self._salute_step_sec)
        cycles = max(1, self._salute_cycles)

        try:
            for _ in range(cycles):
                self._publish_tilt(min_tilt, force=True)
                time.sleep(step_delay)
                self._publish_tilt(max_tilt, force=True)
                time.sleep(step_delay)
                self._publish_tilt(min_tilt, force=True)
                time.sleep(step_delay)
            self._publish_tilt(return_tilt, force=True)
        finally:
            self._saluting = False

    def _set_cleaning(self, enabled: bool) -> None:
        if enabled == self._cleaning_enabled:
            return
        self._cleaning_enabled = enabled
        self._call_setbool(self._vacuum_client, enabled, 'aspiradora')
        self._call_setbool(self._brush_client, enabled, 'cepillo')

    def _call_setbool(self, client: rclpy.client.Client, desired: bool, name: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warn(f'Servicio {name} no está disponible aún')
            return
        req = SetBool.Request()
        req.data = desired
        future = client.call_async(req)

        def _callback(fut: rclpy.task.Future) -> None:
            try:
                resp = fut.result()
                self.get_logger().info(f'{name}: {resp.message}')
            except Exception as exc:  # pragma: no cover - robustez
                self.get_logger().warn(f'Error llamando {name}: {exc}')

        future.add_done_callback(_callback)

    def destroy_node(self) -> bool:
        self.get_logger().info('Deteniendo person tracker; restaurando estado neutro')
        self._searching = False
        self._last_detection = None
        self._last_area = None
        self._publish_expression(self._expression_idle)
        self._publish_velocity(0.0, 0.0)
        self._publish_tilt(self._neutral_tilt, force=True)
        self._send_movement_command(self._stop_speed_command)
        if hasattr(self, '_control_timer') and self._control_timer is not None:
            self._control_timer.cancel()
        if self._salute_thread is not None and self._salute_thread.is_alive():
            self._salute_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArturitoPersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
