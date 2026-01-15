import fcntl
import json
import math
import random
import re
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool


class CleanQuick(Node):
    """Modo limpieza: activa aspiradora/escobillas y navega evitando obstáculos."""

    def __init__(self) -> None:
        super().__init__('clean_quick')

        self._cmd_vel_topic = str(
            self.declare_parameter('cmd_vel_topic', 'arturito/cmd_vel_clean').value
        )
        if not self._cmd_vel_topic.startswith('/'):
            self._cmd_vel_topic = f'/{self._cmd_vel_topic}'
        self._status_topic = str(
            self.declare_parameter('status_topic', 'arturito/status_raw').value
        )
        if not self._status_topic.startswith('/'):
            self._status_topic = f'/{self._status_topic}'
        self._mode_topic = str(
            self.declare_parameter(
                'mode_topic', '/assistant/mode/cleaning_quick'
            ).value
        )
        self._mode_enabled = bool(
            self.declare_parameter('mode_enabled', True).value
        )
        self._max_speed_mps = float(self.declare_parameter('max_speed_mps', 0.25).value)
        self._speed_pwm = float(self.declare_parameter('speed_pwm', 200.0).value)
        self._forward_speed = self._max_speed_mps * (self._speed_pwm / 255.0)
        self._reverse_speed = float(self.declare_parameter('reverse_speed_mps', 0.08).value)
        self._reverse_time = float(self.declare_parameter('reverse_time_sec', 0.3).value)
        self._turn_speed = float(self.declare_parameter('turn_speed_radps', 1.2).value)
        self._turn_angle_deg = float(self.declare_parameter('turn_angle_deg', 90.0).value)
        self._obstacle_distance_cm = float(
            self.declare_parameter('obstacle_distance_cm', 10.0).value
        )
        self._obstacle_min_distance_cm = float(
            self.declare_parameter('obstacle_min_distance_cm', 5.0).value
        )
        self._status_timeout = float(self.declare_parameter('status_timeout_sec', 1.0).value)
        self._use_yaw_turn = bool(self.declare_parameter('use_yaw_turn', True).value)
        self._turn_direction_mode = str(
            self.declare_parameter('turn_direction', 'random').value
        ).lower()
        self._dist_key = str(self.declare_parameter('dist_key', 'dist').value)
        self._yaw_key = str(self.declare_parameter('yaw_key', 'yaw').value)
        self._tilt_topic = str(self.declare_parameter('tilt_topic', '/head/tilt').value)
        if not self._tilt_topic.startswith('/'):
            self._tilt_topic = f'/{self._tilt_topic}'
        self._tilt_active_deg = float(self.declare_parameter('tilt_active_deg', 45.0).value)
        self._tilt_idle_deg = float(self.declare_parameter('tilt_idle_deg', 0.0).value)
        self._vacuum_service = str(
            self.declare_parameter('vacuum_service', 'arturito/set_vacuum').value
        )
        self._brush_service = str(
            self.declare_parameter('brush_service', 'arturito/set_brush').value
        )
        self._service_wait_timeout = float(
            self.declare_parameter('service_wait_timeout_sec', 1.5).value
        )

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._tilt_pub = self.create_publisher(Float32, self._tilt_topic, 10)
        self.create_subscription(String, self._status_topic, self._on_status, 10)
        if self._mode_topic:
            self.create_subscription(Bool, self._mode_topic, self._on_mode_change, 10)

        self._vacuum_client = self.create_client(SetBool, self._vacuum_service)
        self._brush_client = self.create_client(SetBool, self._brush_service)

        self._lock_file = self._acquire_singleton_lock()
        self._state = 'forward'
        self._turn_end_time: Optional[rclpy.time.Time] = None
        self._turn_direction = 1.0
        self._last_dist_cm: Optional[float] = None
        self._last_yaw_deg: Optional[float] = None
        self._last_status_stamp: Optional[rclpy.time.Time] = None
        self._turn_start_yaw: Optional[float] = None
        self._turn_target_angle_deg = self._turn_angle_deg
        self._reverse_end_time: Optional[rclpy.time.Time] = None
        self._bumper_left = False
        self._bumper_right = False
        self._bumper_left_prev = False
        self._bumper_right_prev = False
        self._pending_bumper_turn: Optional[float] = None
        self._active_bumper_turn: Optional[float] = None

        self._timer = self.create_timer(0.1, self._on_timer)
        self._publish_stop()
        self._publish_tilt(self._tilt_active_deg)
        self._set_service(self._vacuum_client, True, 'aspiradora')
        self._set_service(self._brush_client, True, 'escobillas')
        self.get_logger().info('Modo limpieza activado.')
        self.get_logger().info('CleanQuick listo: navegando y limpiando.')

    def _on_status(self, msg: String) -> None:
        payload = msg.data
        data = self._parse_status(payload)
        dist = None
        yaw = None
        bumper_left = None
        bumper_right = None
        if data is not None:
            dist = data.get(self._dist_key)
            yaw = data.get(self._yaw_key)
            bumper_left = data.get('bL')
            bumper_right = data.get('bR')
        else:
            dist = self._extract_number(payload, self._dist_key)
            yaw = self._extract_number(payload, self._yaw_key)
            bumper_left = self._extract_number(payload, 'bL')
            bumper_right = self._extract_number(payload, 'bR')

        if dist is not None:
            try:
                self._last_dist_cm = float(dist)
            except (TypeError, ValueError):
                pass
        if yaw is not None:
            try:
                self._last_yaw_deg = float(yaw)
            except (TypeError, ValueError):
                pass
        if bumper_left is not None:
            try:
                self._bumper_left = bool(int(bumper_left))
            except (TypeError, ValueError):
                pass
        if bumper_right is not None:
            try:
                self._bumper_right = bool(int(bumper_right))
            except (TypeError, ValueError):
                pass

        if (self._bumper_left and not self._bumper_left_prev) or (
            self._bumper_right and not self._bumper_right_prev
        ):
            if self._bumper_left and self._bumper_right:
                self._pending_bumper_turn = 1.0 if random.random() >= 0.5 else -1.0
            elif self._bumper_left:
                self._pending_bumper_turn = -1.0
            else:
                self._pending_bumper_turn = 1.0

        self._bumper_left_prev = self._bumper_left
        self._bumper_right_prev = self._bumper_right
        self._last_status_stamp = self.get_clock().now()

    def _on_mode_change(self, msg: Bool) -> None:
        if msg is None:
            return
        self._apply_mode(bool(msg.data))

    def _apply_mode(self, enabled: bool) -> None:
        if enabled == self._mode_enabled:
            return
        self._mode_enabled = enabled
        if enabled:
            self.get_logger().info("Modo limpieza rápida activado por comando de voz.")
            self._state = 'forward'
            self._publish_tilt(self._tilt_active_deg)
            self._set_service(self._vacuum_client, True, 'aspiradora')
            self._set_service(self._brush_client, True, 'escobillas')
            return
        self.get_logger().info("Modo limpieza rápida desactivado por comando de voz.")
        self._publish_stop()
        self._set_service(self._vacuum_client, False, 'aspiradora')
        self._set_service(self._brush_client, False, 'escobillas')
        self._publish_tilt(self._tilt_idle_deg)

    def _status_is_stale(self, now: rclpy.time.Time) -> bool:
        if self._last_status_stamp is None:
            return True
        age = now - self._last_status_stamp
        return age > Duration(seconds=self._status_timeout)

    def _should_turn(self) -> bool:
        if self._last_dist_cm is None or not math.isfinite(self._last_dist_cm):
            return False
        if not (self._obstacle_min_distance_cm <= self._last_dist_cm):
            return False
        return self._last_dist_cm <= self._obstacle_distance_cm

    def _pick_turn_direction(self) -> float:
        if self._turn_direction_mode == 'left':
            return 1.0
        if self._turn_direction_mode == 'right':
            return -1.0
        return 1.0 if random.random() >= 0.5 else -1.0

    def _start_turn(
        self,
        now: rclpy.time.Time,
        angle_deg: Optional[float] = None,
        direction: Optional[float] = None,
    ) -> None:
        self._turn_direction = direction if direction is not None else self._pick_turn_direction()
        self._turn_target_angle_deg = angle_deg if angle_deg is not None else self._turn_angle_deg
        self._turn_start_yaw = self._last_yaw_deg
        if not self._use_yaw_turn or self._turn_start_yaw is None:
            angle_rad = math.radians(self._turn_target_angle_deg)
            duration = angle_rad / max(abs(self._turn_speed), 1e-6)
            self._turn_end_time = now + Duration(seconds=duration)
        else:
            self._turn_end_time = None
        self._state = 'turning'

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_tilt(self, tilt_deg: float) -> None:
        self._tilt_pub.publish(Float32(data=float(tilt_deg)))

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if not self._mode_enabled:
            self._publish_stop()
            return
        if self._status_is_stale(now):
            self._publish_stop()
            self._state = 'stale'
            return

        if self._state == 'turning':
            if self._turn_end_time is not None:
                if now >= self._turn_end_time:
                    self._state = 'forward'
                    self._turn_end_time = None
            else:
                if self._turn_completed_by_yaw():
                    self._state = 'forward'
                    self._turn_start_yaw = None
        elif self._state == 'backing':
            if self._reverse_end_time is not None and now >= self._reverse_end_time:
                self._reverse_end_time = None
                direction = self._active_bumper_turn
                self._active_bumper_turn = None
                if direction is not None:
                    self._start_turn(now, angle_deg=45.0, direction=direction)
                else:
                    self._state = 'forward'
        elif self._state in ('forward', 'stale'):
            bumper_turn = self._consume_bumper_turn()
            if bumper_turn is not None:
                self._active_bumper_turn = bumper_turn
                self._state = 'backing'
                self._reverse_end_time = now + Duration(seconds=self._reverse_time)
            elif self._should_turn():
                self._start_turn(now)
            else:
                self._state = 'forward'

        twist = Twist()
        if self._state == 'turning':
            twist.angular.z = self._turn_direction * self._turn_speed
        elif self._state == 'backing':
            twist.linear.x = -abs(self._reverse_speed)
        elif self._state == 'forward':
            twist.linear.x = self._forward_speed

        self._cmd_pub.publish(twist)

    def _turn_completed_by_yaw(self) -> bool:
        if self._turn_start_yaw is None or self._last_yaw_deg is None:
            return False
        delta = self._angular_distance_deg(self._turn_start_yaw, self._last_yaw_deg)
        if self._turn_direction > 0.0:
            return delta >= self._turn_target_angle_deg
        return delta <= -self._turn_target_angle_deg

    def _consume_bumper_turn(self) -> Optional[float]:
        if self._pending_bumper_turn is not None:
            direction = self._pending_bumper_turn
            self._pending_bumper_turn = None
            return direction
        return None

    @staticmethod
    def _angular_distance_deg(start: float, end: float) -> float:
        return (end - start + 180.0) % 360.0 - 180.0

    def _parse_status(self, payload: str) -> Optional[dict]:
        raw = payload.strip()
        if not raw:
            return None
        if raw[0] != '{':
            start = raw.find('{')
            end = raw.rfind('}')
            if start != -1 and end != -1 and end > start:
                raw = raw[start:end + 1]
            else:
                raw = '{' + raw
                if not raw.endswith('}'):
                    raw += '}'
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return None

    @staticmethod
    def _extract_number(payload: str, key: str) -> Optional[float]:
        pattern = rf'"?{re.escape(key)}"?\s*:\s*(-?\d+(?:\.\d+)?)'
        match = re.search(pattern, payload)
        if not match:
            return None
        try:
            return float(match.group(1))
        except ValueError:
            return None

    def _set_service(self, client: rclpy.client.Client, desired: bool, name: str) -> None:
        if not client.wait_for_service(timeout_sec=self._service_wait_timeout):
            self.get_logger().warn(
                f'Servicio {name} no respondió tras {self._service_wait_timeout:.2f}s'
            )
            return
        req = SetBool.Request()
        req.data = desired
        future = client.call_async(req)

        def _callback(fut: rclpy.task.Future) -> None:
            try:
                resp = fut.result()
                self.get_logger().info(f'{name}: {resp.message}')
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(f'Error llamando {name}: {exc}')

        future.add_done_callback(_callback)

    def _acquire_singleton_lock(self):
        lock_path = Path('/tmp/clean_quick.lock')
        lock_file = lock_path.open('a+')
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            lock_file.close()
            raise RuntimeError('Ya existe una instancia de clean_quick en ejecucion.')
        return lock_file

    def destroy_node(self) -> bool:
        self._timer.cancel()
        self._publish_stop()
        self._publish_tilt(self._tilt_idle_deg)
        self._set_service(self._vacuum_client, False, 'aspiradora')
        self._set_service(self._brush_client, False, 'escobillas')
        if hasattr(self, '_lock_file') and self._lock_file:
            try:
                fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
            except OSError:
                pass
            try:
                self._lock_file.close()
            except OSError:
                pass
        self.get_logger().info('Modo limpieza desactivado.')
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CleanQuick()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
