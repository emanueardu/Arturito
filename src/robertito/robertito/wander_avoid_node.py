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
from std_msgs.msg import Float32, String


class WanderAvoid(Node):
    """Simple wander node: moves forward until close to obstacle, then turns 90°."""

    def __init__(self) -> None:
        super().__init__('wander_avoid')

        self._cmd_vel_topic = str(self.declare_parameter('cmd_vel_topic', '/cmd_vel/wander').value)
        if not self._cmd_vel_topic.startswith('/'):
            self._cmd_vel_topic = f'/{self._cmd_vel_topic}'
        self._status_topic = str(
            self.declare_parameter('status_topic', 'arturito/status_raw').value
        )
        if not self._status_topic.startswith('/'):
            self._status_topic = f'/{self._status_topic}'
        self._forward_speed = float(self.declare_parameter('forward_speed_mps', 0.25).value)
        self._reverse_speed = float(self.declare_parameter('reverse_speed_mps', 0.08).value)
        self._reverse_time = float(self.declare_parameter('reverse_time_sec', 0.3).value)
        self._turn_speed = float(self.declare_parameter('turn_speed_radps', 1.2).value)
        self._turn_angle_deg = float(self.declare_parameter('turn_angle_deg', 90.0).value)
        self._obstacle_distance_cm = float(
            self.declare_parameter('obstacle_distance_cm', 10.0).value
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

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._tilt_pub = self.create_publisher(Float32, self._tilt_topic, 10)
        self.create_subscription(String, self._status_topic, self._on_status, 10)

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
        self.get_logger().info('Modo navegacion activado.')
        self.get_logger().info('WanderAvoid listo: avanzará y esquivará obstáculos a 10 cm.')

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

    def _status_is_stale(self, now: rclpy.time.Time) -> bool:
        if self._last_status_stamp is None:
            return True
        age = now - self._last_status_stamp
        return age > Duration(seconds=self._status_timeout)

    def _should_turn(self) -> bool:
        if self._last_dist_cm is None or not math.isfinite(self._last_dist_cm):
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
        side = 'izquierda' if self._turn_direction > 0.0 else 'derecha'
        angle = int(round(self._turn_target_angle_deg))
        self.get_logger().info(f'Girando {angle}° a {side}.')

    def _publish_stop(self) -> None:
        twist = Twist()
        self._cmd_pub.publish(twist)

    def _publish_tilt(self, tilt_deg: float) -> None:
        self._tilt_pub.publish(Float32(data=float(tilt_deg)))

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self._status_is_stale(now):
            self._publish_stop()
            if self._state != 'stale':
                self.get_logger().warn('Sin datos recientes de estado, deteniendo.')
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
        else:
            twist = Twist()

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
        # Returns signed shortest angular distance in degrees in [-180, 180).
        delta = (end - start + 180.0) % 360.0 - 180.0
        return delta

    def destroy_node(self) -> bool:
        self._timer.cancel()
        self._publish_stop()
        self._publish_tilt(self._tilt_idle_deg)
        if hasattr(self, '_lock_file') and self._lock_file:
            try:
                fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
            except OSError:
                pass
            try:
                self._lock_file.close()
            except OSError:
                pass
        return super().destroy_node()

    def _acquire_singleton_lock(self):
        lock_path = Path('/tmp/wander_avoid.lock')
        lock_file = lock_path.open('a+')
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            lock_file.close()
            raise RuntimeError('Ya existe una instancia de wander_avoid en ejecucion.')
        return lock_file


def main() -> None:
    rclpy.init()
    node = WanderAvoid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
