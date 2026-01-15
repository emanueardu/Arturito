from __future__ import annotations

import json
import math
import os
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class OpenLoopOdom(Node):
    """Simple open-loop odometry derived from cmd_vel + IMU yaw."""

    def __init__(self) -> None:
        super().__init__('robertito_open_loop_odom')
        self._cmd_vel_topic = str(self.declare_parameter('cmd_vel_topic', '/cmd_vel').value)
        if not self._cmd_vel_topic.startswith('/'):
            self._cmd_vel_topic = f'/{self._cmd_vel_topic}'

        self._imu_topic = str(self.declare_parameter('imu_topic', '/arturito/imu').value)
        self._odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        self._base_frame = str(self.declare_parameter('base_frame', 'base_link').value)

        self._frequency = float(self.declare_parameter('publish_frequency', 25.0).value)
        self._cmd_timeout = float(self.declare_parameter('cmd_vel_timeout', 0.6).value)
        self._max_linear_speed = float(self.declare_parameter('max_linear_speed', 0.25).value)
        self._max_angular_speed = float(self.declare_parameter('max_angular_speed', 1.2).value)
        self._max_linear_accel = float(self.declare_parameter('max_linear_accel', 0.6).value)
        self._max_angular_accel = float(self.declare_parameter('max_angular_accel', 1.0).value)
        self._linear_deadband = float(self.declare_parameter('linear_deadband', 0.01).value)
        self._angular_deadband = float(self.declare_parameter('angular_deadband', 0.01).value)
        self._imu_bias_path = str(
            self.declare_parameter('gyro_bias_path', '~/.ros/robertito_gyro_bias.json').value
        )
        self._calibration_duration = float(
            self.declare_parameter('gyro_calibration_duration', 5.0).value
        )
        self._publish_filtered_imu = bool(
            self.declare_parameter('publish_filtered_imu', False).value
        )
        self._filtered_imu_topic = str(
            self.declare_parameter('filtered_imu_topic', '/imu/data').value
        )
        if self._publish_filtered_imu and not self._filtered_imu_topic.startswith('/'):
            self._filtered_imu_topic = f'/{self._filtered_imu_topic}'

        self._target_linear = 0.0
        self._target_angular = 0.0
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._yaw = 0.0
        self._gyro_bias: Optional[float] = self._load_bias()
        self._calibration_sum = 0.0
        self._calibration_samples = 0
        self._calibrated = self._gyro_bias is not None
        self._calibration_start_time: Optional[rclpy.time.Time] = None

        self._last_cmd_time: Optional[rclpy.time.Time] = None
        self._last_update_time: Optional[rclpy.time.Time] = None
        self._last_imu_time: Optional[rclpy.time.Time] = None

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._transform_broadcaster = TransformBroadcaster(self)
        self._imu_pub = (
            self.create_publisher(Imu, self._filtered_imu_topic, 5)
            if self._publish_filtered_imu
            else None
        )

        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Imu, self._imu_topic, self._on_imu, 20)
        self.create_timer(1.0 / max(self._frequency, 1e-3), self._on_timer)

        self.get_logger().info('Open-loop odometry ready (cmd_vel + IMU).')

    # region Callbacks -------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        self._last_cmd_time = self.get_clock().now()
        self._target_linear = max(-self._max_linear_speed, min(self._max_linear_speed, linear))
        self._target_angular = max(-self._max_angular_speed, min(self._max_angular_speed, angular))

    def _on_imu(self, msg: Imu) -> None:
        now = self.get_clock().now()
        if self._gyro_bias is None:
            self._calibrate_gyro(msg, now)
            if not self._calibrated:
                return
        rate = msg.angular_velocity.z - (self._gyro_bias or 0.0)
        if self._last_imu_time is not None:
            dt = (now - self._last_imu_time).nanoseconds * 1e-9
        else:
            dt = 0.0
        self._last_imu_time = now
        if dt > 1e-6:
            self._yaw = _wrap_angle(self._yaw + rate * dt)
        if self._imu_pub is not None:
            filtered = Imu()
            filtered.header = msg.header
            filtered.angular_velocity = msg.angular_velocity
            filtered.linear_acceleration = msg.linear_acceleration
            filtered.orientation = msg.orientation
            filtered.angular_velocity.z = rate
            self._imu_pub.publish(filtered)

    def _calibrate_gyro(self, msg: Imu, stamp: rclpy.time.Time) -> None:
        if self._calibration_start_time is None:
            self._calibration_start_time = stamp
        z_rate = float(msg.angular_velocity.z)
        self._calibration_sum += z_rate
        self._calibration_samples += 1
        elapsed = (stamp - self._calibration_start_time).nanoseconds * 1e-9
        if elapsed >= self._calibration_duration:
            self._gyro_bias = self._calibration_sum / max(self._calibration_samples, 1)
            self._calibrated = True
            self._save_bias(self._gyro_bias)
            self.get_logger().info(f'Gyro calibrated: bias={self._gyro_bias:.5f} rad/s')

    # endregion -------------------------------------------------------------

    # region Timer -----------------------------------------------------------

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self._last_update_time is None:
            dt = 1.0 / max(self._frequency, 1e-3)
        else:
            dt = (now - self._last_update_time).nanoseconds * 1e-9
        self._last_update_time = now
        dt = max(dt, 1e-5)

        if self._cmd_stale(now):
            self._target_linear = 0.0
            self._target_angular = 0.0

        self._current_linear = self._ramp(self._current_linear, self._target_linear, self._max_linear_accel, dt)
        self._current_angular = self._ramp(
            self._current_angular, self._target_angular, self._max_angular_accel, dt
        )

        linear = self._current_linear if abs(self._current_linear) >= self._linear_deadband else 0.0
        angular = self._current_angular if abs(self._current_angular) >= self._angular_deadband else 0.0

        self._pose_x += linear * dt * math.cos(self._yaw)
        self._pose_y += linear * dt * math.sin(self._yaw)

        if not self._calibrated and abs(angular) > 0.0:
            self._yaw = _wrap_angle(self._yaw + angular * dt)

        self._publish_odom(now, linear, angular)

    # endregion -------------------------------------------------------------

    # region Helpers ---------------------------------------------------------

    def _publish_odom(self, now: rclpy.time.Time, linear: float, angular: float) -> None:
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._pose_x
        odom.pose.pose.position.y = self._pose_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._quaternion_from_yaw(self._yaw)
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._pose_x
        transform.transform.translation.y = self._pose_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation
        self._transform_broadcaster.sendTransform(transform)

    def _cmd_stale(self, now: rclpy.time.Time) -> bool:
        if self._last_cmd_time is None:
            return False
        elapsed = (now - self._last_cmd_time).nanoseconds * 1e-9
        return elapsed > self._cmd_timeout

    @staticmethod
    def _ramp(current: float, target: float, accel: float, dt: float) -> float:
        if accel <= 0.0:
            return target
        delta = target - current
        limit = accel * dt
        if abs(delta) <= limit:
            return target
        return current + math.copysign(limit, delta)

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()
        half = 0.5 * yaw
        q.w = math.cos(half)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half)
        return q

    def _load_bias(self) -> Optional[float]:
        path = os.path.expanduser(self._imu_bias_path)
        if not path or not os.path.isfile(path):
            return None
        try:
            with open(path, 'r') as handle:
                data = json.load(handle)
            bias = float(data.get('bias_z'))
            self.get_logger().info(f'Loaded gyro bias {bias:.5f} from {path}')
            return bias
        except Exception as exc:
            self.get_logger().warn(f'No se pudo leer bias de gyro: {exc}')
            return None

    def _save_bias(self, value: float) -> None:
        path = os.path.expanduser(self._imu_bias_path)
        if not path:
            return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as handle:
                json.dump({'bias_z': value}, handle)
        except Exception as exc:
            self.get_logger().warn(f'No se pudo guardar bias de gyro: {exc}')

    # endregion -------------------------------------------------------------


def main() -> None:
    rclpy.init()
    node = OpenLoopOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
