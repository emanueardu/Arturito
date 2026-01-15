from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool, Header


class ObstaclesNode(Node):
    """Publishes safety stops and obstacle clouds from sonar + bumpers."""

    def __init__(self) -> None:
        super().__init__('robertito_obstacles')
        self._sonar_topic = str(self.declare_parameter('sonar_topic', '/arturito/ultrasonic').value)
        self._bumper_left_topic = str(
            self.declare_parameter('bumper_left_topic', '/arturito/bumper_left').value
        )
        self._bumper_right_topic = str(
            self.declare_parameter('bumper_right_topic', '/arturito/bumper_right').value
        )
        self._safety_topic = str(self.declare_parameter('safety_topic', '/cmd_vel/safety_stop').value)
        self._pointcloud_topic = str(
            self.declare_parameter('pointcloud_topic', '/obstacle_points').value
        )
        self._pointcloud_frame = str(
            self.declare_parameter('pointcloud_frame', 'base_link').value
        )
        self._sonar_stop_distance = float(
            self.declare_parameter('sonar_stop_distance', 0.25).value
        )
        self._sonar_min = float(self.declare_parameter('sonar_min_range', 0.05).value)
        self._sonar_max = float(self.declare_parameter('sonar_max_range', 2.0).value)
        self._sonar_timeout = float(self.declare_parameter('sonar_timeout_sec', 1.0).value)
        self._bumper_distance = float(self.declare_parameter('bumper_distance', 0.12).value)
        self._bumper_angle = math.radians(
            float(self.declare_parameter('bumper_angle_deg', 60.0).value)
        )

        self._safety_rate = float(self.declare_parameter('safety_publish_rate', 15.0).value)
        self._cloud_rate = float(
            self.declare_parameter('cloud_publish_rate', 5.0).value
        )

        self._sonar_range: Optional[float] = None
        self._sonar_stamp: Optional[rclpy.time.Time] = None
        self._bumper_left = False
        self._bumper_right = False

        self._safety_pub = self.create_publisher(Twist, self._safety_topic, 10)
        self._cloud_pub = self.create_publisher(PointCloud2, self._pointcloud_topic, 5)
        self._zero_twist = Twist()

        self.create_subscription(Range, self._sonar_topic, self._on_sonar, 10)
        self.create_subscription(Bool, self._bumper_left_topic, self._on_bumper_left, 10)
        self.create_subscription(Bool, self._bumper_right_topic, self._on_bumper_right, 10)

        self.create_timer(1.0 / max(self._safety_rate, 1e-3), self._publish_safety)
        self.create_timer(1.0 / max(self._cloud_rate, 1e-3), self._publish_cloud)

        self.get_logger().info('Nodo de seguridad y obstáculos listo.')

    def _on_sonar(self, msg: Range) -> None:
        if msg.range <= 0.0 or msg.range != msg.range:
            return
        self._sonar_range = max(self._sonar_min, min(self._sonar_max, msg.range))
        self._sonar_stamp = self.get_clock().now()

    def _on_bumper_left(self, msg: Bool) -> None:
        self._bumper_left = bool(msg.data)

    def _on_bumper_right(self, msg: Bool) -> None:
        self._bumper_right = bool(msg.data)

    def _publish_safety(self) -> None:
        if self._should_stop():
            self._safety_pub.publish(self._zero_twist)

    def _should_stop(self) -> bool:
        if self._bumper_left or self._bumper_right:
            return True
        if self._sonar_range is None or self._sonar_stamp is None:
            return False
        age = (self.get_clock().now() - self._sonar_stamp).nanoseconds * 1e-9
        if age > self._sonar_timeout:
            return False
        return self._sonar_range <= self._sonar_stop_distance

    def _publish_cloud(self) -> None:
        points: List[List[float]] = []
        if self._sonar_range is not None:
            dist = max(self._sonar_min, min(self._sonar_max, self._sonar_range))
            points.append([dist, 0.0, 0.0])
        if self._bumper_left:
            x = self._bumper_distance * math.cos(self._bumper_angle)
            y = self._bumper_distance * math.sin(self._bumper_angle)
            points.append([x, y, 0.0])
        if self._bumper_right:
            x = self._bumper_distance * math.cos(self._bumper_angle)
            y = -self._bumper_distance * math.sin(self._bumper_angle)
            points.append([x, y, 0.0])
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._pointcloud_frame
        cloud = pc2.create_cloud_xyz32(header, points)
        self._cloud_pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = ObstaclesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
