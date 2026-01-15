from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _angle_diff(a: float, b: float) -> float:
    return _wrap_angle(a - b)


@dataclass
class Measurement:
    x: float
    y: float
    yaw: Optional[float]
    source: str
    confidence: Optional[float] = None
    timestamp: float = time.time()


class MapOdomBroadcaster(Node):
    """Updates map->odom using periodic WiFi and docking pose estimates."""

    def __init__(self) -> None:
        super().__init__('robertito_map_odom_broadcaster')
        self._wifi_topic = str(self.declare_parameter('wifi_pose_topic', '/robot_web/wifi_pose').value)
        self._docking_topic = str(self.declare_parameter('docking_info_topic', '/robot_web/docking_info').value)
        self._status_topic = str(
            self.declare_parameter('status_topic', '/localization/status').value
        )
        self._map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self._odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        self._base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self._alpha = float(self.declare_parameter('alpha', 0.4).value)
        self._strong_alpha = float(self.declare_parameter('strong_alpha', 0.8).value)
        self._max_jump_xy = float(self.declare_parameter('max_jump_xy', 0.8).value)
        max_jump_yaw_deg = float(self.declare_parameter('max_jump_yaw_deg', 35.0).value)
        self._max_jump_yaw = math.radians(max_jump_yaw_deg)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._map_odom_x = 0.0
        self._map_odom_y = 0.0
        self._map_odom_yaw = 0.0
        self._initialized = False
        self._map_base_yaw = 0.0

        self._status_pub = (
            self.create_publisher(String, self._status_topic, 5)
            if self._status_topic
            else None
        )

        self.create_subscription(String, self._wifi_topic, self._on_wifi_pose, 5)
        self.create_subscription(String, self._docking_topic, self._on_docking_info, 5)
        self.get_logger().info('Map->Odom broadcaster listo.')

    def _on_wifi_pose(self, msg: String) -> None:
        measurement = self._parse_wifi(msg.data)
        if measurement:
            self._handle_measurement(measurement)

    def _on_docking_info(self, msg: String) -> None:
        measurement = self._parse_docking(msg.data)
        if measurement:
            self._handle_measurement(measurement)

    def _parse_wifi(self, payload: str) -> Optional[Measurement]:
        try:
            data: dict[str, Any] = json.loads(payload)
        except json.JSONDecodeError:
            self.get_logger().warn('WiFi pose inválida, ignorando.')
            return None
        x = data.get('x')
        y = data.get('y')
        if x is None or y is None:
            return None
        yaw = data.get('theta')
        confidence = data.get('confidence')
        timestamp = data.get('timestamp', time.time())
        return Measurement(
            x=float(x),
            y=float(y),
            yaw=float(yaw) if yaw is not None else self._map_base_yaw,
            source='wifi',
            confidence=float(confidence) if confidence is not None else None,
            timestamp=float(timestamp),
        )

    def _parse_docking(self, payload: str) -> Optional[Measurement]:
        try:
            data: dict[str, Any] = json.loads(payload)
        except json.JSONDecodeError:
            self.get_logger().warn('Docking info inválida, ignorando.')
            return None
        records = data.get('records', [])
        for record in reversed(records):
            pose = record.get('pose')
            if not pose:
                continue
            pos = pose.get('position') or {}
            ori = pose.get('orientation') or {}
            x = pos.get('x')
            y = pos.get('y')
            if x is None or y is None:
                continue
            yaw = None
            if ori:
                yaw = self._quaternion_to_yaw(ori)
            return Measurement(
                x=float(x),
                y=float(y),
                yaw=yaw if yaw is not None else self._map_base_yaw,
                source='docking',
                confidence=record.get('confidence'),
                timestamp=record.get('timestamp', time.time()),
            )
        return None

    def _handle_measurement(self, measurement: Measurement) -> None:
        transform = self._lookup_odom_base()
        if transform is None:
            return
        odom_x = transform.transform.translation.x
        odom_y = transform.transform.translation.y
        odom_yaw = self._quaternion_to_yaw(transform.transform.rotation)
        map_base_yaw = measurement.yaw if measurement.yaw is not None else self._map_base_yaw
        new_yaw = _wrap_angle(map_base_yaw - odom_yaw)
        cos_mo = math.cos(new_yaw)
        sin_mo = math.sin(new_yaw)
        x_offset = cos_mo * odom_x - sin_mo * odom_y
        y_offset = sin_mo * odom_x + cos_mo * odom_y
        new_x = measurement.x - x_offset
        new_y = measurement.y - y_offset

        if self._initialized and not self._validate_jump(new_x, new_y, new_yaw):
            return

        alpha = self._strong_alpha if measurement.source == 'docking' else self._alpha
        if not self._initialized:
            alpha = 1.0
        self._map_odom_x += alpha * (new_x - self._map_odom_x)
        self._map_odom_y += alpha * (new_y - self._map_odom_y)
        yaw_diff = _angle_diff(new_yaw, self._map_odom_yaw)
        self._map_odom_yaw = _wrap_angle(self._map_odom_yaw + alpha * yaw_diff)
        self._map_base_yaw = map_base_yaw

        self._initialized = True
        self._broadcast_transform()
        self._publish_status(measurement)

    def _validate_jump(self, x: float, y: float, yaw: float) -> bool:
        distance = math.hypot(x - self._map_odom_x, y - self._map_odom_y)
        if distance > self._max_jump_xy:
            self.get_logger().warn(
                f'Salto XY detectado ({distance:.2f} m), descartando medición.'
            )
            return False
        yaw_delta = abs(_angle_diff(yaw, self._map_odom_yaw))
        if yaw_delta > self._max_jump_yaw:
            self.get_logger().warn(
                f'Salto yaw detectado ({math.degrees(yaw_delta):.1f}°), descartando medición.'
            )
            return False
        return True

    def _lookup_odom_base(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, self.get_clock().now()
            )
        except TransformException as exc:
            self.get_logger().warn(f'No se encontró tf {self._odom_frame}->{self._base_frame}: {exc}')
            return None

    def _broadcast_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = self._map_odom_x
        transform.transform.translation.y = self._map_odom_y
        transform.transform.translation.z = 0.0
        half = 0.5 * self._map_odom_yaw
        transform.transform.rotation.w = math.cos(half)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(half)
        self._tf_broadcaster.sendTransform(transform)

    def _publish_status(self, measurement: Measurement) -> None:
        if self._status_pub is None:
            return
        payload = {
            'source': measurement.source,
            'confidence': measurement.confidence,
            'timestamp': measurement.timestamp,
            'map_odom': {
                'x': self._map_odom_x,
                'y': self._map_odom_y,
                'yaw': self._map_odom_yaw,
            },
        }
        self._status_pub.publish(String(data=json.dumps(payload)))

    @staticmethod
    def _quaternion_to_yaw(quat: Any) -> float:
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main() -> None:
    rclpy.init()
    node = MapOdomBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
