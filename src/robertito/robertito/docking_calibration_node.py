import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose


def _expand_path(path: str) -> str:
    if not path:
        return path
    return os.path.expanduser(path.strip())


def _pose_to_dict(pose: Pose) -> Dict[str, float]:
    return {
        'position': {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
        },
        'orientation': {
            'x': float(pose.orientation.x),
            'y': float(pose.orientation.y),
            'z': float(pose.orientation.z),
            'w': float(pose.orientation.w),
        },
    }


@dataclass
class DockingRecord:
    distance_m: Optional[float] = None
    tag_id: Optional[str] = None
    confidence: Optional[float] = None
    bbox: Optional[Dict[str, Any]] = None
    pose: Optional[Dict[str, Any]] = None
    label: Optional[str] = None
    tag_size_m: Optional[float] = None
    timestamp: float = field(default_factory=time.time)
    record_id: str = field(default_factory=lambda: f'dock_{int(time.time() * 1000)}')

    def to_dict(self) -> Dict[str, Any]:
        return {
            'record_id': self.record_id,
            'timestamp': self.timestamp,
            'distance_m': self.distance_m,
            'tag_id': self.tag_id,
            'confidence': self.confidence,
            'label': self.label,
            'tag_size_m': self.tag_size_m,
            'bbox': self.bbox,
            'pose': self.pose,
        }


class DockingCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__('docking_calibration_node')
        self._range_topic = str(
            self.declare_parameter('range_topic', '/arturito/ultrasonic').value
        )
        self._tag_topic = str(
            self.declare_parameter('tag_topic', 'tag_detections').value
        )
        self._info_topic = str(
            self.declare_parameter('info_topic', '/robot_web/docking_info').value
        )
        self._calibrate_topic = str(
            self.declare_parameter('calibrate_topic', '/robot_web/docking_calibrate').value
        )
        self._status_topic = str(
            self.declare_parameter('status_topic', '/robot_web/docking_status').value
        )
        self._store_path = _expand_path(
            str(
                self.declare_parameter(
                    'store_path', '~/.ros/robertito_docking_info.json'
                ).value
            )
        )
        self._max_history = int(self.declare_parameter('max_history', 10).value)
        self._tag_size_m = float(
            self.declare_parameter('tag_size_m', 0.06).value
        )

        self._last_range: Optional[float] = None
        self._last_detection: Optional[Dict[str, Any]] = None
        self._records: List[DockingRecord] = []

        qos = 10
        self.create_subscription(Range, self._range_topic, self._on_range, qos)
        self.create_subscription(Detection2DArray, self._tag_topic, self._on_tags, qos)
        self.create_subscription(String, self._calibrate_topic, self._on_calibrate, qos)
        self._info_pub = self.create_publisher(String, self._info_topic, qos)
        self._status_pub = self.create_publisher(String, self._status_topic, qos)

        self._load_records()
        self._publish_info()
        self.get_logger().info(
            f'DockingCalibration listo: range={self._range_topic}, tags={self._tag_topic}, store={self._store_path}'
        )

    def _on_range(self, msg: Range) -> None:
        self._last_range = float(msg.range)

    def _on_tags(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            self._last_detection = None
            return
        detection = msg.detections[0]
        self._last_detection = self._parse_detection(detection)
        self._publish_status()

    def _parse_detection(self, detection: Any) -> Dict[str, Any]:
        bbox = detection.bbox
        center = bbox.center
        bbox_info = {
            'center': {
                'x': float(center.position.x),
                'y': float(center.position.y),
                'theta': float(center.theta),
            },
            'size_x': float(bbox.size_x),
            'size_y': float(bbox.size_y),
        }
        tag_id = detection.id or None
        confidence = None
        pose_info = None
        if detection.results:
            result = detection.results[0]
            hypothesis = result.hypothesis
            tag_id = tag_id or hypothesis.class_id
            confidence = float(hypothesis.score) if hypothesis.score else None
            if result.pose:
                pose_info = _pose_to_dict(result.pose.pose)
        return {'tag_id': tag_id, 'confidence': confidence, 'bbox': bbox_info, 'pose': pose_info}

    def _on_calibrate(self, msg: String) -> None:
        label = None
        if msg.data:
            try:
                payload = json.loads(msg.data)
                label = str(payload.get('label', '')).strip() or None
            except json.JSONDecodeError:
                label = str(msg.data).strip() or None
        record = DockingRecord(
            distance_m=self._last_range,
            tag_id=self._last_detection.get('tag_id') if self._last_detection else None,
            confidence=self._last_detection.get('confidence') if self._last_detection else None,
            bbox=self._last_detection.get('bbox') if self._last_detection else None,
            pose=self._last_detection.get('pose') if self._last_detection else None,
            label=label,
            tag_size_m=self._tag_size_m,
        )
        self._records.append(record)
        self._records = self._records[-self._max_history :]
        self._save_records()
        self._publish_info()
        self.get_logger().info(
            f'Calibración registrada: distance={record.distance_m} tag={record.tag_id} label={label}'
        )
        self._publish_status()

    def _publish_status(self) -> None:
        payload = {
            'distance_m': self._last_range,
            'tag_id': self._last_detection.get('tag_id') if self._last_detection else None,
            'confidence': self._last_detection.get('confidence') if self._last_detection else None,
            'timestamp': time.time(),
        }
        self._status_pub.publish(String(data=json.dumps(payload)))

    def _publish_info(self) -> None:
        payload = {
            'records': [record.to_dict() for record in self._records],
            'updated_at': time.time(),
        }
        self._info_pub.publish(String(data=json.dumps(payload)))

    def _load_records(self) -> None:
        if not self._store_path:
            return
        try:
            if not os.path.exists(self._store_path):
                return
            with open(self._store_path, 'r', encoding='utf-8') as handle:
                data = json.load(handle)
            raw = data.get('records', [])
            for entry in raw:
                try:
                    record = DockingRecord(
                        distance_m=entry.get('distance_m'),
                        tag_id=entry.get('tag_id'),
                        confidence=entry.get('confidence'),
                        bbox=entry.get('bbox'),
                        pose=entry.get('pose'),
                        label=entry.get('label'),
                        tag_size_m=entry.get('tag_size_m'),
                    )
                    record.timestamp = entry.get('timestamp', record.timestamp)
                    record.record_id = entry.get('record_id', record.record_id)
                    self._records.append(record)
                except Exception:
                    continue
            self._records = self._records[-self._max_history :]
        except Exception as exc:
            self.get_logger().warn(f'No se pudieron cargar registros de docking: {exc}')

    def _save_records(self) -> None:
        if not self._store_path:
            return
        try:
            directory = os.path.dirname(self._store_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            payload = {
                'records': [record.to_dict() for record in self._records],
                'updated_at': time.time(),
            }
            with open(self._store_path, 'w', encoding='utf-8') as handle:
                json.dump(payload, handle, indent=2)
        except Exception as exc:
            self.get_logger().warn(f'No se pudieron guardar registros de docking: {exc}')


def main() -> None:
    rclpy.init()
    node = DockingCalibrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
