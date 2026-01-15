from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose
import yaml

import tf2_ros


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert roll, pitch, yaw (radians) to quaternion components."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def load_known_tags(yaml_path: Optional[str]) -> Dict[int, dict]:
    if not yaml_path:
        return {}
    path = Path(yaml_path)
    if not path.exists():
        raise FileNotFoundError(f'Known tags file not found: {yaml_path}')
    data = yaml.safe_load(path.read_text())
    tags = {}
    for tag in data.get('tags', []):
        tags[int(tag['id'])] = tag
    return tags


class TagDetectorNode(Node):
    """Detects ArUco tags in the camera stream and publishes 2D detections + TF."""

    def __init__(self) -> None:
        super().__init__('tag_detector_node')

        self._bridge = CvBridge()
        self._dictionary_name = self.declare_parameter('aruco_dictionary', 'DICT_4X4_50').value
        self._tag_size_m = float(self.declare_parameter('tag_size_m', 0.048).value)
        self._camera_topic = self.declare_parameter('camera_topic', '/camera/image_raw').value
        self._camera_info_topic = self.declare_parameter('camera_info_topic', '/camera/camera_info').value
        self._camera_frame = self.declare_parameter('camera_frame', 'camera_link').value
        known_tags_file = self.declare_parameter('known_tags_file', '').value
        self._known_tags = load_known_tags(known_tags_file)

        self._dictionary = self._load_dictionary(self._dictionary_name)
        self._detector_parameters = self._create_detector_parameters()

        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        if self._known_tags:
            self._publish_static_tags(self._known_tags)

        self._detections_pub = self.create_publisher(Detection2DArray, 'tag_detections', 10)
        self._image_sub = self.create_subscription(Image, self._camera_topic, self._on_image, 10)
        self._camera_info_sub = self.create_subscription(CameraInfo, self._camera_info_topic, self._on_camera_info, 10)

        self.get_logger().info(
            f'Tag detector ready. Dictionary={self._dictionary_name} size={self._tag_size_m:.3f}m'
        )

    @staticmethod
    def _load_dictionary(name: str) -> cv2.aruco.Dictionary:
        if not name.startswith('DICT_'):
            name = f'DICT_{name}'
        if not hasattr(cv2.aruco, name):
            raise ValueError(f'Aruco dictionary {name} not found in OpenCV.')
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    @staticmethod
    def _create_detector_parameters():
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            return cv2.aruco.DetectorParameters_create()
        return cv2.aruco.DetectorParameters()

    def _publish_static_tags(self, known_tags: Dict[int, dict]) -> None:
        transforms: List[TransformStamped] = []
        for tag_id, tag in known_tags.items():
            pose = tag.get('pose', {})
            pos = pose.get('position', {})
            orientation = pose.get('orientation_rpy', {})
            roll = float(orientation.get('roll', 0.0))
            pitch = float(orientation.get('pitch', 0.0))
            yaw = float(orientation.get('yaw', 0.0))
            qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

            frame_id = tag.get('frame_id', f'tag_{tag_id}')
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = frame_id
            transform.transform.translation.x = float(pos.get('x', 0.0))
            transform.transform.translation.y = float(pos.get('y', 0.0))
            transform.transform.translation.z = float(pos.get('z', 0.0))
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)
        if transforms:
            self._static_broadcaster.sendTransform(transforms)
            self.get_logger().info(f'Published {len(transforms)} static tag transforms.')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_matrix = np.array(msg.k, dtype=float).reshape((3, 3))
        if msg.d and len(msg.d) >= 4:
            self._dist_coeffs = np.array(msg.d, dtype=float)
        else:
            self._dist_coeffs = np.zeros((5,))

    def _on_image(self, msg: Image) -> None:
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception:
            # If conversion fails (e.g. RGB image), fallback to BGR then convert.
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image, self._dictionary, parameters=self._detector_parameters
        )
        detections = Detection2DArray()
        detections.header = Header(stamp=msg.header.stamp, frame_id=self._camera_frame)

        if ids is None or len(ids) == 0:
            self._detections_pub.publish(detections)
            return

        ids = ids.flatten()
        pose_available = self._camera_matrix is not None and self._dist_coeffs is not None
        rvecs: Optional[np.ndarray] = None
        tvecs: Optional[np.ndarray] = None
        if pose_available:
            try:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self._tag_size_m, self._camera_matrix, self._dist_coeffs
                )
            except cv2.error as exc:
                self.get_logger().warn(f'Pose estimation error: {exc}')
                pose_available = False

        for idx, marker_id in enumerate(ids):
            detection = Detection2D()
            detection.header = detections.header
            detection.id = str(int(marker_id))

            corner = corners[idx].reshape((-1, 2))
            center = corner.mean(axis=0)
            width = np.linalg.norm(corner[0] - corner[1])
            height = np.linalg.norm(corner[1] - corner[2])
            bbox = BoundingBox2D()
            bbox.center.x = float(center[0])
            bbox.center.y = float(center[1])
            bbox.center.theta = 0.0
            bbox.size_x = float(width)
            bbox.size_y = float(height)
            detection.bbox = bbox

            if pose_available and rvecs is not None and tvecs is not None:
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = detection.id
                hypothesis.hypothesis.score = 1.0
                hypothesis.pose.pose.position.x = float(tvecs[idx][0][0])
                hypothesis.pose.pose.position.y = float(tvecs[idx][0][1])
                hypothesis.pose.pose.position.z = float(tvecs[idx][0][2])
                rotation_matrix, _ = cv2.Rodrigues(rvecs[idx])
                quat = self._rotation_matrix_to_quaternion(rotation_matrix)
                hypothesis.pose.pose.orientation.x = quat[0]
                hypothesis.pose.pose.orientation.y = quat[1]
                hypothesis.pose.pose.orientation.z = quat[2]
                hypothesis.pose.pose.orientation.w = quat[3]
                detection.results.append(hypothesis)

                child_frame = self._known_tags.get(int(marker_id), {}).get('frame_id', f'tag_{marker_id}')
                self._publish_detection_tf(
                    child_frame, msg.header.stamp, tvecs[idx][0], rotation_matrix
                )

            detections.detections.append(detection)

        self._detections_pub.publish(detections)

    def _rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray) -> Tuple[float, float, float, float]:
        m = rotation_matrix
        trace = np.trace(m)
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2
            qw = 0.25 * s
            qx = (m[2, 1] - m[1, 2]) / s
            qy = (m[0, 2] - m[2, 0]) / s
            qz = (m[1, 0] - m[0, 1]) / s
        else:
            if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
                s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
                qw = (m[2, 1] - m[1, 2]) / s
                qx = 0.25 * s
                qy = (m[0, 1] + m[1, 0]) / s
                qz = (m[0, 2] + m[2, 0]) / s
            elif m[1, 1] > m[2, 2]:
                s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
                qw = (m[0, 2] - m[2, 0]) / s
                qx = (m[0, 1] + m[1, 0]) / s
                qy = 0.25 * s
                qz = (m[1, 2] + m[2, 1]) / s
            else:
                s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
                qw = (m[1, 0] - m[0, 1]) / s
                qx = (m[0, 2] + m[2, 0]) / s
                qy = (m[1, 2] + m[2, 1]) / s
                qz = 0.25 * s
        return qx, qy, qz, qw

    def _publish_detection_tf(
        self,
        child_frame: str,
        stamp,
        tvec: np.ndarray,
        rotation_matrix: np.ndarray,
    ) -> None:
        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(rotation_matrix)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._camera_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(tvec[0])
        transform.transform.translation.y = float(tvec[1])
        transform.transform.translation.z = float(tvec[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = TagDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
