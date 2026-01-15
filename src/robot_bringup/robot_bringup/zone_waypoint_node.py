import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def load_yaml_file(path: Path) -> dict:
    with path.open('r') as stream:
        return yaml.safe_load(stream)


def yaw_to_quaternion(theta: float):
    """Convert a planar yaw angle into quaternion components."""
    half = theta * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class ZoneWaypointNode(Node):
    """Publishes zone markers and accepts zone name commands to trigger Nav2 goals."""

    def __init__(self):
        super().__init__('zone_waypoint_node')
        package_share = Path(get_package_share_directory('robot_bringup'))

        zone_file = self.declare_parameter(
            'zone_file',
            str(package_share / 'maps' / 'apartamento_zones.yaml')
        ).value
        waypoint_file = self.declare_parameter(
            'waypoint_file',
            str(package_share / 'maps' / 'apartamento_waypoints.yaml')
        ).value

        self._auto_cleaning = bool(
            self.declare_parameter('auto_cleaning', True).value
        )
        vacuum_topic = str(
            self.declare_parameter('vacuum_topic', '/vacuum/enabled').value
        )
        brush_topic = str(
            self.declare_parameter('brush_topic', '/brush/enabled').value
        )
        catalog_topic = str(
            self.declare_parameter('zone_catalog_topic', '/robot_web/zone_catalog').value
        )
        state_topic = str(
            self.declare_parameter('zone_state_topic', '/robot_web/zone_state').value
        )
        goal_topic = str(
            self.declare_parameter('zone_goal_topic', '/zone_goal').value
        )

        zone_data = load_yaml_file(Path(zone_file))
        self._zones = zone_data.get('zones', {})
        self._map_metadata: Dict[str, Any] = {
            'resolution': zone_data.get('resolution'),
            'image_width_px': zone_data.get('image_width_px'),
            'image_height_px': zone_data.get('image_height_px'),
            'origin': zone_data.get('origin'),
        }
        waypoint_data = load_yaml_file(Path(waypoint_file))['waypoints']
        self._waypoints: Dict[str, dict] = {item['id']: item['pose'] for item in waypoint_data}
        self._dock_pose: Optional[dict] = self._waypoints.get('base')

        self._marker_pub = self.create_publisher(MarkerArray, 'zone_markers', 10)
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._catalog_pub = self.create_publisher(String, catalog_topic, latched_qos)
        self._state_pub = self.create_publisher(String, state_topic, 10)
        self._vacuum_pub = self.create_publisher(Bool, vacuum_topic, 10)
        self._brush_pub = self.create_publisher(Bool, brush_topic, 10)
        self._zone_sub = self.create_subscription(String, goal_topic, self._zone_goal_callback, 10)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._marker_timer = self.create_timer(1.0, self._publish_markers)
        self._queued_zone: Optional[str] = None
        self._active_zone: Optional[str] = None

        self._publish_catalog()
        self._publish_state('disponible', None)

        self.get_logger().info('Loaded %d zones and %d waypoints.',
                               len(self._zones), len(self._waypoints))

    def _publish_markers(self):
        markers = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        frame = 'map'

        for idx, (zone_id, zone) in enumerate(self._zones.items()):
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = frame
            marker.id = idx
            marker.action = Marker.ADD
            marker.type = Marker.LINE_STRIP
            marker.ns = 'zone_polygons'
            marker.scale.x = 0.02
            marker.color.a = 0.8
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.2

            for point in zone['polygon']:
                marker.points.append(Point(x=point[0], y=point[1], z=0.05))
            # Close polygon loop for visualization
            first = zone['polygon'][0]
            marker.points.append(Point(x=first[0], y=first[1], z=0.05))

            text_marker = Marker()
            text_marker.header.stamp = timestamp
            text_marker.header.frame_id = frame
            text_marker.id = idx + 1000
            text_marker.action = Marker.ADD
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.ns = 'zone_labels'
            text_marker.scale.z = 0.25
            text_marker.color.a = 0.9
            text_marker.color.r = 0.1
            text_marker.color.g = 0.1
            text_marker.color.b = 0.9

            centroid = zone['centroid']
            text_marker.pose.position.x = centroid[0]
            text_marker.pose.position.y = centroid[1]
            text_marker.pose.position.z = 0.2
            text_marker.text = zone_id

            markers.markers.append(marker)
            markers.markers.append(text_marker)

        if markers.markers:
            self._marker_pub.publish(markers)

    def _zone_goal_callback(self, msg: String):
        zone_id = msg.data.strip().lower()
        if zone_id not in self._waypoints:
            self.get_logger().warn('Zone "%s" not recognized. Available: %s',
                                   zone_id, ', '.join(self._waypoints.keys()))
            return

        self._active_zone = zone_id
        self._publish_state('en_cola', zone_id)

        if not self._nav_client.server_is_ready():
            self.get_logger().warn('Nav2 action server not ready yet, queuing request for "%s".',
                                   zone_id)
            self._publish_state('esperando_nav2', zone_id)
            self._queued_zone = zone_id
            self._nav_client.wait_for_server()
            if self._queued_zone:
                self._dispatch_goal(self._queued_zone)
                self._queued_zone = None
            return

        self._dispatch_goal(zone_id)

    def _dispatch_goal(self, zone_id: str):
        pose_data = self._waypoints[zone_id]

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose_data['x']
        goal.pose.pose.position.y = pose_data['y']
        quat = yaw_to_quaternion(pose_data.get('theta', 0.0))
        goal.pose.pose.orientation.x = quat[0]
        goal.pose.pose.orientation.y = quat[1]
        goal.pose.pose.orientation.z = quat[2]
        goal.pose.pose.orientation.w = quat[3]

        self.get_logger().info('Sending robot to zone "%s" (%.2f, %.2f, %.2f rad).',
                               zone_id, pose_data['x'], pose_data['y'], pose_data.get('theta', 0.0))

        self._nav_client.wait_for_server()
        self._set_cleaning(True)
        self._publish_state('navegando', zone_id)
        send_future = self._nav_client.send_goal_async(goal,
                                                       feedback_callback=self._nav_feedback_cb)
        send_future.add_done_callback(lambda fut: self._goal_response_cb(fut, zone_id))

    def _goal_response_cb(self, future, zone_id: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Zone "%s" goal rejected by Nav2.', zone_id)
            self._publish_state('rechazado', zone_id)
            self._set_cleaning(False)
            return
        self.get_logger().info('Zone "%s" goal accepted, waiting for result.', zone_id)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._goal_result_cb(fut, zone_id))

    def _nav_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback.current_pose.pose
        self.get_logger().debug('Current pose x=%.2f y=%.2f', feedback.position.x, feedback.position.y)

    def _goal_result_cb(self, future, zone_id: str):
        status = future.result().status
        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('Robot reached zone "%s".', zone_id)
            self._publish_state('completado', zone_id)
        else:
            self.get_logger().warn('Navigation to "%s" finished with status %d.', zone_id, status)
            self._publish_state('fallo', zone_id)
        self._set_cleaning(False)
        self._active_zone = None

    def _publish_catalog(self):
        payload: Dict[str, Any] = {
            'zones': [
                {
                    'id': zone_id,
                    'centroid': zone.get('centroid', [0.0, 0.0]),
                    'area_m2': zone.get('area_m2', 0.0),
                    'polygon': zone.get('polygon', []),
                }
                for zone_id, zone in self._zones.items()
            ]
        }
        metadata_clean = {
            key: value
            for key, value in self._map_metadata.items()
            if value is not None
        }
        if metadata_clean:
            payload['metadata'] = metadata_clean
        if self._dock_pose:
            payload['dock_pose'] = self._dock_pose
        self._catalog_pub.publish(String(data=json.dumps(payload)))

    def _publish_state(self, state: str, zone_id: Optional[str]):
        payload = {
            'estado': state,
            'zona': zone_id,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        }
        self._state_pub.publish(String(data=json.dumps(payload)))

    def _set_cleaning(self, active: bool) -> None:
        if not self._auto_cleaning:
            return
        msg = Bool(data=bool(active))
        self._vacuum_pub.publish(msg)
        self._brush_pub.publish(msg)


def main(args: List[str] = None):
    rclpy.init(args=args)
    node = ZoneWaypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
