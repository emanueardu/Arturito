import math
from enum import Enum, auto
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

import tf2_ros
from tf2_ros import TransformException

import yaml


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def yaw_to_quaternion(theta: float):
    half = theta * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class DockState(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    SEARCHING = auto()
    DOCKING = auto()
    DOCKED = auto()
    FAILED = auto()


class GoToBaseNode(Node):
    """State machine that sends the robot to its base and performs ArUco-guided docking."""

    def __init__(self) -> None:
        super().__init__('go_to_base_node')

        package_share = Path(get_package_share_directory('robot_bringup'))
        default_waypoint = package_share / 'maps' / 'apartamento_waypoints.yaml'

        self._waypoint_file = Path(
            self.declare_parameter('waypoint_file', str(default_waypoint)).value
        )
        self._base_id: str = self.declare_parameter('base_id', 'base').value
        self._target_tag_id: int = int(self.declare_parameter('target_tag_id', 0).value)
        self._tag_frame: str = self.declare_parameter(
            'tag_frame', f'tag_{self._target_tag_id}'
        ).value
        self._dock_distance: float = float(self.declare_parameter('dock_distance', 0.02).value)
        self._range_timeout = Duration(seconds=float(self.declare_parameter('range_timeout', 2.0).value))
        self._detection_timeout = Duration(
            seconds=float(self.declare_parameter('detection_timeout', 1.5).value)
        )
        self._align_tolerance: float = float(
            self.declare_parameter('align_tolerance', 0.05).value
        )
        self._max_linear_speed: float = float(
            self.declare_parameter('max_linear_speed', 0.15).value
        )
        self._min_linear_speed: float = float(
            self.declare_parameter('min_linear_speed', 0.02).value
        )
        self._max_angular_speed: float = float(
            self.declare_parameter('max_angular_speed', 0.4).value
        )
        self._linear_kp: float = float(self.declare_parameter('linear_kp', 0.6).value)
        self._angular_kp: float = float(self.declare_parameter('angular_kp', 1.8).value)
        self._search_angular_speed: float = float(
            self.declare_parameter('search_angular_speed', 0.25).value
        )
        self._distance_request_period = Duration(
            seconds=float(self.declare_parameter('distance_request_period', 1.0).value)
        )
        self._map_frame: str = self.declare_parameter('map_frame', 'map').value
        self._base_frame: str = self.declare_parameter('base_frame', 'base_link').value

        self._base_pose = self._load_base_pose(self._waypoint_file, self._base_id)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, 'go_to_base/status', 10)

        qos = QoSPresetProfiles.SYSTEM_DEFAULT.value
        self._tag_sub = self.create_subscription(
            Detection2DArray, 'tag_detections', self._detections_cb, qos
        )
        self._range_sub = self.create_subscription(
            Range, 'arturito/ultrasonic', self._range_cb, qos
        )

        self._distance_client = self.create_client(Trigger, 'arturito/request_distance')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._state = DockState.IDLE
        self._nav_goal_handle = None
        self._nav_result_future = None

        self._last_range: Optional[float] = None
        self._last_range_stamp: Optional[Time] = None
        self._last_detection_stamp: Optional[Time] = None
        self._distance_future = None
        self._last_distance_request: Optional[Time] = None

        self._control_timer = self.create_timer(0.1, self._control_loop)
        self._nav_start_timer = self.create_timer(1.0, self._attempt_nav_start)

        self.get_logger().info(
            f'Ir a base inicializado -> waypoint "{self._base_id}", aruco id={self._target_tag_id} frame={self._tag_frame}'
        )

    # region Parameter helpers -------------------------------------------------

    def _load_base_pose(self, waypoint_file: Path, base_id: str) -> dict:
        if not waypoint_file.exists():
            raise FileNotFoundError(f'Waypoint file not found: {waypoint_file}')
        data = yaml.safe_load(waypoint_file.read_text())
        waypoints = data.get('waypoints', [])
        for item in waypoints:
            if str(item.get('id')) == str(base_id):
                pose = item.get('pose', {})
                if 'x' in pose and 'y' in pose:
                    return pose
        raise ValueError(f'Base id "{base_id}" not found in {waypoint_file}')

    # endregion ----------------------------------------------------------------

    # region Navigation --------------------------------------------------------

    def _attempt_nav_start(self) -> None:
        if self._state != DockState.IDLE:
            self._nav_start_timer.cancel()
            return
        if not self._nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().debug('Waiting for Nav2 action server...')
            return
        self._send_nav_goal()
        self._set_state(DockState.NAVIGATING)
        self._nav_start_timer.cancel()

    def _send_nav_goal(self) -> None:
        pose = self._base_pose
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose.get('x', 0.0))
        goal.pose.pose.position.y = float(pose.get('y', 0.0))
        theta = float(pose.get('theta', 0.0))
        quat = yaw_to_quaternion(theta)
        goal.pose.pose.orientation.x = quat[0]
        goal.pose.pose.orientation.y = quat[1]
        goal.pose.pose.orientation.z = quat[2]
        goal.pose.pose.orientation.w = quat[3]

        self.get_logger().info(
            f'Enviando navegación hacia la base "{self._base_id}" '
            f'(x={goal.pose.pose.position.x:.2f}, y={goal.pose.pose.position.y:.2f}, yaw={theta:.2f} rad).'
        )
        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rechazó la meta hacia la base.')
            self._set_state(DockState.FAILED)
            return
        self.get_logger().info('Meta de navegación aceptada, esperando resultado...')
        self._nav_goal_handle = goal_handle
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Error esperando resultado de Nav2: {exc}')
            self._set_state(DockState.FAILED)
            return

        status = result.status
        self._nav_goal_handle = None
        self._nav_result_future = None
        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('Navegación completada, iniciando búsqueda del ArUco.')
            self._set_state(DockState.SEARCHING)
        else:
            self.get_logger().warn(f'Nav2 finalizó con estado {status}, abortando docking.')
            self._set_state(DockState.FAILED)

    def _nav_feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback.current_pose.pose
        self.get_logger().debug(
            f'Progreso navegación x={feedback.position.x:.2f} y={feedback.position.y:.2f}'
        )

    # endregion ----------------------------------------------------------------

    # region Subscriptions -----------------------------------------------------

    def _detections_cb(self, msg: Detection2DArray) -> None:
        for detection in msg.detections:
            if detection.id == str(self._target_tag_id):
                self._last_detection_stamp = self.get_clock().now()
                if self._state == DockState.SEARCHING:
                    self.get_logger().info('ArUco detectado, iniciando maniobra de docking.')
                    self._set_state(DockState.DOCKING)
                return

    def _range_cb(self, msg: Range) -> None:
        self._last_range = msg.range
        self._last_range_stamp = self.get_clock().now()

    # endregion ----------------------------------------------------------------

    # region Control loop ------------------------------------------------------

    def _control_loop(self) -> None:
        if self._state == DockState.NAVIGATING:
            return
        if self._state == DockState.SEARCHING:
            self._publish_twist(0.0, self._search_angular_speed)
            return
        if self._state == DockState.DOCKING:
            self._run_docking_control()
            return
        if self._state == DockState.DOCKED or self._state == DockState.FAILED:
            self._publish_twist(0.0, 0.0)
            return

    def _run_docking_control(self) -> None:
        now = self.get_clock().now()

        if self._last_detection_stamp is None:
            self.get_logger().debug('Sin detecciones recientes; volviendo a búsqueda.')
            self._set_state(DockState.SEARCHING)
            return
        if now - self._last_detection_stamp > self._detection_timeout:
            self.get_logger().warn('Perdimos el ArUco; regresando a búsqueda.')
            self._set_state(DockState.SEARCHING)
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, self._tag_frame, Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException as exc:
            self.get_logger().debug(f'TF no disponible: {exc}')
            return

        dx = transform.transform.translation.x
        dy = transform.transform.translation.y
        planar_distance = math.hypot(dx, dy)
        heading_error = math.atan2(dy, dx)

        if self._last_range is not None and self._last_range_stamp is not None:
            if now - self._last_range_stamp > self._range_timeout:
                self.get_logger().debug('Medición ultrasónica vencida, solicitando actualización.')
                self._maybe_request_distance(now)
            elif self._last_range <= self._dock_distance:
                self.get_logger().info('Umbral de 2cm alcanzado mediante ultrasonido.')
                self._set_state(DockState.DOCKED)
                return

        if planar_distance <= self._dock_distance:
            self.get_logger().info(
                f'Distancia al ArUco <= {self._dock_distance:.3f} m, docking completado.'
            )
            self._set_state(DockState.DOCKED)
            return

        angular_cmd = clamp(
            heading_error * self._angular_kp, -self._max_angular_speed, self._max_angular_speed
        )

        if abs(heading_error) > self._align_tolerance:
            linear_cmd = 0.0
        else:
            forward_error = planar_distance - self._dock_distance
            raw_linear = forward_error * self._linear_kp
            linear_cmd = clamp(raw_linear, self._min_linear_speed, self._max_linear_speed)

        self._publish_twist(linear_cmd, angular_cmd)
        self._maybe_request_distance(now)

    def _maybe_request_distance(self, now: Time) -> None:
        if self._distance_request_period.nanoseconds <= 0:
            return
        if self._distance_future is not None and not self._distance_future.done():
            return
        if self._last_distance_request is not None:
            if now - self._last_distance_request < self._distance_request_period:
                return
        if not self._distance_client.service_is_ready():
            return
        self._last_distance_request = now
        self._distance_future = self._distance_client.call_async(Trigger.Request())

    def _publish_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    # endregion ----------------------------------------------------------------

    # region State helpers -----------------------------------------------------

    def _set_state(self, new_state: DockState) -> None:
        if self._state == new_state:
            return
        self._state = new_state
        self.get_logger().info(f'Estado docking -> {new_state.name}')
        self._status_pub.publish(String(data=new_state.name))
        if new_state in (DockState.DOCKED, DockState.FAILED):
            self._publish_twist(0.0, 0.0)

    # endregion ----------------------------------------------------------------


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GoToBaseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
