from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelBridge(Node):
    """Remaps nav2 cmd_vel_nav output onto /cmd_vel/nav2 for the mux."""

    def __init__(self) -> None:
        super().__init__('robertito_cmd_vel_bridge')
        raw_input = str(self.declare_parameter('input_topic', '/cmd_vel_nav').value)
        raw_output = str(self.declare_parameter('output_topic', '/cmd_vel/nav2').value)
        if not raw_input.startswith('/'):
            raw_input = f'/{raw_input}'
        if not raw_output.startswith('/'):
            raw_output = f'/{raw_output}'

        self._publisher = self.create_publisher(Twist, raw_output, 10)
        self.create_subscription(Twist, raw_input, self._on_twist, 10)

    def _on_twist(self, msg: Twist) -> None:
        self._publisher.publish(deepcopy(msg))


def main() -> None:
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
