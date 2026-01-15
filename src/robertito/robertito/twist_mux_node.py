from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass
class _InputSource:
    name: str
    topic: str
    priority: float
    timeout: float
    last_msg: Optional[Twist] = None
    last_time: Optional[rclpy.time.Time] = None

    def is_active(self, now: rclpy.time.Time) -> bool:
        if self.last_msg is None or self.last_time is None:
            return False
        if self.timeout <= 0:
            return True
        age = (now - self.last_time).nanoseconds * 1e-9
        return age <= self.timeout


class TwistMux(Node):
    """Simple priority-based cmd_vel mux for Robertito."""

    def __init__(self) -> None:
        super().__init__('robertito_twist_mux')

        default_inputs = [
            {'name': 'safety', 'topic': '/cmd_vel/safety_stop', 'priority': 30, 'timeout': 0.5},
            {'name': 'nav2', 'topic': '/cmd_vel/nav2', 'priority': 20, 'timeout': 0.5},
            {'name': 'person_track', 'topic': '/cmd_vel/person_track', 'priority': 10, 'timeout': 0.5},
            {'name': 'wander', 'topic': '/cmd_vel/wander', 'priority': 5, 'timeout': 0.5},
            {'name': 'teleop', 'topic': '/cmd_vel/teleop', 'priority': 4, 'timeout': 0.5},
        ]
        poll_hz = float(self.declare_parameter('publish_frequency', 20.0).value)
        output_topic = str(self.declare_parameter('output_topic', '/cmd_vel').value)
        if not output_topic.startswith('/'):
            output_topic = f'/{output_topic}'

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._inputs = self._create_input_sources(deepcopy(default_inputs))
        for source in self._inputs:
            self.create_subscription(Twist, source.topic, self._make_callback(source), 10)
        self._timer = self.create_timer(1.0 / max(poll_hz, 1e-3), self._publish_best)
        self._last_active: Optional[str] = None

    @staticmethod
    def _create_input_sources(raw_inputs: List) -> List[_InputSource]:
        sources: List[_InputSource] = []
        for entry in raw_inputs:
            if not isinstance(entry, dict):
                continue
            topic = str(entry.get('topic', '')).strip()
            if not topic:
                continue
            if not topic.startswith('/'):
                topic = f'/{topic}'
            sources.append(
                _InputSource(
                    name=str(entry.get('name', topic)),
                    topic=topic,
                    priority=float(entry.get('priority', 0)),
                    timeout=float(entry.get('timeout', 0.5)),
                )
            )
        # Ensure deterministic order by priority (desc)
        return sorted(sources, key=lambda src: src.priority, reverse=True)

    def _make_callback(self, source: _InputSource):
        def _callback(msg: Twist) -> None:
            source.last_msg = deepcopy(msg)
            source.last_time = self.get_clock().now()

        return _callback

    def _publish_best(self) -> None:
        now = self.get_clock().now()
        best_source = None
        for source in self._inputs:
            if source.is_active(now):
                best_source = source
                break

        if best_source is None:
            twist = Twist()
            source_name = None
        else:
            twist = deepcopy(best_source.last_msg)
            source_name = best_source.name

        if self._last_active != source_name:
            self.get_logger().debug(f'Cmd_vel source -> {source_name or "idle"}')
            self._last_active = source_name

        self._publisher.publish(twist)


def main() -> None:
    rclpy.init()
    node = TwistMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
