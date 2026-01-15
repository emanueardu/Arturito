import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class RobertitoNodeManager(Node):
    """Coordina reacciones globales cuando se detecta la palabra clave."""

    def __init__(self) -> None:
        super().__init__("robertito_node_manager")

        self._wake_topic = self.declare_parameter(
            "wake_topic", "/wake_word/detected"
        ).get_parameter_value().string_value
        self._eyes_topic = self.declare_parameter(
            "eyes_expression_topic", "robertito/eyes_expression"
        ).get_parameter_value().string_value
        self._tts_topic = self.declare_parameter(
            "tts_topic", "/assistant/say"
        ).get_parameter_value().string_value
        self._wake_greeting = self.declare_parameter(
            "wake_greeting", "Sí, ¿en qué puedo ayudarte?"
        ).get_parameter_value().string_value
        raw_startup = self.declare_parameter("startup_greeting", "").get_parameter_value().string_value
        if raw_startup.strip():
            self._startup_greeting = raw_startup.strip()
        else:
            self._startup_greeting = "Buenos días, soy Robertito, estoy listo para funcionar"
        self._startup_delay = float(
            self.declare_parameter("startup_delay_sec", 0.8).value
        )
        self._happy_expression = self.declare_parameter(
            "wake_expression", "happy"
        ).get_parameter_value().string_value
        self._cooldown_sec = self.declare_parameter(
            "activation_cooldown_sec", 2.5
        ).get_parameter_value().double_value

        self._tts_pub = self.create_publisher(String, self._tts_topic, 10)
        self._eyes_pub = self.create_publisher(String, self._eyes_topic, 10)
        self._last_activation: Optional[float] = None
        self._wake_sub = self.create_subscription(
            Bool, self._wake_topic, self._on_wake_event, 10
        )
        self.get_logger().info(
            f"Node manager escuchando activaciones en {self._wake_topic}"
        )
        self._startup_timer: Optional[rclpy.timer.Timer] = None
        if self._startup_greeting.strip():
            self._schedule_startup_message()

    def _on_wake_event(self, msg: Bool) -> None:
        if not msg.data:
            return
        now = time.monotonic()
        if (
            self._last_activation is not None
            and now - self._last_activation < self._cooldown_sec
        ):
            return
        self._last_activation = now
        self._send_wake_greeting()
        self._set_expression(self._happy_expression)

    def _send_wake_greeting(self) -> None:
        if not self._wake_greeting:
            return
        self._publish_tts(self._wake_greeting)
        self.get_logger().info("Saludo enviado al TTS tras wake word.")

    def _schedule_startup_message(self) -> None:
        delay = max(0.0, self._startup_delay)
        if delay == 0.0:
            self._publish_tts(self._startup_greeting)
            return
        self._startup_timer = self.create_timer(delay, self._on_startup_timer)

    def _on_startup_timer(self) -> None:
        self._publish_tts(self._startup_greeting)
        if self._startup_timer is not None:
            timer = self._startup_timer
            self._startup_timer = None
            timer.cancel()
            self.destroy_timer(timer)

    def _publish_tts(self, text: str) -> None:
        cleaned = text.strip()
        if not cleaned:
            return
        payload = f"[robertito_node_manager] {cleaned}"
        msg = String()
        msg.data = payload
        self._tts_pub.publish(msg)

    def _set_expression(self, expression: str) -> None:
        if not expression:
            return
        msg = String()
        msg.data = expression
        self._eyes_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobertitoNodeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
