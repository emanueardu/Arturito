from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String


class FaceGreeterNode(Node):
    """Listens for face detections and triggers eye expressions and TTS greetings."""

    def __init__(self) -> None:
        super().__init__("arturito_face_greeter")

        face_topic = str(self.declare_parameter("face_topic", "arturito/face_detected").value)
        self._expression_topic = str(
            self.declare_parameter("expression_topic", "arturito/eyes_expression").value
        )
        self._tts_topic = str(self.declare_parameter("tts_topic", "arturito/say").value)
        self._happy_expression = str(
            self.declare_parameter("happy_expression", "happy").value
        ).lower()
        self._idle_expression = str(
            self.declare_parameter("idle_expression", "normal").value
        ).lower()
        self._greeting_text = str(
            self.declare_parameter("greeting_text", "Hola, como estas?").value
        )
        cooldown = float(self.declare_parameter("greeting_cooldown_sec", 10.0).value)
        self._cooldown = Duration(seconds=max(cooldown, 0.0))

        queue_size = 10
        self._face_sub = self.create_subscription(Bool, face_topic, self._on_face, queue_size)
        self._expr_pub = self.create_publisher(String, self._expression_topic, queue_size)
        self._tts_pub = self.create_publisher(String, self._tts_topic, queue_size)

        self._face_present = False
        self._last_expression: Optional[str] = None
        self._last_greet_time = self.get_clock().now() - Duration(seconds=3600.0)

        self.get_logger().info(
            "Face greeter listo: escuchando detecciones en %s" % face_topic
        )

    def _on_face(self, msg: Bool) -> None:
        now = self.get_clock().now()
        if msg.data:
            if not self._face_present:
                self._face_present = True
                self._publish_expression(self._happy_expression)
                self._send_greeting(now)
            else:
                if now - self._last_greet_time >= self._cooldown:
                    self._send_greeting(now)
                self._publish_expression(self._happy_expression)
        else:
            if self._face_present:
                self._face_present = False
                self._publish_expression(self._idle_expression)

    def _publish_expression(self, expression: str) -> None:
        expression = expression.lower()
        if expression == self._last_expression:
            return
        self._last_expression = expression
        self._expr_pub.publish(String(data=expression))

    def _send_greeting(self, now) -> None:
        self._last_greet_time = now
        self._tts_pub.publish(String(data=self._greeting_text))
        self.get_logger().info("Saludando: %s" % self._greeting_text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceGreeterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
