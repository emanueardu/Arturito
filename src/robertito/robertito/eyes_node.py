"""Nodo orquestador de ojos para Robertito.

Subs: robertito/eyes_expression (String), arturito/camera/faces (Detection2DArray),
      /behavior/thinking_active (Bool), /wake_word/detected (Bool),
      /behavior/speaking_active (Bool), arturito/camera/brightness (Float32).
Pubs: arturito/eyes_status (String), robertito/motion_request (String JSON),
      /head/tilt (Float32 si enable_head_motion).
Srv:  arturito/blink_now (Trigger).
Render: 20 Hz vía rclpy timer; push a OLED en el mismo callback.
"""
from __future__ import annotations

import json
import threading
import time

import rclpy
from PIL import ImageOps
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from robertito.eyes import (
    EXPRESSIONS, MOTION_PROFILES, BlinkController, EyesAnimator,
    EyesRenderer, GazeController, resolve_expression,
)


_DEPRECATED = (
    "sleep_expression", "sleep_animation_expression_a",
    "sleep_animation_expression_b", "sleep_animation_enabled",
    "sleep_animation_period_sec", "sleep_tts_topic", "sleep_tts_message",
    "person_expression", "person_timeout_sec",
    "light_state_topic", "light_state_inverted",
)


class RobertitoEyesNode(Node):
    """Orquestador de los OLEDs SH1106 de Robertito (animación 20 Hz)."""

    def __init__(self) -> None:
        super().__init__("robertito_eyes")

        p = self.declare_parameter
        # Hardware
        i2c_bus = int(p("i2c_bus", 1).value)
        left_addr = int(p("left_address", 0x3C).value)
        right_addr = int(p("right_address", 0x3D).value)
        driver = str(p("driver", "sh1106").value).lower()
        brightness = int(p("brightness", 200).value)
        self._mirror_right = bool(p("mirror_right", False).value)
        # 0=0° / 1=90° / 2=180° / 3=270° — rotación aplicada por luma.oled
        # antes del push I2C. Útil cuando los OLEDs están montados invertidos.
        oled_rotate_l = int(p("oled_rotate_left", 0).value)
        oled_rotate_r = int(p("oled_rotate_right", 0).value)
        # Animator
        self._frame_rate_hz = float(p("frame_rate_hz", 20.0).value)
        transition_ms = float(p("transition_duration_ms", 200.0).value)
        default_expr = str(p("default_expression", "calma").value)
        # Blink
        blink_interval = float(p("blink_interval_sec", 4.0).value)
        blink_jitter = float(p("blink_jitter_sec", 2.0).value)
        p("blink_duration_sec", 0.25)  # legacy compat
        p("enable_auto_blink", True)
        # Gaze
        face_topic = str(p("person_detection_topic", "arturito/camera/faces").value)
        gaze_face_timeout = float(p("gaze_face_timeout_s", 1.0).value)
        gaze_idle_freq = float(p("gaze_idle_freq_s", 1.8).value)
        gaze_idle_amp_x = float(p("gaze_idle_amp_x", 0.25).value)
        gaze_idle_amp_y = float(p("gaze_idle_amp_y", 0.18).value)
        self._person_locks_expression = bool(p("person_locks_expression", False).value)
        # Triggers
        self._expression_topic = str(p("expression_topic", "robertito/eyes_expression").value)
        thinking_topic = str(p("thinking_active_topic", "/behavior/thinking_active").value)
        wake_topic = str(p("wake_word_topic", "/wake_word/detected").value)
        speaking_topic = str(p("speaking_active_topic", "/behavior/speaking_active").value)
        self._wake_pulse_duration_s = float(p("wake_pulse_duration_s", 0.4).value)
        # Light / dark
        light_level_topic = str(p("light_level_topic", "arturito/camera/brightness").value)
        self._dark_threshold = float(p("dark_threshold", 40.0).value)
        # Motion (hooks)
        motion_topic = str(p("motion_request_topic", "robertito/motion_request").value)
        head_tilt_topic = str(p("head_tilt_topic", "/head/tilt").value)
        self._enable_head_motion = bool(p("enable_head_motion", True).value)
        # Deprecated: declarar para no romper YAMLs viejos
        for dp in _DEPRECATED:
            try:
                if not self.has_parameter(dp):
                    p(dp, "")
            except Exception:
                pass

        # OLEDs
        self._display_lock = threading.Lock()
        self._left_display, self._right_display = self._init_displays(
            driver, i2c_bus, left_addr, right_addr, brightness,
            oled_rotate_l, oled_rotate_r)

        # Pipeline
        w, h = self._left_display.width, self._left_display.height
        self._renderer = EyesRenderer(w, h)
        self._animator = EyesAnimator(transition_duration_s=transition_ms / 1000.0)
        self._gaze = GazeController(
            face_timeout_s=gaze_face_timeout,
            idle_freq_s=gaze_idle_freq,
            idle_amp_x=gaze_idle_amp_x, idle_amp_y=gaze_idle_amp_y)
        self._blink = BlinkController(
            interval_s=blink_interval, jitter_s=blink_jitter,
            frame_dt_s=1.0 / self._frame_rate_hz)

        # Estado
        self._desired_expression = resolve_expression(default_expr)
        if self._desired_expression not in EXPRESSIONS:
            self._desired_expression = "calma"
        self._dark = False
        self._thinking_active = False
        self._wake_pulse_until: float = 0.0

        # Callback groups: el frame_tick va aislado en mutex propio (push I2C ~55ms)
        # y todo lo demás (subs, service, log_fps) en un grupo reentrante para que
        # no quede starved por el frame loop. Requiere MultiThreadedExecutor en main().
        self._frame_cb_group = MutuallyExclusiveCallbackGroup()
        self._sub_cb_group = ReentrantCallbackGroup()

        # Pub/Sub
        self._pub_status = self.create_publisher(String, "arturito/eyes_status", 10)
        self._pub_motion = self.create_publisher(String, motion_topic, 10)
        self._pub_head_tilt = self.create_publisher(Float32, head_tilt_topic, 10)
        self.create_subscription(
            String, self._expression_topic, self._on_expression, 10,
            callback_group=self._sub_cb_group,
        )
        if face_topic:
            self.create_subscription(
                Detection2DArray, face_topic, self._on_face_detection, 10,
                callback_group=self._sub_cb_group,
            )
        self.create_subscription(
            Bool, thinking_topic, self._on_thinking_active, 10,
            callback_group=self._sub_cb_group,
        )
        self.create_subscription(
            Bool, wake_topic, self._on_wake_word, 10,
            callback_group=self._sub_cb_group,
        )
        self.create_subscription(
            Bool, speaking_topic, self._on_speaking_active, 10,
            callback_group=self._sub_cb_group,
        )
        if light_level_topic:
            self.create_subscription(
                Float32, light_level_topic, self._on_light_level, 10,
                callback_group=self._sub_cb_group,
            )
        self.create_service(
            Trigger, "arturito/blink_now", self._srv_blink_now,
            callback_group=self._sub_cb_group,
        )

        # Timers
        self.create_timer(
            1.0 / self._frame_rate_hz, self._frame_tick,
            callback_group=self._frame_cb_group,
        )
        self._fps_counter = 0
        self._fps_last_log_t = time.monotonic()
        self.create_timer(5.0, self._log_fps, callback_group=self._sub_cb_group)

        self._set_target_expression(default_expr)
        self.get_logger().info(
            f"Robertito eyes ready @{self._frame_rate_hz:.0f} Hz "
            f"(default={self._desired_expression}). "
            f"Expresiones: {sorted(EXPRESSIONS.keys())}")

    # ───────────────────────────────────────────────── hardware
    @staticmethod
    def _init_displays(driver: str, bus: int, left_addr: int,
                       right_addr: int, brightness: int,
                       rotate_left: int = 0, rotate_right: int = 0):
        from luma.core.interface.serial import i2c
        from luma.oled.device import sh1106, ssd1306
        cls = ssd1306 if driver == "ssd1306" else sh1106
        left = cls(i2c(port=bus, address=left_addr), rotate=rotate_left)
        right = cls(i2c(port=bus, address=right_addr), rotate=rotate_right)
        try:
            left.contrast(brightness)
            right.contrast(brightness)
        except Exception:
            pass
        return left, right

    # ───────────────────────────────────────────────── frame loop
    def _frame_tick(self) -> None:
        t = time.monotonic()
        self._animator.set_target(self._resolve_active_expression(t))
        pair = self._animator.tick(t, self._gaze, self._blink)
        left_img = self._renderer.render(pair.left, "L", t)
        right_img = self._renderer.render(pair.right, "R", t)
        if self._mirror_right:
            right_img = ImageOps.mirror(right_img)
        try:
            with self._display_lock:
                self._left_display.display(left_img)
                self._right_display.display(right_img)
        except Exception as exc:
            self.get_logger().warn(f"Display push falló: {exc}")
        self._fps_counter += 1

    def _resolve_active_expression(self, t: float) -> str:
        if self._dark:
            return "dormido"
        if t < self._wake_pulse_until:
            return "sorprendido"
        if self._thinking_active:
            return "pensativo"
        return self._desired_expression

    # ───────────────────────────────────────────────── callbacks
    def _on_expression(self, msg: String) -> None:
        if msg.data:
            self._set_target_expression(msg.data)

    def _on_face_detection(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            self._gaze.clear_face()
            return
        det = max(msg.detections,
                  key=lambda d: (d.bbox.size_x * d.bbox.size_y) if d.bbox else 0.0)
        cx = det.bbox.center.position.x if det.bbox else 320.0
        cy = det.bbox.center.position.y if det.bbox else 240.0
        nx = max(-1.0, min(1.0, (cx - 320.0) / 320.0))
        ny = max(-1.0, min(1.0, (cy - 240.0) / 240.0))
        self._gaze.set_face(nx, ny, time.monotonic())
        if self._person_locks_expression:
            self._set_target_expression("feliz")

    def _on_thinking_active(self, msg: Bool) -> None:
        self._thinking_active = bool(msg.data)

    def _on_wake_word(self, msg: Bool) -> None:
        if bool(msg.data):
            self._wake_pulse_until = time.monotonic() + self._wake_pulse_duration_s

    def _on_speaking_active(self, msg: Bool) -> None:
        self._gaze.set_speaking(bool(msg.data))

    def _on_light_level(self, msg: Float32) -> None:
        self._dark = float(msg.data) < self._dark_threshold

    def _srv_blink_now(self, _req, resp):
        self._blink.force_blink(time.monotonic())
        resp.success = True
        resp.message = "blink forced"
        return resp

    # ───────────────────────────────────────────────── core
    def _set_target_expression(self, name: str) -> None:
        canonical = resolve_expression(name)
        if canonical == "_blink":
            self._blink.force_blink(time.monotonic())
            return
        if canonical == "_glance_left":
            self._gaze.set_glance("left", time.monotonic(), 1.0)
            return
        if canonical == "_glance_right":
            self._gaze.set_glance("right", time.monotonic(), 1.0)
            return
        if canonical not in EXPRESSIONS:
            self.get_logger().warn(f'Expresión desconocida "{name}"')
            return
        if canonical == self._desired_expression:
            return
        self._desired_expression = canonical
        self._pub_status.publish(String(data=canonical))
        profile = MOTION_PROFILES.get(canonical, {})
        self._pub_motion.publish(
            String(data=json.dumps({"expression": canonical, "profile": profile})))
        if "head_tilt_deg" in profile and self._enable_head_motion:
            self._pub_head_tilt.publish(Float32(data=float(profile["head_tilt_deg"])))

    def _log_fps(self) -> None:
        now = time.monotonic()
        elapsed = now - self._fps_last_log_t
        fps = self._fps_counter / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(f"eyes_fps={fps:.1f}")
        self._fps_counter = 0
        self._fps_last_log_t = now

    def destroy_node(self) -> bool:
        try:
            with self._display_lock:
                for d in (self._left_display, self._right_display):
                    try:
                        d.clear()
                    except Exception:
                        pass
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RobertitoEyesNode()
    # 2 threads: 1 para frame_tick (push I2C bloqueante), 1 para subs/service/log.
    # Sin esto, el frame loop monopoliza el SingleThreadedExecutor y las
    # subscriptions de expresión nunca se procesan.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
