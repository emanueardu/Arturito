import threading
import time
from typing import Callable, Dict, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

try:
    from PIL import Image, ImageDraw, ImageFont, ImageOps
except ImportError as exc:  # pragma: no cover - runtime dependency check
    raise RuntimeError('Pillow is required for ArturitoEyes node') from exc

try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import sh1106, ssd1306
except ImportError as exc:  # pragma: no cover - runtime dependency check
    raise RuntimeError('luma.oled is required for ArturitoEyes node') from exc


class EyesRenderer:
    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
        self.margin_x = max(4, width // 10)
        self.margin_y = max(4, height // 8)
        self.outline_width = max(1, width // 32)
        self.pupil_radius_base = max(4, int(min(width, height) * 0.18))
        self._expression_map: Dict[str, Callable[[], Tuple[Image.Image, Image.Image]]] = {
            'normal': self._expression_normal,
            'happy': self._expression_happy,
            'angry': self._expression_angry,
            'confused': self._expression_confused,
            'focus': self._expression_focus,
            'hearts': self._expression_hearts,
            'sad': self._expression_sad,
            'sleeping': self._expression_sleeping,
            'sleeping_alt': self._expression_sleeping_alt,
            'sleepy': self._expression_sleepy,
            'surprised': self._expression_surprised,
            'wink_left': self._expression_wink_left,
            'wink_right': self._expression_wink_right,
            'blink': self._expression_blink,
        }

    def available(self) -> Tuple[str, ...]:
        return tuple(sorted(self._expression_map.keys()))

    def render_pair(self, name: str) -> Tuple[Image.Image, Image.Image]:
        key = name.lower()
        renderer = self._expression_map.get(key, self._expression_normal)
        return renderer()

    # region Base drawing helpers -------------------------------------------
    def _blank(self) -> Image.Image:
        return Image.new('1', (self.width, self.height), 0)

    def _draw_eye(
        self,
        *,
        offset_x: float = 0.0,
        offset_y: float = 0.0,
        pupil_scale: float = 1.0,
        lid_top: float = 0.0,
        lid_bottom: float = 0.0,
        eyebrow: str | None = None,
        highlight: bool = True,
    ) -> Image.Image:
        img = self._blank()
        draw = ImageDraw.Draw(img)
        eyeball_box = [
            self.margin_x,
            self.margin_y,
            self.width - self.margin_x,
            self.height - self.margin_y,
        ]
        draw.ellipse(eyeball_box, fill=255)
        draw.ellipse(eyeball_box, outline=0, width=self.outline_width)

        pupil_rx = max(2, int(self.pupil_radius_base * pupil_scale))
        pupil_ry = max(2, int(self.pupil_radius_base * pupil_scale * 1.1))
        cx = self.width // 2 + int(offset_x * self.width * 0.22)
        cy = self.height // 2 + int(offset_y * self.height * 0.26)
        pupil_box = [cx - pupil_rx, cy - pupil_ry, cx + pupil_rx, cy + pupil_ry]
        draw.ellipse(pupil_box, fill=0)
        if highlight and pupil_rx > 4 and pupil_ry > 4:
            hx = cx - pupil_rx + max(1, pupil_rx // 3)
            hy = cy - pupil_ry + max(1, pupil_ry // 3)
            draw.ellipse(
                [hx, hy, hx + max(1, pupil_rx // 2), hy + max(1, pupil_ry // 2)],
                fill=255,
            )

        if lid_top > 0.0:
            lid_height = int((self.height / 2) * lid_top)
            draw.rectangle([0, 0, self.width, self.margin_y + lid_height], fill=0)
        if lid_bottom > 0.0:
            lid_height = int((self.height / 2) * lid_bottom)
            draw.rectangle(
                [0, self.height - (self.margin_y + lid_height), self.width, self.height],
                fill=0,
            )

        if eyebrow:
            self._draw_eyebrow(draw, eyebrow)
        return img

    def _draw_closed_eye(self, *, smile: bool = False, smile_up: bool = False) -> Image.Image:
        img = self._blank()
        draw = ImageDraw.Draw(img)
        line_width = max(2, self.height // 12)
        y = self.height // 2
        if smile:
            radius = self.width - 2 * self.margin_x
            box = [
                self.margin_x,
                y - radius // 3,
                self.width - self.margin_x,
                y + radius // 3,
            ]
            if smile_up:
                draw.arc(box, start=0, end=180, fill=255, width=line_width)
            else:
                draw.arc(box, start=180, end=360, fill=255, width=line_width)
        else:
            draw.line(
                [(self.margin_x, y), (self.width - self.margin_x, y)],
                fill=255,
                width=line_width,
            )
        return img

    def _draw_heart_eye(self) -> Image.Image:
        img = self._blank()
        draw = ImageDraw.Draw(img)
        size = min(self.width, self.height) - 2 * self.margin_x
        cx = self.width // 2
        cy = self.height // 2
        half = size // 2
        triangle = [
            (cx, cy + half),
            (cx - half, cy),
            (cx + half, cy),
        ]
        draw.polygon(triangle, fill=255)
        draw.ellipse([cx - half, cy - half, cx, cy + half // 2], fill=255)
        draw.ellipse([cx, cy - half, cx + half, cy + half // 2], fill=255)
        return img

    def _draw_eyebrow(self, draw: ImageDraw.ImageDraw, mode: str) -> None:
        thickness = max(2, self.height // 14)
        y = int(self.margin_y * 0.6)
        if mode == 'angry_left':
            draw.line(
                [(self.margin_x, y + thickness), (self.width - self.margin_x, y - thickness)],
                fill=255,
                width=thickness,
            )
        elif mode == 'angry_right':
            draw.line(
                [(self.margin_x, y - thickness), (self.width - self.margin_x, y + thickness)],
                fill=255,
                width=thickness,
            )

    def _draw_zzz(self, draw: ImageDraw.ImageDraw, *, offset_x: int = 0, offset_y: int = 0) -> None:
        font = ImageFont.load_default()
        text = 'Zzz'
        try:
            bbox = draw.textbbox((0, 0), text, font=font)
            text_w = bbox[2] - bbox[0]
            text_h = bbox[3] - bbox[1]
        except AttributeError:
            text_w, text_h = draw.textsize(text, font=font)
        x = max(0, self.width - self.margin_x - text_w) + offset_x
        y = max(0, self.margin_y // 2) + offset_y
        draw.text((x, y), text, fill=255, font=font)

    # endregion -------------------------------------------------------------

    # region Expressions ----------------------------------------------------
    def _expression_normal(self) -> Tuple[Image.Image, Image.Image]:
        eye = self._draw_eye()
        return eye, eye.copy()

    def _expression_happy(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_closed_eye(smile=True, smile_up=True)
        right = self._draw_closed_eye(smile=True, smile_up=True)
        return left, right

    def _expression_angry(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_eye(offset_x=0.15, lid_top=0.2, eyebrow='angry_left')
        right = self._draw_eye(offset_x=-0.15, lid_top=0.2, eyebrow='angry_right')
        return left, right

    def _expression_confused(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_eye(offset_x=-0.2, offset_y=-0.1, lid_top=0.1)
        right = self._draw_eye(offset_x=0.2, offset_y=0.2, lid_bottom=0.2)
        return left, right

    def _expression_focus(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_eye(pupil_scale=0.7)
        right = self._draw_eye(pupil_scale=0.7)
        return left, right

    def _expression_hearts(self) -> Tuple[Image.Image, Image.Image]:
        heart_left = self._draw_heart_eye()
        heart_right = self._draw_heart_eye()
        return heart_left, heart_right

    def _expression_sad(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_eye(offset_y=0.25, lid_top=0.3)
        right = self._draw_eye(offset_y=0.25, lid_top=0.3)
        return left, right

    def _expression_sleeping(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_closed_eye()
        right = self._draw_closed_eye()
        self._draw_zzz(ImageDraw.Draw(left), offset_x=0, offset_y=0)
        self._draw_zzz(ImageDraw.Draw(right), offset_x=0, offset_y=0)
        return left, right

    def _expression_sleeping_alt(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_closed_eye()
        right = self._draw_closed_eye()
        self._draw_zzz(ImageDraw.Draw(left), offset_x=-2, offset_y=2)
        self._draw_zzz(ImageDraw.Draw(right), offset_x=-2, offset_y=2)
        return left, right

    def _expression_sleepy(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_eye(lid_top=0.5, lid_bottom=0.1)
        right = self._draw_eye(lid_top=0.5, lid_bottom=0.1)
        return left, right

    def _expression_surprised(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_eye(pupil_scale=0.5, highlight=False)
        right = self._draw_eye(pupil_scale=0.5, highlight=False)
        return left, right

    def _expression_wink_left(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_closed_eye(smile=True)
        right = self._draw_eye(offset_x=0.15)
        return left, right

    def _expression_wink_right(self) -> Tuple[Image.Image, Image.Image]:
        left = self._draw_eye(offset_x=-0.15)
        right = self._draw_closed_eye(smile=True)
        return left, right

    def _expression_blink(self) -> Tuple[Image.Image, Image.Image]:
        closed = self._draw_closed_eye()
        return closed, closed.copy()

    # endregion -------------------------------------------------------------


class ArturitoEyes(Node):
    def __init__(self) -> None:
        super().__init__('arturito_eyes')

        self._i2c_bus = int(self.declare_parameter('i2c_bus', 1).value)
        self._left_address = int(self.declare_parameter('left_address', 0x3C).value)
        self._right_address = int(self.declare_parameter('right_address', 0x3D).value)
        self._driver = str(self.declare_parameter('driver', 'sh1106').value).lower()
        self._default_expression = str(
            self.declare_parameter('default_expression', 'normal').value
        ).lower()
        self._enable_auto_blink = bool(
            self.declare_parameter('enable_auto_blink', True).value
        )
        self._blink_interval = float(self.declare_parameter('blink_interval_sec', 7.0).value)
        self._blink_duration = float(self.declare_parameter('blink_duration_sec', 0.15).value)
        self._mirror_right = bool(self.declare_parameter('mirror_right', False).value)
        self._brightness = int(self.declare_parameter('brightness', 255).value)
        self._person_detection_topic = str(
            self.declare_parameter('person_detection_topic', '').value
        )
        self._person_expression = str(
            self.declare_parameter('person_expression', 'happy').value
        ).lower()
        self._person_timeout = float(
            self.declare_parameter('person_timeout_sec', 1.0).value
        )
        self._sleep_expression = str(
            self.declare_parameter('sleep_expression', 'sleeping').value
        ).lower()
        self._sleep_tts_topic = str(
            self.declare_parameter('sleep_tts_topic', '/assistant/say').value
        )
        self._sleep_tts_message = str(
            self.declare_parameter('sleep_tts_message', 'Buenas noches').value
        )
        self._sleep_animation_enabled = bool(
            self.declare_parameter('sleep_animation_enabled', True).value
        )
        self._sleep_animation_period = float(
            self.declare_parameter('sleep_animation_period_sec', 0.7).value
        )
        self._light_state_topic = str(
            self.declare_parameter('light_state_topic', '').value
        )
        self._light_state_inverted = bool(
            self.declare_parameter('light_state_inverted', False).value
        )
        self._light_level_topic = str(
            self.declare_parameter('light_level_topic', '').value
        )
        self._dark_threshold = float(
            self.declare_parameter('dark_threshold', 5.0).value
        )
        # Topic configurable para la expresión de los ojos (puede ser overridden por el launch)
        self._expression_topic = str(
            self.declare_parameter('expression_topic', 'arturito/eyes_expression').value
        )

        device_cls = {'sh1106': sh1106, 'ssd1306': ssd1306}.get(self._driver)
        if device_cls is None:
            raise ValueError(f'Unsupported OLED driver "{self._driver}"')

        serial_left = i2c(port=self._i2c_bus, address=self._left_address)
        serial_right = i2c(port=self._i2c_bus, address=self._right_address)
        self._left_display = device_cls(serial_left)
        self._right_display = device_cls(serial_right)
        self._left_display.contrast(self._brightness)
        self._right_display.contrast(self._brightness)

        self._renderer = EyesRenderer(self._left_display.width, self._left_display.height)
        self._available = set(self._renderer.available())

        if self._default_expression not in self._available:
            self.get_logger().warn(
                f'Default expression "{self._default_expression}" unknown, falling back to normal'
            )
            self._default_expression = 'normal'
        if self._person_expression not in self._available:
            self.get_logger().warn(
                f'Person expression "{self._person_expression}" unknown, using default expression'
            )
            self._person_expression = self._default_expression
        if self._sleep_expression not in self._available:
            self.get_logger().warn(
                f'Sleep expression "{self._sleep_expression}" unknown, using sleepy expression'
            )
            self._sleep_expression = 'sleepy' if 'sleepy' in self._available else self._default_expression

        self._pub_status = self.create_publisher(String, 'arturito/eyes_status', 10)
        self._tts_pub = (
            self.create_publisher(String, self._sleep_tts_topic, 10)
            if self._sleep_tts_topic
            else None
        )
        # Suscribirse al tópico configurado para expresiones
        self.create_subscription(String, self._expression_topic, self._on_expression, 10)
        self.create_service(Trigger, 'arturito/blink_now', self._srv_blink_now)
        if self._person_detection_topic:
            self.create_subscription(
                Detection2DArray,
                self._person_detection_topic,
                self._on_person_detection,
                10,
            )
        if self._light_state_topic:
            self.create_subscription(Bool, self._light_state_topic, self._on_light_state, 10)
        if self._light_level_topic:
            self.create_subscription(Float32, self._light_level_topic, self._on_light_level, 10)

        self._display_lock = threading.Lock()
        self._desired_expression = self._default_expression
        self._target_expression = self._default_expression
        self._active_expression = None
        self._blink_active = False
        self._blink_restore_time = 0.0
        self._last_person_time = 0.0
        self._dark = False
        self._sleep_anim_state = False
        self._sleep_anim_timer = None

        self._state_timer = self.create_timer(0.05, self._state_tick)
        self._auto_blink_timer = None
        if self._enable_auto_blink and self._blink_interval > 0.0:
            self._auto_blink_timer = self.create_timer(
                self._blink_interval, self._auto_blink_tick
            )
        if self._sleep_animation_enabled and self._sleep_animation_period > 0.0:
            self._sleep_anim_timer = self.create_timer(
                self._sleep_animation_period, self._sleep_animation_tick
            )

        self._set_expression(self._default_expression, publish=True)
        self.get_logger().info(
            'Arturito eyes node ready. Expressions: %s'
            % ', '.join(sorted(self._available))
        )

    # region ROS callbacks --------------------------------------------------
    def _on_expression(self, msg: String) -> None:
        requested = msg.data.strip().lower()
        if not requested:
            return
        if requested not in self._available:
            self.get_logger().warn(
                f'Unknown expression "{requested}". Available: {sorted(self._available)}'
            )
            return
        self._desired_expression = requested
        if not self._blink_active and not self._dark and not self._person_visible():
            self._target_expression = requested
            self._set_expression(requested, publish=True)

    def _srv_blink_now(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self._trigger_blink()
        resp.success = True
        resp.message = 'Blink triggered'
        return resp

    def _state_tick(self) -> None:
        if self._blink_active and time.monotonic() >= self._blink_restore_time:
            self._blink_active = False
        if not self._blink_active:
            expression = self._resolve_expression()
            if expression != self._active_expression:
                self._target_expression = expression
                self._set_expression(expression, publish=True)

    def _auto_blink_tick(self) -> None:
        if not self._enable_auto_blink:
            return
        if self._blink_active:
            return
        if self._dark:
            return
        if self._target_expression == 'blink':
            return
        self._trigger_blink()

    def _on_person_detection(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            return
        self._last_person_time = time.monotonic()

    def _on_light_state(self, msg: Bool) -> None:
        dark = bool(msg.data)
        if not self._light_state_inverted:
            dark = not dark
        self._set_dark(dark)

    def _on_light_level(self, msg: Float32) -> None:
        dark = float(msg.data) <= self._dark_threshold
        self._set_dark(dark)

    def _sleep_animation_tick(self) -> None:
        if not self._dark or self._blink_active:
            return
        self._sleep_anim_state = not self._sleep_anim_state
        expression = self._resolve_expression()
        if expression != self._active_expression:
            self._target_expression = expression
            self._set_expression(expression, publish=True)

    # endregion -------------------------------------------------------------

    def _trigger_blink(self) -> None:
        self._target_expression = self._resolve_expression()
        self._blink_active = True
        self._blink_restore_time = time.monotonic() + max(0.05, self._blink_duration)
        self._set_expression('blink', publish=True)

    def _set_expression(self, expression: str, *, publish: bool) -> None:
        with self._display_lock:
            left_img, right_img = self._renderer.render_pair(expression)
            if self._mirror_right:
                right_img = ImageOps.mirror(right_img)
            self._left_display.display(left_img)
            self._right_display.display(right_img)
            if publish and expression != self._active_expression:
                self._pub_status.publish(String(data=expression))
            self._active_expression = expression

    def _person_visible(self) -> bool:
        if self._person_timeout <= 0.0:
            return False
        if self._last_person_time <= 0.0:
            return False
        return (time.monotonic() - self._last_person_time) <= self._person_timeout

    def _resolve_expression(self) -> str:
        if self._dark:
            if self._sleep_animation_enabled and self._sleep_animation_period > 0.0:
                return 'sleeping_alt' if self._sleep_anim_state else 'sleeping'
            return self._sleep_expression
        if self._person_detection_topic and self._person_visible():
            return self._person_expression
        return self._desired_expression

    def _set_dark(self, value: bool) -> None:
        if value == self._dark:
            return
        self._dark = value
        if self._blink_active:
            self._blink_active = False
        if self._dark and self._tts_pub is not None and self._sleep_tts_message:
            payload = self._build_tts_message(self._sleep_tts_message)
            if payload:
                self._tts_pub.publish(String(data=payload))
        expression = self._resolve_expression()
        self._target_expression = expression
        self._set_expression(expression, publish=True)

    @staticmethod
    def _build_tts_message(text: str) -> str:
        cleaned = text.strip()
        if not cleaned:
            return ""
        return f"[robertito_eyes] {cleaned}"

    def destroy_node(self) -> bool:
        self.get_logger().info('Shutting down Arturito eyes node')
        if self._auto_blink_timer:
            self._auto_blink_timer.cancel()
        if self._sleep_anim_timer:
            self._sleep_anim_timer.cancel()
        self._state_timer.cancel()
        with self._display_lock:
            blank = self._renderer._blank()
            self._left_display.display(blank)
            self._right_display.display(blank)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArturitoEyes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
