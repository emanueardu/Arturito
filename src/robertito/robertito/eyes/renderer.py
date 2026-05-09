"""Renderer de un EyeState a una imagen PIL monocromo (mode='1').

Todas las shapes con cuerpo de ojo respetan el patrón de bordes redondeados
estilo CALMA. Las rotaciones se hacen en mode 'L' (no 'mode 1') para evitar
artefactos visuales — pasamos a 'L', rotamos con BICUBIC, threshold a 128 y
volvemos a '1'.
"""
from __future__ import annotations

import math
import random
from typing import Tuple

from PIL import Image, ImageDraw

from robertito.eyes.easing import breath
from robertito.eyes.state import EyeState


_BG = 0  # negro
_FG = 1  # blanco (en mode '1')


class EyesRenderer:
    """Renderiza un EyeState a una imagen 128x64 mode='1'."""

    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height

    def render(self, state: EyeState, side: str, t: float) -> Image.Image:
        """Renderiza un ojo a un canvas mode='1'.

        side='L' o 'R' afecta inclinación, brow y posición de extras.
        t en segundos para animaciones.
        """
        img = Image.new("1", (self.width, self.height), _BG)
        draw = ImageDraw.Draw(img)

        # Jitter por tensión (anger / sorprendido).
        dx, dy = self._jitter_offset(state, t)

        shape = state.shape
        if shape == "rect":
            self._shape_rect(img, draw, state, t, dx, dy)
        elif shape == "inclined_rect":
            self._shape_inclined_rect(img, state, side, t, dx, dy)
        elif shape == "arc_up":
            self._shape_arc(img, draw, state, t, dx, dy, kind="up")
        elif shape == "arc_down":
            self._shape_arc(img, draw, state, t, dx, dy, kind="down")
        elif shape == "line":
            self._shape_line(img, draw, state, t, dx, dy)
        elif shape == "heart":
            self._shape_heart(img, draw, state, t, dx, dy)
        elif shape == "xx":
            self._shape_xx(img, draw, state, t, dx, dy)
        else:
            self._shape_rect(img, draw, state, t, dx, dy)

        # Decoración no dependiente del cuerpo del ojo
        if state.show_brow:
            self._draw_brow(draw, side, state, t, dx, dy)
        if "anger_spark" in state.extras:
            self._draw_anger_spark(draw, side, t)
        if "teardrop" in state.extras:
            self._draw_teardrop(draw, side, t)
        if "sparkles" in state.extras:
            self._draw_sparkles(draw, self.width // 2, self.height // 2, t)
        if "z_floating_slow" in state.extras:
            self._draw_z_floating(draw, t, period=2.4, base_size=8)
        if "z_floating_fast" in state.extras:
            self._draw_z_floating(draw, t, period=1.4, base_size=7)
        if "eyelash_dots" in state.extras:
            self._draw_eyelash_dots(draw, state)

        return img

    # ─────────────────────────────────────────────────────── shapes

    def _eye_bbox(
        self, state: EyeState, t: float, dx: int, dy: int
    ) -> Tuple[int, int, int, int]:
        breath_amp = state.breath_amp_px
        b = breath(t, state.breath_freq_hz)
        w = max(2, state.width + int(round(breath_amp * b)))
        h = max(2, state.height + int(round(breath_amp * b)))
        cx = self.width // 2 + dx
        cy = self.height // 2 + dy
        x0 = cx - w // 2
        y0 = cy - h // 2
        return (x0, y0, x0 + w, y0 + h)

    def _shape_rect(
        self, img: Image.Image, draw: ImageDraw.ImageDraw,
        state: EyeState, t: float, dx: int, dy: int,
    ) -> None:
        bbox = self._eye_bbox(state, t, dx, dy)
        r = max(0, min(state.corner_radius, (bbox[2] - bbox[0]) // 2 - 1))
        draw.rounded_rectangle(bbox, radius=r, fill=_FG)
        self._apply_eyelids(draw, bbox, state)
        if state.pupil_visible:
            self._draw_pupil_glint(draw, bbox, state)

    def _shape_inclined_rect(
        self, img: Image.Image, state: EyeState, side: str,
        t: float, dx: int, dy: int,
    ) -> None:
        # Render rectángulo redondeado en sub-canvas mode 'L', rotar, threshold.
        breath_amp = state.breath_amp_px
        b = breath(t, state.breath_freq_hz)
        w = max(2, state.width + int(round(breath_amp * b)))
        h = max(2, state.height + int(round(breath_amp * b)))
        # Extra padding para que la rotación no recorte
        pad = max(w, h)
        sub_w = w + pad
        sub_h = h + pad
        sub = Image.new("L", (sub_w, sub_h), 0)
        sub_draw = ImageDraw.Draw(sub)
        rect_bbox = (
            (sub_w - w) // 2,
            (sub_h - h) // 2,
            (sub_w - w) // 2 + w,
            (sub_h - h) // 2 + h,
        )
        r = max(0, min(state.corner_radius, w // 2 - 1))
        sub_draw.rounded_rectangle(rect_bbox, radius=r, fill=255)

        # Determinar el ángulo signed según side y modo
        ang = state.inclination_deg
        if state.inclination_mode == "outer_drop":
            angle = -ang if side == "L" else ang
        elif state.inclination_mode == "inner_drop":
            angle = ang if side == "L" else -ang
        else:
            angle = ang

        sub = sub.rotate(angle, resample=Image.BICUBIC)
        # Threshold a binario y pegar al canvas final
        bin_img = sub.point(lambda v: 255 if v >= 128 else 0).convert("1")
        cx = self.width // 2 + dx - sub_w // 2
        cy = self.height // 2 + dy - sub_h // 2
        img.paste(bin_img, (cx, cy))

        # Eyelids y pupila/glint van encima en el canvas final (sin rotar)
        # — son aproximaciones; suficientes para el estilo cartoon.
        bbox_approx = self._eye_bbox(state, t, dx, dy)
        draw = ImageDraw.Draw(img)
        self._apply_eyelids(draw, bbox_approx, state)
        if state.pupil_visible:
            self._draw_pupil_glint(draw, bbox_approx, state)

    def _shape_arc(
        self, img: Image.Image, draw: ImageDraw.ImageDraw,
        state: EyeState, t: float, dx: int, dy: int, kind: str,
    ) -> None:
        bbox = self._eye_bbox(state, t, dx, dy)
        # Para arc_up usamos el rango angular 180-360 (parte superior),
        # para arc_down el 0-180 (parte inferior).
        if kind == "up":
            start, end = 180, 360
        else:
            start, end = 0, 180
        draw.arc(bbox, start, end, fill=_FG, width=state.arc_thickness)

    def _shape_line(
        self, img: Image.Image, draw: ImageDraw.ImageDraw,
        state: EyeState, t: float, dx: int, dy: int,
    ) -> None:
        bbox = self._eye_bbox(state, t, dx, dy)
        r = max(0, min(state.corner_radius, (bbox[3] - bbox[1]) // 2))
        draw.rounded_rectangle(bbox, radius=r, fill=_FG)

    def _shape_heart(
        self, img: Image.Image, draw: ImageDraw.ImageDraw,
        state: EyeState, t: float, dx: int, dy: int,
    ) -> None:
        # Pulse opcional
        scale = 1.0
        if "heart_pulse" in state.extras:
            scale = 1.0 + 0.05 * math.sin(2.0 * math.pi * 2.0 * t)
        elif "heart_pulse_slow" in state.extras:
            scale = 1.0 + 0.06 * math.sin(2.0 * math.pi * 1.5 * t)

        cx = self.width // 2 + dx
        cy = self.height // 2 + dy
        w = max(8, int(state.width * scale))
        h = max(8, int(state.height * scale))

        # Dos lóbulos circulares + triángulo inferior
        lobe_r = w // 4
        lobe_y = cy - h // 4
        lx = cx - lobe_r
        rx = cx + lobe_r
        draw.ellipse((lx - lobe_r, lobe_y - lobe_r, lx + lobe_r, lobe_y + lobe_r), fill=_FG)
        draw.ellipse((rx - lobe_r, lobe_y - lobe_r, rx + lobe_r, lobe_y + lobe_r), fill=_FG)
        # Triángulo
        tri = [
            (cx - w // 2, lobe_y),
            (cx + w // 2, lobe_y),
            (cx, cy + h // 2),
        ]
        draw.polygon(tri, fill=_FG)

    def _shape_xx(
        self, img: Image.Image, draw: ImageDraw.ImageDraw,
        state: EyeState, t: float, dx: int, dy: int,
    ) -> None:
        scale = 1.0
        if "bounce_xx" in state.extras:
            scale = 1.0 + 0.08 * math.sin(2.0 * math.pi * 3.0 * t)
        cx = self.width // 2 + dx
        cy = self.height // 2 + dy
        w = max(6, int(state.width * scale))
        h = max(6, int(state.height * scale))
        thick = max(1, state.arc_thickness)

        # Líneas cruzadas: sweep en perpendicular para grosor uniforme
        for off in range(-thick // 2, thick // 2 + 1):
            draw.line(
                (cx - w // 2 + off, cy - h // 2, cx + w // 2 + off, cy + h // 2),
                fill=_FG, width=1,
            )
            draw.line(
                (cx - w // 2 + off, cy + h // 2, cx + w // 2 + off, cy - h // 2),
                fill=_FG, width=1,
            )

    # ─────────────────────────────────────────────────────── eyelids / pupil

    def _apply_eyelids(
        self, draw: ImageDraw.ImageDraw,
        bbox: Tuple[int, int, int, int], state: EyeState,
    ) -> None:
        x0, y0, x1, y1 = bbox
        h = y1 - y0
        if state.eyelid_top_pct > 0:
            y_top = y0 + int(h * state.eyelid_top_pct)
            draw.rectangle((x0, y0, x1, y_top), fill=_BG)
        if state.eyelid_bottom_pct > 0:
            y_bot = y1 - int(h * state.eyelid_bottom_pct)
            draw.rectangle((x0, y_bot, x1, y1), fill=_BG)

    def _draw_pupil_glint(
        self, draw: ImageDraw.ImageDraw,
        bbox: Tuple[int, int, int, int], state: EyeState,
    ) -> None:
        x0, y0, x1, y1 = bbox
        cx = (x0 + x1) // 2
        cy = (y0 + y1) // 2
        eye_w = x1 - x0
        eye_h = y1 - y0
        # Tamaño base de pupila relativo al ojo
        base_r = max(2, min(eye_w, eye_h) // 3)
        r = max(1, int(base_r * state.pupil_size))
        # Posición de pupila normalizada (-1..+1)
        max_off_x = max(0, eye_w // 2 - r - 2)
        max_off_y = max(0, eye_h // 2 - r - 2)
        px = cx + int(state.pupil_x * max_off_x)
        py = cy + int(state.pupil_y * max_off_y)

        if state.pupil_shape == "heart":
            # Pupila tipo corazón pequeño
            self._draw_small_heart(draw, px, py, r * 2)
        else:
            draw.ellipse((px - r, py - r, px + r, py + r), fill=_BG)

        if state.glint_visible:
            gx = px + state.glint_offset[0]
            gy = py + state.glint_offset[1]
            gs = max(1, state.glint_size)
            draw.ellipse((gx - gs // 2, gy - gs // 2,
                          gx + gs // 2 + 1, gy + gs // 2 + 1), fill=_FG)

    def _draw_small_heart(
        self, draw: ImageDraw.ImageDraw, cx: int, cy: int, size: int,
    ) -> None:
        lobe_r = max(1, size // 4)
        lobe_y = cy - size // 4
        draw.ellipse((cx - lobe_r * 2, lobe_y - lobe_r,
                      cx, lobe_y + lobe_r), fill=_BG)
        draw.ellipse((cx, lobe_y - lobe_r,
                      cx + lobe_r * 2, lobe_y + lobe_r), fill=_BG)
        draw.polygon(
            [(cx - size // 2, lobe_y),
             (cx + size // 2, lobe_y),
             (cx, cy + size // 2)],
            fill=_BG,
        )

    # ─────────────────────────────────────────────────────── extras / decor

    def _jitter_offset(self, state: EyeState, t: float) -> Tuple[int, int]:
        dx = 0
        dy = 0
        if "jitter_tension" in state.extras:
            rng = random.Random(int(t * 20))
            dx += rng.randint(-1, 1)
            dy += rng.randint(-1, 1)
        if "burst_shake" in state.extras:
            phase = (t % 1.6)
            if phase < 0.3:
                rng = random.Random(int(t * 60))
                dx += rng.randint(-3, 3)
                dy += rng.randint(-3, 3)
        if "shake" in state.extras:
            dx += int(2 * math.sin(2.0 * math.pi * 6.0 * t))
            dy += int(2 * math.cos(2.0 * math.pi * 7.0 * t))
        return dx, dy

    def _draw_brow(
        self, draw: ImageDraw.ImageDraw, side: str, state: EyeState,
        t: float, dx: int, dy: int,
    ) -> None:
        r"""Dibuja una ceja angry corta por encima del rect del ojo.

        - Largo: state.width * 0.55 (escalado al ancho del ojo, no al canvas).
        - Y: 6-8 px arriba del borde superior del rect del ojo.
        - Slope: 4 px entre extremos. L = `\`, R = `/`.
        - Thickness: 3 px.
        - Brow jitter independiente del jitter del cuerpo (±1 px).
        """
        cx = self.width // 2 + dx
        cy = self.height // 2 + dy
        # Top del rect del ojo (sin tilt, suficiente como referencia visual)
        eye_top_y = cy - state.height // 2
        # Brow jitter independiente, leve (±1 px)
        rng = random.Random(int(t * 17) ^ (1 if side == "L" else 2))
        bjx = rng.randint(-1, 1)
        bjy = rng.randint(-1, 1)
        gap = 7  # separación brow_bottom ↔ eye_top
        slope = 4  # diferencia y entre extremos
        brow_bottom_y = max(0, eye_top_y - gap + bjy)
        brow_top_y = max(0, brow_bottom_y - slope)
        # Largo escalado al ancho del ojo, NO al canvas
        brow_len = max(8, int(state.width * 0.55))
        x_left = cx - brow_len // 2 + bjx
        x_right = cx + brow_len // 2 + bjx
        if side == "L":
            # outer (izq) alto → inner (der) bajo, slope \
            draw.line(
                (x_left, brow_top_y, x_right, brow_bottom_y),
                fill=_FG, width=3,
            )
        else:
            # inner (izq) bajo → outer (der) alto, slope /
            draw.line(
                (x_left, brow_bottom_y, x_right, brow_top_y),
                fill=_FG, width=3,
            )

    def _draw_anger_spark(
        self, draw: ImageDraw.ImageDraw, side: str, t: float,
    ) -> None:
        period = 1.5
        visible_for = 0.20
        phase = t % period
        if phase >= visible_for:
            return
        # Zigzag de 3 segmentos en una esquina superior
        if side == "L":
            ox = 8                   # ojo L → outer = lado izquierdo del canvas
        else:
            ox = self.width - 18     # ojo R → outer = lado derecho del canvas
        oy = 6
        pts = [
            (ox, oy),
            (ox + 4, oy + 4),
            (ox - 2, oy + 6),
            (ox + 5, oy + 10),
        ]
        for a, b in zip(pts[:-1], pts[1:]):
            draw.line((a, b), fill=_FG, width=1)

    def _draw_teardrop(
        self, draw: ImageDraw.ImageDraw, side: str, t: float,
    ) -> None:
        period = 2.5
        visible_for = 0.7
        phase = t % period
        if phase >= visible_for:
            return
        progress = phase / visible_for
        cx = self.width // 2 + (12 if side == "L" else -12)
        cy = self.height // 2 + 14
        dy = int(16 * progress)
        # Triángulo + circulito (gota)
        draw.polygon(
            [(cx, cy + dy - 6), (cx - 3, cy + dy - 1), (cx + 3, cy + dy - 1)],
            fill=_FG,
        )
        draw.ellipse((cx - 3, cy + dy - 2, cx + 3, cy + dy + 4), fill=_FG)

    def _draw_sparkles(
        self, draw: ImageDraw.ImageDraw, eye_cx: int, eye_cy: int, t: float,
    ) -> None:
        # 3 sparkles "+" en posiciones determinísticas
        positions = [(-24, -18), (28, -22), (22, 18)]
        period = 0.35
        visible_for = 0.18
        for i, (ox, oy) in enumerate(positions):
            phase_offset = i * period / 3
            phase = (t + phase_offset) % period
            if phase >= visible_for:
                continue
            x = eye_cx + ox
            y = eye_cy + oy
            draw.line((x - 2, y, x + 2, y), fill=_FG, width=1)
            draw.line((x, y - 2, x, y + 2), fill=_FG, width=1)

    def _draw_z_floating(
        self, draw: ImageDraw.ImageDraw, t: float, period: float, base_size: int,
    ) -> None:
        phase = (t % period) / period
        # Sube 18 px en el primer 70 %, fade-out en el último 30 %
        if phase < 0.7:
            rise = int(18 * (phase / 0.7))
            size = base_size
        else:
            rise = 18
            fade = (phase - 0.7) / 0.3
            size = max(1, int(base_size * (1.0 - fade)))
        cx = self.width - 22
        cy = 22 - rise
        if cy < -size:
            return
        self._draw_z_glyph(draw, cx, cy, size)

    def _draw_z_glyph(
        self, draw: ImageDraw.ImageDraw, cx: int, cy: int, size: int,
    ) -> None:
        if size < 2:
            return
        s = size
        # Barra superior
        draw.rectangle((cx - s, cy - s, cx + s, cy - s + 2), fill=_FG)
        # Barra inferior
        draw.rectangle((cx - s, cy + s - 2, cx + s, cy + s), fill=_FG)
        # Diagonal de top-right a bottom-left, 2 px de ancho
        for off in (-1, 0, 1):
            draw.line(
                (cx + s + off, cy - s, cx - s + off, cy + s),
                fill=_FG, width=1,
            )

    def _draw_eyelash_dots(
        self, draw: ImageDraw.ImageDraw, state: EyeState,
    ) -> None:
        cx = self.width // 2
        # Dots fijos debajo del arco hacia abajo
        cy = self.height // 2 + 4
        for ox in (-12, 0, 12):
            draw.ellipse((cx + ox - 1, cy - 1, cx + ox + 1, cy + 1), fill=_FG)
