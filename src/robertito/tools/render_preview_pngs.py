#!/usr/bin/env python3
"""Genera previews PNG de las 13 expresiones canónicas (L+R + grid).

Uso:
    cd ~/ros2_ws
    PYTHONPATH=src/robertito python3 src/robertito/tools/render_preview_pngs.py

Output:
    /tmp/eyes_preview_<expr>_L.png   (uno por expresión)
    /tmp/eyes_preview_<expr>_R.png   (uno por expresión)
    /tmp/eyes_preview_grid.png       (grid 5x3 con todas)

No requiere ROS, solo Pillow.
"""
from __future__ import annotations

import os
import sys
import time

# Si el script se corre fuera del workspace, agregar src/robertito al sys.path.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from robertito.eyes import (  # noqa: E402
    EXPRESSIONS, BlinkController, EyesAnimator, EyesRenderer, GazeController,
)


_OUT_DIR = "/tmp"
_W, _H = 128, 64
_PREVIEW_T = 0.5  # segundos dentro de la animación a capturar
_FRAME_DT = 0.05


def _capture_frame(expr_name: str, target_t: float = _PREVIEW_T) -> tuple[Image.Image, Image.Image]:
    """Construye animator + renderer y devuelve (left, right) en t=target_t."""
    renderer = EyesRenderer(_W, _H)
    animator = EyesAnimator(transition_duration_s=0.0)
    gaze = GazeController(seed=42)
    blink = BlinkController(interval_s=10.0, jitter_s=0.0, frame_dt_s=_FRAME_DT)

    # Setear target sin transición
    animator.set_target(expr_name)
    # Avanzar tiempo en pasos de _FRAME_DT hasta target_t
    t = 0.0
    pair = animator.tick(t, gaze, blink)
    while t < target_t:
        t += _FRAME_DT
        pair = animator.tick(t, gaze, blink)

    left = renderer.render(pair.left, "L", t)
    right = renderer.render(pair.right, "R", t)
    return left, right


def _scale(img: Image.Image, factor: int = 3) -> Image.Image:
    return img.convert("L").resize(
        (img.width * factor, img.height * factor), Image.NEAREST
    )


def main() -> int:
    os.makedirs(_OUT_DIR, exist_ok=True)
    expressions = sorted(EXPRESSIONS.keys())
    n = len(expressions)
    print(f"Renderizando {n} expresiones a {_OUT_DIR}/eyes_preview_*.png ...")

    captured: dict[str, tuple[Image.Image, Image.Image]] = {}
    failed: list[str] = []

    for name in expressions:
        try:
            left, right = _capture_frame(name)
            left.save(f"{_OUT_DIR}/eyes_preview_{name}_L.png")
            right.save(f"{_OUT_DIR}/eyes_preview_{name}_R.png")
            captured[name] = (left, right)
            print(f"  ✔ {name}")
        except Exception as exc:
            print(f"  ✘ {name}: {exc}")
            failed.append(name)

    if failed:
        print(f"\nFALLARON {len(failed)} expresiones: {failed}")
        return 2

    # Grid 5×3 (15 slots → 13 usados, 2 vacíos al final)
    cols = 5
    rows = 3
    cell_w = _W * 3 + 4 * 2  # L + sep + R
    cell_h = _H * 3 + 16     # con espacio para label arriba
    pad = 8
    grid_w = pad + cols * (cell_w + pad)
    grid_h = pad + rows * (cell_h + pad)
    grid = Image.new("L", (grid_w, grid_h), 32)  # gris oscuro
    draw = ImageDraw.Draw(grid)
    font = ImageFont.load_default()

    for i, name in enumerate(expressions):
        col = i % cols
        row = i // cols
        x0 = pad + col * (cell_w + pad)
        y0 = pad + row * (cell_h + pad)
        # Label
        draw.text((x0, y0), name, fill=240, font=font)
        # L y R escalados x3 lado a lado
        left, right = captured[name]
        l3 = _scale(left, 3)
        r3 = _scale(right, 3)
        grid.paste(l3, (x0, y0 + 14))
        grid.paste(r3, (x0 + l3.width + 4, y0 + 14))

    grid_path = f"{_OUT_DIR}/eyes_preview_grid.png"
    grid.save(grid_path)
    print(f"\nGrid: {grid_path}")
    print(f"Total PNGs: {2 * n + 1} ({n} L + {n} R + 1 grid)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
