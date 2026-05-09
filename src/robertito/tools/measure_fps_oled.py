#!/usr/bin/env python3
"""Medición de FPS efectivo del pipeline de ojos contra OLEDs reales.

Sin ROS: instancia el renderer + animator + controllers y empuja frames de
'calma' (con respiración + saccades + parpadeo) durante 10 s. Reporta:

  FPS efectivo
  Latencia min/avg/max por display (ms)
  Si el bus está saturado vs CPU bound (push_ms vs render_ms)

Uso:
    cd ~/ros2_ws
    PYTHONPATH=src/robertito python3 src/robertito/tools/measure_fps_oled.py

ATENCIÓN: si el nodo robertito_eyes está corriendo, va a competir por el bus.
Detené el servicio antes de ejecutar.
"""
from __future__ import annotations

import os
import statistics
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from luma.core.interface.serial import i2c  # noqa: E402
from luma.oled.device import sh1106  # noqa: E402

from robertito.eyes import (  # noqa: E402
    BlinkController, EyesAnimator, EyesRenderer, GazeController,
)


def main() -> int:
    bus = 1
    left_addr = 0x3C
    right_addr = 0x3D
    duration_s = 10.0
    target_hz = 20.0

    print(f"Conectando OLEDs (bus={bus}, L=0x{left_addr:02X}, R=0x{right_addr:02X}) ...")
    left = sh1106(i2c(port=bus, address=left_addr))
    right = sh1106(i2c(port=bus, address=right_addr))

    renderer = EyesRenderer(left.width, left.height)
    animator = EyesAnimator(transition_duration_s=0.0)
    gaze = GazeController(seed=7)
    blink = BlinkController(interval_s=4.0, jitter_s=2.0, frame_dt_s=1.0 / target_hz)
    animator.set_target("calma")

    print(f"Midiendo {duration_s:.0f}s a target {target_hz:.0f} Hz ...")
    render_ms: list[float] = []
    push_ms: list[float] = []
    t0 = time.monotonic()
    deadline = t0 + duration_s
    frames = 0
    while time.monotonic() < deadline:
        t = time.monotonic()
        # render
        r0 = time.monotonic()
        pair = animator.tick(t, gaze, blink)
        l_img = renderer.render(pair.left, "L", t)
        r_img = renderer.render(pair.right, "R", t)
        render_ms.append((time.monotonic() - r0) * 1000.0)
        # push a OLED
        p0 = time.monotonic()
        left.display(l_img)
        right.display(r_img)
        push_ms.append((time.monotonic() - p0) * 1000.0)
        frames += 1

    elapsed = time.monotonic() - t0
    fps = frames / elapsed
    rmin, ravg, rmax = min(render_ms), statistics.mean(render_ms), max(render_ms)
    pmin, pavg, pmax = min(push_ms), statistics.mean(push_ms), max(push_ms)

    print()
    print(f"{'metric':<25} {'min':>8} {'avg':>8} {'max':>8}")
    print("-" * 53)
    print(f"{'render (ms)':<25} {rmin:>8.2f} {ravg:>8.2f} {rmax:>8.2f}")
    print(f"{'push 2× display (ms)':<25} {pmin:>8.2f} {pavg:>8.2f} {pmax:>8.2f}")
    print()
    print(f"Frames: {frames}    elapsed: {elapsed:.2f}s    FPS: {fps:.1f}")
    print(f"Target: {target_hz:.0f} Hz")

    bottleneck = "BUS (push)" if pavg > ravg else "CPU (render)"
    print(f"Cuello de botella estimado: {bottleneck}")
    if fps < target_hz * 0.9:
        print(f"⚠ FPS por debajo del 90% del target. Considerá bajar a {fps * 0.9:.0f} Hz.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
