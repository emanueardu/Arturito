"""Helpers puros sin estado para easing, lerp, breath y saccades.

Todos los timing usan time.monotonic() (vía el caller). Todos los random usan
random.Random(seed) determinístico para que los previews sean reproducibles.
"""
from __future__ import annotations

import math
import random
from typing import Tuple


def clamp(v: float, lo: float, hi: float) -> float:
    """Devuelve `v` recortado al rango [lo, hi]."""
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def lerp(a: float, b: float, t: float) -> float:
    """Interpola linealmente entre `a` y `b` con factor `t` (no clamped)."""
    return a + (b - a) * t


def ease_in_out_cubic(t: float) -> float:
    """Ease cúbico simétrico. t fuera de [0,1] se clampea."""
    t = clamp(t, 0.0, 1.0)
    if t < 0.5:
        return 4.0 * t * t * t
    inv = -2.0 * t + 2.0
    return 1.0 - (inv * inv * inv) / 2.0


def breath(t: float, freq_hz: float = 0.3) -> float:
    """Onda sinusoidal ±1 para modular respiración."""
    return math.sin(2.0 * math.pi * freq_hz * t)


def random_saccade(
    t: float,
    seed: int,
    freq_s: float = 1.8,
    amp_x: float = 0.25,
    amp_y: float = 0.18,
    transition_s: float = 0.2,
) -> Tuple[float, float]:
    """Devuelve (px, py) en rango aprox. [-amp, +amp].

    El movimiento se compone de fixaciones discretas cada `freq_s` segundos.
    Entre fixaciones hay una transición suave de `transition_s` segundos.
    Determinístico por (seed, índice de fixación).
    """
    if freq_s <= 0:
        return (0.0, 0.0)
    idx = int(t // freq_s)
    local_t = t - idx * freq_s

    # Seed determinístico via combinación lineal (Python 3.12 no acepta tuplas).
    rng_curr = random.Random(seed * 1_000_003 + idx)
    cx = rng_curr.uniform(-amp_x, amp_x)
    cy = rng_curr.uniform(-amp_y, amp_y)

    rng_prev = random.Random(seed * 1_000_003 + (idx - 1))
    px = rng_prev.uniform(-amp_x, amp_x)
    py = rng_prev.uniform(-amp_y, amp_y)

    if local_t < transition_s:
        e = ease_in_out_cubic(local_t / transition_s)
        return (lerp(px, cx, e), lerp(py, cy, e))
    return (cx, cy)


if __name__ == "__main__":
    # Smoke tests
    assert ease_in_out_cubic(0.0) == 0.0
    assert ease_in_out_cubic(1.0) == 1.0
    assert abs(ease_in_out_cubic(0.5) - 0.5) < 1e-9
    assert clamp(5, 0, 3) == 3
    assert clamp(-1, 0, 3) == 0
    assert lerp(0.0, 10.0, 0.5) == 5.0
    print("easing.py self-test OK")
