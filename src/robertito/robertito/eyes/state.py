"""Dataclasses inmutables que describen el estado visual de los ojos."""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional, Tuple


@dataclass(frozen=True)
class EyeState:
    """Estado de UN ojo individual.

    Todos los campos son opcionales con defaults razonables (estilo CALMA).
    Las dimensiones son en pixels dentro del canvas 128x64 del OLED.
    """

    # Forma del ojo base
    shape: str = "rect"
    # "rect" | "arc_up" | "arc_down" | "line" | "heart" | "xx" | "inclined_rect"
    width: int = 72
    height: int = 48
    corner_radius: int = 14
    inclination_deg: float = 0.0
    inclination_mode: str = "none"  # "none" | "outer_drop" | "inner_drop"
    arc_thickness: int = 10

    # Pupila
    pupil_visible: bool = True
    pupil_size: float = 1.0
    pupil_x: float = 0.0
    pupil_y: float = 0.0
    pupil_shape: str = "circle"  # "circle" | "heart"

    # Glint
    glint_visible: bool = True
    glint_size: int = 3
    glint_offset: Tuple[int, int] = (-4, -4)

    # Párpados
    eyelid_top_pct: float = 0.0
    eyelid_bottom_pct: float = 0.0

    # Decoración estática
    show_brow: bool = False
    brow_style: str = "angry"

    # Respiración
    breath_freq_hz: float = 0.3
    breath_amp_px: int = 1

    # Animación
    blink_disabled: bool = False
    extras: Tuple[str, ...] = ()
    # ("teardrop", "heart_pulse", "shake", "sparkles",
    #  "z_floating_slow", "z_floating_fast", "eyelash_dots", "anger_spark",
    #  "jitter_tension", "burst_shake", "heart_pulse_slow", "wink_dynamic",
    #  "sag_dynamic", "bounce_xx")

    # Gaze override (para glance_left/right o presets con mirada fija)
    gaze_lock_x: Optional[float] = None
    gaze_lock_y: Optional[float] = None
    gaze_lock_duration_s: float = 0.0

    def with_(self, **overrides) -> "EyeState":
        """Devuelve copia con los campos sobrescritos."""
        return replace(self, **overrides)


@dataclass(frozen=True)
class EyesPair:
    """Par de ojos. Si asymmetric=False, left==right en runtime."""

    left: EyeState
    right: EyeState
    asymmetric: bool = False
