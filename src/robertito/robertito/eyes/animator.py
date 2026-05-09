"""Interpolación entre presets de expresiones con ease cúbico.

Campos numéricos se interpolan suave durante `transition_duration_s`. Campos
no-numéricos (shape, extras, etc.) switchean al inicio de la transición.

Si `transition_duration_s <= 1e-3` se hace SNAP inmediato al target en
`set_target()` (caso preview o usuario que no quiere transición).
"""
from __future__ import annotations

import time
from dataclasses import replace
from typing import Optional

from robertito.eyes.easing import clamp, ease_in_out_cubic, lerp
from robertito.eyes.expressions import EXPRESSIONS
from robertito.eyes.state import EyeState, EyesPair


# Umbral por debajo del cual se considera "sin transición".
_SNAP_THRESHOLD_S = 1e-3


# Campos numéricos a interpolar entre estados.
_NUMERIC_FIELDS: tuple[str, ...] = (
    "width", "height", "corner_radius",
    "pupil_size", "pupil_x", "pupil_y",
    "eyelid_top_pct", "eyelid_bottom_pct",
    "inclination_deg", "breath_freq_hz", "breath_amp_px",
    "arc_thickness", "glint_size",
)


def _interp_eye(a: EyeState, b: EyeState, t: float) -> EyeState:
    """Interpola campos numéricos de a→b con factor t (0..1).

    Campos no-numéricos (shape, extras, pupil_visible, etc.) toman el valor
    de `b` cuando t > 0 (cambian al inicio de la transición).
    """
    if t <= 0.0:
        return a
    if t >= 1.0:
        return b
    overrides: dict = {}
    for name in _NUMERIC_FIELDS:
        va = getattr(a, name)
        vb = getattr(b, name)
        if isinstance(va, int) and isinstance(vb, int):
            overrides[name] = int(round(lerp(float(va), float(vb), t)))
        else:
            overrides[name] = lerp(float(va), float(vb), t)
    overrides.update(
        shape=b.shape,
        pupil_shape=b.pupil_shape,
        pupil_visible=b.pupil_visible,
        glint_visible=b.glint_visible,
        glint_offset=b.glint_offset,
        show_brow=b.show_brow,
        brow_style=b.brow_style,
        extras=b.extras,
        inclination_mode=b.inclination_mode,
        blink_disabled=b.blink_disabled,
        gaze_lock_x=b.gaze_lock_x,
        gaze_lock_y=b.gaze_lock_y,
        gaze_lock_duration_s=b.gaze_lock_duration_s,
    )
    return replace(a, **overrides)


class EyesAnimator:
    """Mantiene `current_pair` y maneja transiciones a `target_pair`."""

    def __init__(self, transition_duration_s: float = 0.2) -> None:
        self.transition_duration_s = transition_duration_s
        self.current_pair: EyesPair = EXPRESSIONS["calma"]
        self.target_pair: EyesPair = EXPRESSIONS["calma"]
        self._target_name: str = "calma"
        # None hasta que `tick()` lo registra con el tiempo lógico del caller
        # — esto desacopla el reloj de `set_target()` del de `tick()` y resuelve
        # el bug en el que el preview llama tick(0.0) tras set_target() seteado
        # con time.monotonic() (~12345s) → elapsed muy negativo.
        self._transition_start_t: Optional[float] = None
        self._transitioning: bool = False

    def set_target(self, name: str) -> None:
        """Cambia la expresión objetivo.

        Si `transition_duration_s <= 1e-3`, hace snap inmediato (no espera a
        `tick()`). En caso contrario marca la transición pendiente y deja que
        el primer `tick()` registre el origen temporal con el tiempo lógico
        del caller.
        """
        if name not in EXPRESSIONS:
            return
        # Si ya estamos targetando este nombre (con o sin transición en curso),
        # no hacer nada. Sin esto, llamar set_target() cada frame con el mismo
        # nombre durante la transición resetea _transition_start_t=None y la
        # transición nunca progresa → ojos congelados en current_pair (bug v1).
        if name == self._target_name:
            return
        self.target_pair = EXPRESSIONS[name]
        self._target_name = name
        if self.transition_duration_s <= _SNAP_THRESHOLD_S:
            # Snap inmediato: current = target, sin transición pendiente.
            self.current_pair = self.target_pair
            self._transitioning = False
            self._transition_start_t = None
        else:
            self._transitioning = True
            self._transition_start_t = None  # se establece en el primer tick()

    def _sample_current(self, t: float) -> EyesPair:
        if not self._transitioning:
            return self.current_pair
        if self._transition_start_t is None:
            self._transition_start_t = t
        elapsed = t - self._transition_start_t
        if self.transition_duration_s <= _SNAP_THRESHOLD_S:
            progress = 1.0
        else:
            progress = clamp(elapsed / self.transition_duration_s, 0.0, 1.0)
        eased = ease_in_out_cubic(progress)
        left = _interp_eye(self.current_pair.left, self.target_pair.left, eased)
        right = _interp_eye(self.current_pair.right, self.target_pair.right, eased)
        return EyesPair(left=left, right=right, asymmetric=self.target_pair.asymmetric)

    def tick(self, t: float, gaze, blink) -> EyesPair:
        """Devuelve el EyesPair interpolado para este frame."""
        if self._transitioning:
            if self.transition_duration_s <= _SNAP_THRESHOLD_S:
                # Snap defensivo (set_target ya debió aplicarlo).
                self.current_pair = self.target_pair
                self._transitioning = False
                pair = self.current_pair
            else:
                if self._transition_start_t is None:
                    self._transition_start_t = t
                elapsed = t - self._transition_start_t
                if elapsed >= self.transition_duration_s:
                    self.current_pair = self.target_pair
                    self._transitioning = False
                    pair = self.current_pair
                else:
                    pair = self._sample_current(t)
        else:
            pair = self.current_pair

        pair = gaze.update(t, pair)
        pair = blink.update(t, pair)
        return pair


if __name__ == "__main__":
    from robertito.eyes import EXPRESSIONS as _EXPR  # noqa: F401  (re-import OK)
    from robertito.eyes.blink import BlinkController
    from robertito.eyes.gaze import GazeController

    g = GazeController(seed=42)
    b = BlinkController(interval_s=10.0, jitter_s=0.0, frame_dt_s=0.05)

    # Test 1: con duration=0 debe snappear inmediato
    a = EyesAnimator(transition_duration_s=0.0)
    a.set_target("corazones")
    pair = a.tick(0.0, g, b)
    assert pair.left.shape == "heart", f"FAIL: shape={pair.left.shape}"
    print("OK · snap immediate (duration=0)")

    # Test 2: con duration=0.2 después de 0.5s debe estar en target
    a = EyesAnimator(transition_duration_s=0.2)
    a.set_target("risa_fuerte")
    _ = a.tick(0.0, g, b)
    pair = a.tick(0.5, g, b)
    assert pair.left.shape == "xx", f"FAIL: shape={pair.left.shape}"
    print("OK · transition completed (duration=0.2, t=0.5)")

    # Test 3: con duration=0.2 a t=0.0 todavía está en current (no crash)
    a = EyesAnimator(transition_duration_s=0.2)
    a.set_target("dormido")
    pair = a.tick(0.0, g, b)
    print("OK · transition in progress (no crash)")
