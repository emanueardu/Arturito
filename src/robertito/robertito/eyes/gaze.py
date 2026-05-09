"""Gaze controller: face tracking + saccades + glance overrides."""
from __future__ import annotations

from dataclasses import replace
from typing import Optional, Tuple

from robertito.eyes.easing import clamp, ease_in_out_cubic, lerp, random_saccade
from robertito.eyes.state import EyeState, EyesPair


class GazeController:
    """Modula pupil_x/pupil_y según face tracking, saccades idle y glance."""

    def __init__(
        self,
        face_timeout_s: float = 1.0,
        idle_freq_s: float = 1.8,
        idle_amp_x: float = 0.25,
        idle_amp_y: float = 0.18,
        seed: int = 1,
    ) -> None:
        self.face_timeout_s = face_timeout_s
        self.idle_freq_s = idle_freq_s
        self.idle_amp_x = idle_amp_x
        self.idle_amp_y = idle_amp_y
        self.seed = seed
        self.speaking: bool = False
        self._face_target: Optional[Tuple[float, float]] = None
        self._face_target_t: float = 0.0
        self._glance_lock: Optional[Tuple[float, float, float]] = None  # (x, y, end_t)
        # Lag suave hacia face_target
        self._smoothed_x: float = 0.0
        self._smoothed_y: float = 0.0
        self._last_update_t: float = 0.0

    def set_face(self, target_x: float, target_y: float, t: float) -> None:
        """Set face target en rango -1..+1."""
        self._face_target = (clamp(target_x, -1.0, 1.0), clamp(target_y, -1.0, 1.0))
        self._face_target_t = t

    def clear_face(self) -> None:
        self._face_target = None

    def set_glance(self, direction: str, t: float, duration: float = 1.0) -> None:
        if direction == "left":
            self._glance_lock = (-0.7, 0.0, t + duration)
        elif direction == "right":
            self._glance_lock = (0.7, 0.0, t + duration)

    def set_speaking(self, active: bool) -> None:
        self.speaking = active

    def update(self, t: float, pair: EyesPair) -> EyesPair:
        """Modifica pupil_x/pupil_y de los ojos según fuente de gaze.

        Si la expresión define gaze_lock_x/y específico, se respeta.
        """
        target_x, target_y = self._compute_target(t)

        def _apply(eye: EyeState) -> EyeState:
            # Respetar override del preset
            if eye.gaze_lock_x is not None and eye.gaze_lock_y is not None:
                return eye
            if not eye.pupil_visible:
                return eye
            # En presets con pupil_x asimétrica (picaron, enojado), respetar
            # la posición ya marcada (no overwrite) si está fuera de cero.
            base_px = eye.pupil_x
            base_py = eye.pupil_y
            new_px = base_px + target_x * 0.5
            new_py = base_py + target_y * 0.5
            new_px = clamp(new_px, -1.0, 1.0)
            new_py = clamp(new_py, -1.0, 1.0)
            return replace(eye, pupil_x=new_px, pupil_y=new_py)

        return EyesPair(
            left=_apply(pair.left),
            right=_apply(pair.right),
            asymmetric=pair.asymmetric,
        )

    def _compute_target(self, t: float) -> Tuple[float, float]:
        # Glance lock tiene prioridad
        if self._glance_lock is not None:
            x, y, end_t = self._glance_lock
            if t < end_t:
                return (x, y)
            self._glance_lock = None

        # Face target reciente
        if self._face_target is not None:
            age = t - self._face_target_t
            if age <= self.face_timeout_s:
                # Lag exponencial suave hacia el target (~150 ms)
                tx, ty = self._face_target
                if self._last_update_t > 0:
                    dt = max(0.0, t - self._last_update_t)
                    alpha = clamp(dt / 0.15, 0.0, 1.0)
                else:
                    alpha = 0.5
                self._smoothed_x = lerp(self._smoothed_x, tx, alpha)
                self._smoothed_y = lerp(self._smoothed_y, ty, alpha)
                self._last_update_t = t
                return (self._smoothed_x, self._smoothed_y)
            self._face_target = None

        # Saccades idle (modulados por speaking)
        freq = self.idle_freq_s
        ax = self.idle_amp_x
        ay = self.idle_amp_y
        if self.speaking:
            freq = freq / 1.6
            ax *= 0.5
            ay *= 0.5
        x, y = random_saccade(t, self.seed, freq_s=freq, amp_x=ax, amp_y=ay)
        self._smoothed_x = x
        self._smoothed_y = y
        self._last_update_t = t
        return (x, y)
