"""Controlador de parpadeo con squash & stretch."""
from __future__ import annotations

import enum
import random
from dataclasses import replace
from typing import Tuple

from robertito.eyes.easing import clamp
from robertito.eyes.state import EyesPair


class BlinkPhase(enum.IntEnum):
    IDLE = 0
    CLOSING = 1
    CLOSED = 2
    OPENING = 3


class BlinkController:
    """Maneja el ciclo de parpadeo con squash & stretch.

    5 frames a 20 Hz = 250 ms total:
      CLOSING:  2 frames (lid: 0 → 0.5 → 1.0)
      CLOSED:   1 frame  (lid: 1.0)
      OPENING:  2 frames (lid: 1.0 → 0.5 → 0)
    """

    _CLOSING_FRAMES = 2
    _CLOSED_FRAMES = 1
    _OPENING_FRAMES = 2
    _TOTAL_FRAMES = _CLOSING_FRAMES + _CLOSED_FRAMES + _OPENING_FRAMES

    def __init__(
        self,
        interval_s: float = 4.0,
        jitter_s: float = 2.0,
        frame_dt_s: float = 0.05,
    ) -> None:
        self.interval_s = interval_s
        self.jitter_s = jitter_s
        self.frame_dt_s = max(0.001, frame_dt_s)
        self._rng = random.Random(0xB1)
        self._next_blink_t: float = 0.0
        self._initialized: bool = False
        self._blink_start_t: float = 0.0
        self._active: bool = False

    def force_blink(self, t: float) -> None:
        """Dispara un blink ahora (si no hay uno en curso)."""
        if not self._active:
            self._active = True
            self._blink_start_t = t

    def update(self, t: float, pair: EyesPair) -> EyesPair:
        """Aplica el parpadeo al pair y devuelve el EyesPair modificado."""
        if not self._initialized:
            self._next_blink_t = t + self._next_interval()
            self._initialized = True

        disabled = bool(pair.left.blink_disabled or pair.right.blink_disabled)

        if not self._active and not disabled and t >= self._next_blink_t:
            self._active = True
            self._blink_start_t = t

        if not self._active:
            return pair

        elapsed = t - self._blink_start_t
        frame_idx = int(elapsed / self.frame_dt_s)

        if frame_idx >= self._TOTAL_FRAMES:
            self._active = False
            self._next_blink_t = t + self._next_interval()
            return pair

        lid_pct, squash_px = self._frame_to_state(frame_idx)
        return self._apply(pair, lid_pct, squash_px)

    @staticmethod
    def _apply(pair: EyesPair, lid_pct: float, squash_px: int) -> EyesPair:
        l = pair.left
        r = pair.right
        if l.blink_disabled or r.blink_disabled:
            return pair
        new_left = replace(
            l,
            eyelid_top_pct=max(l.eyelid_top_pct, lid_pct),
            width=max(2, l.width + squash_px),
        )
        new_right = replace(
            r,
            eyelid_top_pct=max(r.eyelid_top_pct, lid_pct),
            width=max(2, r.width + squash_px),
        )
        return EyesPair(left=new_left, right=new_right, asymmetric=pair.asymmetric)

    @staticmethod
    def _frame_to_state(frame_idx: int) -> Tuple[float, int]:
        """Mapea índice de frame a (lid_pct, squash_extra_w_px)."""
        if frame_idx < 0:
            return 0.0, 0
        if frame_idx < BlinkController._CLOSING_FRAMES:
            lid = (frame_idx + 1) / BlinkController._CLOSING_FRAMES
            return lid, int(round(2.0 * lid))
        if frame_idx < BlinkController._CLOSING_FRAMES + BlinkController._CLOSED_FRAMES:
            return 1.0, 2
        opening_idx = (
            frame_idx
            - BlinkController._CLOSING_FRAMES
            - BlinkController._CLOSED_FRAMES
        )
        if opening_idx < BlinkController._OPENING_FRAMES:
            lid = 1.0 - (opening_idx + 1) / BlinkController._OPENING_FRAMES
            return clamp(lid, 0.0, 1.0), int(round(2.0 * lid))
        return 0.0, 0

    def _next_interval(self) -> float:
        return max(
            0.5, self.interval_s + self._rng.uniform(-self.jitter_s, self.jitter_s)
        )
