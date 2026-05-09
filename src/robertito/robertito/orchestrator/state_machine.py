"""Estados primarios del orchestrator y máquina de estados básica."""
from __future__ import annotations

import logging
from enum import Enum
from typing import Optional


class State(Enum):
    """Estados primarios del comportamiento de Robertito."""

    WAKING_UP = "waking_up"
    ENGAGED = "engaged"
    IDLE = "idle"
    DROWSY = "drowsy"
    SLEEPING = "sleeping"
    WORKING = "working"


class StateMachine:
    """Storage simple del estado actual y timestamp de entrada.

    No maneja transiciones por sí misma — el orchestrator decide cuándo
    llamar a transition_to(); esta clase solo guarda y loguea.
    """

    def __init__(self, initial: State, now: float, logger: Optional[logging.Logger] = None) -> None:
        self._state: State = initial
        self._entered_at: float = float(now)
        self._logger = logger or logging.getLogger(__name__)

    @property
    def current_state(self) -> State:
        return self._state

    @property
    def entered_at(self) -> float:
        return self._entered_at

    def transition_to(self, new_state: State, now: float) -> bool:
        """Cambia de estado. Devuelve True si efectivamente cambió."""
        if new_state == self._state:
            return False
        prev = self._state
        self._state = new_state
        self._entered_at = float(now)
        self._logger.info(f"FSM {prev.value} → {new_state.value}")
        return True

    def time_in_state(self, now: float) -> float:
        return max(0.0, float(now) - self._entered_at)

    def is_resting(self) -> bool:
        return self._state in (State.DROWSY, State.SLEEPING)

    def is_active(self) -> bool:
        return self._state in (State.ENGAGED, State.WORKING)
