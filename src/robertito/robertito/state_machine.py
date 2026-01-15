from enum import Enum
from threading import Lock


class AssistantState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    THINKING = "thinking"
    SPEAKING = "speaking"


class AssistantStateMachine:
    """Thread-safe wrapper around the assistant's state."""

    def __init__(self) -> None:
        self._state = AssistantState.IDLE
        self._lock = Lock()

    def transition(self, new_state: AssistantState) -> AssistantState:
        with self._lock:
            previous = self._state
            if previous is new_state:
                return previous
            self._state = new_state
            return previous

    def current(self) -> AssistantState:
        with self._lock:
            return self._state

    def reset(self) -> None:
        with self._lock:
            self._state = AssistantState.IDLE
