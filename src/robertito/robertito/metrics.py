import time
from dataclasses import dataclass, fields
from typing import Dict, List, Optional, Tuple


@dataclass
class Metrics:
    t_wake: float = 0.0
    t_feedback_play: float = 0.0
    t_listen_start: float = 0.0
    t_user_end: float = 0.0
    t_stt_done: float = 0.0
    t_llm_first_token: float = 0.0
    t_tts_first_chunk_ready: float = 0.0
    t_audio_play_start: float = 0.0

    # ---------------------------------------------------------------- Helpers
    def reset(self) -> None:
        for field in fields(self):
            setattr(self, field.name, 0.0)

    def wake_to_feedback_ms(self) -> Optional[int]:
        if self.t_wake and self.t_feedback_play:
            return int((self.t_feedback_play - self.t_wake) * 1000)
        return None

    def wake_to_first_audio_ms(self) -> Optional[int]:
        if self.t_wake and self.t_tts_first_chunk_ready:
            return int((self.t_tts_first_chunk_ready - self.t_wake) * 1000)
        return None

    def round_trip_ms(self, round_end: Optional[float]) -> Optional[int]:
        if self.t_wake and round_end:
            return int((round_end - self.t_wake) * 1000)
        return None

    def log(self, logger, round_end: Optional[float] = None) -> None:
        if feedback := self.wake_to_feedback_ms():
            logger.info(f"wake->feedback_ms={feedback}")
        if first_audio := self.wake_to_first_audio_ms():
            logger.info(f"wake->first_audio_ms={first_audio}")
        if total := self.round_trip_ms(round_end):
            logger.info(f"round_trip_ms={total}")


class TurnTimer:
    """Per-turn pipeline latency tracker."""

    def __init__(self) -> None:
        self._t: Dict[str, Tuple[float, float]] = {}  # stage → (start, end)
        self._order: List[str] = []

    def reset(self) -> None:
        self._t.clear()
        self._order.clear()

    def start(self, stage: str) -> None:
        if stage not in self._t:
            self._order.append(stage)
        self._t[stage] = (time.monotonic(), self._t.get(stage, (0.0, 0.0))[1])

    def stop(self, stage: str) -> None:
        if stage in self._t:
            self._t[stage] = (self._t[stage][0], time.monotonic())

    def summary(self) -> str:
        """Return [TURN] A→B=Xms B→C=Yms ... total=Tms"""
        points: List[Tuple[str, float]] = []
        for stage in self._order:
            if stage not in self._t:
                continue
            t_start, t_end = self._t[stage]
            points.append((stage, t_end if t_end else t_start))
        if len(points) < 2:
            return "[TURN] sin datos"
        parts: List[str] = []
        for i in range(1, len(points)):
            prev_name, prev_t = points[i - 1]
            curr_name, curr_t = points[i]
            parts.append(f"{prev_name}→{curr_name}={int((curr_t - prev_t) * 1000)}ms")
        total_ms = int((points[-1][1] - points[0][1]) * 1000)
        parts.append(f"total={total_ms}ms")
        return "[TURN] " + " ".join(parts)
