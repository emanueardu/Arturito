from dataclasses import dataclass, fields
from typing import Optional


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
