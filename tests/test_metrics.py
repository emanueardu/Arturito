from robertito.metrics import Metrics


class DummyLogger:
    def __init__(self):
        self.messages = []

    def info(self, message: str):
        self.messages.append(message)


def test_metrics_log_round_trip():
    metrics = Metrics()
    metrics.t_wake = 0.01
    metrics.t_feedback_play = 0.1
    metrics.t_tts_first_chunk_ready = 0.5
    # Round trip measured at 1.2 seconds
    logger = DummyLogger()

    metrics.log(logger, round_end=1.2)
    assert any("wake->feedback_ms" in msg for msg in logger.messages)
    assert any("wake->first_audio_ms" in msg for msg in logger.messages)
    assert any("round_trip_ms" in msg for msg in logger.messages)
