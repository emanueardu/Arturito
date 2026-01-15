import io
import queue
import threading
import subprocess
import wave
from dataclasses import dataclass
from typing import Optional


def _lazy_import(module_name: str, friendly_name: str):
    try:
        return __import__(module_name)
    except ImportError as exc:  # pragma: no cover - dependency guidance
        raise RuntimeError(
            f"El módulo opcional '{friendly_name}' ({module_name}) es requerido. "
            "Instalalo con 'pip install {friendly_name}' y/o revisá la documentación."
        ) from exc


@dataclass
class AudioFormat:
    sample_rate: int
    channels: int
    sample_width: int


@dataclass
class AudioChunk:
    pcm_data: bytes
    fmt: AudioFormat


class AudioPlayer:
    """Reproductor persistente con cola y backends ajustables."""

    def __init__(
        self,
        backend: str = "sounddevice",
        device_index: Optional[int] = None,
        device_name: Optional[str] = None,
        dtype: str = "int16",
    ) -> None:
        self._backend = backend.lower()
        self._device_index = device_index
        self._device_name = device_name
        self._dtype = dtype
        self._queue: "queue.Queue[AudioChunk]" = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

        self._sounddevice = None
        self._sd_stream = None
        self._current_format: Optional[AudioFormat] = None

        self._pyaudio = None
        self._pyaudio_instance = None
        self._pyaudio_stream = None

        if self._backend == "sounddevice":
            try:
                self._sounddevice = _lazy_import("sounddevice", "sounddevice")
            except RuntimeError:
                self._sounddevice = None
        elif self._backend == "pyaudio":
            try:
                self._pyaudio = _lazy_import("pyaudio", "pyaudio")
            except RuntimeError:
                self._pyaudio = None

    # ------------------------------------------------------------------ API
    def close(self) -> None:
        self._stop_event.set()
        try:
            self._queue.put_nowait(AudioChunk(b"", AudioFormat(0, 0, 0)))
        except queue.Full:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._close_streams()

    def stop(self) -> None:
        self.flush()

    def flush(self) -> None:
        while True:
            try:
                chunk = self._queue.get_nowait()
            except queue.Empty:
                break
            finally:
                try:
                    self._queue.task_done()
                except ValueError:
                    pass

    def reset(self) -> None:
        """Drop queued chunks and restart the backend stream."""
        self.flush()
        self._close_streams()

    def play_pcm(self, pcm_data: bytes, fmt: AudioFormat) -> None:
        if not pcm_data or fmt.sample_rate <= 0:
            return
        self._queue.put(AudioChunk(pcm_data=pcm_data, fmt=fmt))

    def play_wav_bytes(self, wav_bytes: bytes) -> None:
        if not wav_bytes:
            return
        with wave.open(io.BytesIO(wav_bytes), "rb") as wav:
            fmt = AudioFormat(
                sample_rate=wav.getframerate(),
                channels=wav.getnchannels(),
                sample_width=wav.getsampwidth(),
            )
            pcm = wav.readframes(wav.getnframes())
        self.play_pcm(pcm, fmt)

    # ------------------------------------------------------------ Worker loop
    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                chunk = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if chunk.fmt.sample_rate == 0:
                    continue
                self._play_chunk(chunk)
            finally:
                try:
                    self._queue.task_done()
                except ValueError:
                    pass

    def _play_chunk(self, chunk: AudioChunk) -> None:
        if self._backend == "sounddevice":
            self._play_with_sounddevice(chunk)
            return
        if self._backend == "aplay":
            self._play_with_aplay(chunk)
            return
        if self._backend != "pyaudio" or self._pyaudio is None:
            return
        self._play_with_pyaudio(chunk)

    # ------------------------------------------------------------- SoundDevice
    def _play_with_sounddevice(self, chunk: AudioChunk) -> None:
        if not self._ensure_sounddevice_stream(chunk.fmt):
            return
        if self._sd_stream is None:
            return
        try:
            self._sd_stream.write(chunk.pcm_data)
        except Exception:
            pass

    def _ensure_sounddevice_stream(self, fmt: AudioFormat) -> bool:
        if self._sounddevice is None:
            return False
        if self._sd_stream is not None and self._current_format == fmt:
            return True
        self._close_sounddevice_stream()
        try:
            device = self._device_index if self._device_index is not None else self._device_name
            self._sd_stream = self._sounddevice.RawOutputStream(
                samplerate=fmt.sample_rate,
                channels=fmt.channels,
                dtype=self._dtype,
                device=device,
            )
            self._sd_stream.start()
            self._current_format = fmt
            return True
        except Exception:
            self._sd_stream = None
            self._current_format = None
            return False

    def _close_sounddevice_stream(self) -> None:
        if self._sd_stream is not None:
            try:
                self._sd_stream.stop()
                self._sd_stream.close()
            except Exception:
                pass
        self._sd_stream = None
        self._current_format = None

    # ------------------------------------------------------------- PyAudio
    def _play_with_pyaudio(self, chunk: AudioChunk) -> None:
        if self._pyaudio is None:
            return
        if self._pyaudio_stream is None or self._current_format != chunk.fmt:
            self._open_pyaudio_stream(chunk.fmt)
        if self._pyaudio_stream is None:
            return
        try:
            self._pyaudio_stream.write(chunk.pcm_data)
        except Exception:
            pass

    def _open_pyaudio_stream(self, fmt: AudioFormat) -> None:
        self._close_pyaudio_stream()
        if self._pyaudio is None:
            return
        self._pyaudio_instance = self._pyaudio.PyAudio()
        format_id = self._pyaudio_instance.get_format_from_width(fmt.sample_width)
        try:
            self._pyaudio_stream = self._pyaudio_instance.open(
                format=format_id,
                channels=fmt.channels,
                rate=fmt.sample_rate,
                output=True,
                output_device_index=self._device_index,
            )
            self._current_format = fmt
        except Exception:
            self._close_pyaudio_stream()

    def _close_pyaudio_stream(self) -> None:
        if self._pyaudio_stream is not None:
            try:
                self._pyaudio_stream.stop_stream()
                self._pyaudio_stream.close()
            except Exception:
                pass
        if self._pyaudio_instance is not None:
            try:
                self._pyaudio_instance.terminate()
            except Exception:
                pass
        self._pyaudio_stream = None
        self._pyaudio_instance = None
        self._current_format = None

    # -------------------------------------------------------------- APlay
    def _play_with_aplay(self, chunk: AudioChunk) -> None:
        fmt = chunk.fmt
        cmd = [
            "aplay",
            "-q",
            "-f",
            self._aplay_format(fmt.sample_width),
            "-r",
            str(fmt.sample_rate),
            "-c",
            str(fmt.channels),
        ]
        if self._device_name:
            cmd.extend(["-D", self._device_name])
        try:
            subprocess.run(cmd, input=chunk.pcm_data, check=False)
        except Exception:
            pass

    @staticmethod
    def _aplay_format(sample_width: int) -> str:
        if sample_width == 1:
            return "U8"
        if sample_width == 2:
            return "S16_LE"
        if sample_width == 3:
            return "S24_LE"
        return "S16_LE"

    def _close_streams(self) -> None:
        self._close_sounddevice_stream()
        self._close_pyaudio_stream()
