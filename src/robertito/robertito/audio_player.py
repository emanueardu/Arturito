import io
import queue
import threading
import subprocess
import time
import wave
from dataclasses import dataclass
from typing import Callable, Optional


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
    """Reproductor persistente con cola y backends ajustables.

    Pre-pad de silencio: si pasaron `lead_in_idle_threshold_s` desde la última
    reproducción, el parlante BT puede haber entrado en standby. Antes de
    pushear el chunk de audio "real" pusheamos `lead_in_silence_s` segundos
    de silencio para que la fase de wake-up del speaker NO se coma el
    principio de la frase. Si las frases vienen pegadas, el pre-pad se
    omite (el speaker ya está despierto).
    """

    def __init__(
        self,
        backend: str = "sounddevice",
        device_index: Optional[int] = None,
        device_name: Optional[str] = None,
        dtype: str = "int16",
        state_callback: Optional[Callable[[bool], None]] = None,
        error_callback: Optional[Callable[[str], None]] = None,
        lead_in_silence_s: float = 0.35,
        lead_in_idle_threshold_s: float = 4.0,
    ) -> None:
        self._backend = backend.lower()
        self._device_index = device_index
        self._device_name = device_name
        self._dtype = dtype
        self._state_callback = state_callback
        self._error_callback = error_callback
        self._queue: "queue.Queue[AudioChunk]" = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()
        self._playback_active = False
        self._lead_in_silence_s = max(0.0, float(lead_in_silence_s))
        self._lead_in_idle_thr_s = max(0.0, float(lead_in_idle_threshold_s))
        self._last_enqueue_at: float = 0.0
        # Keepalive: cuando no hay audio activo, escribimos pulsos de silencio
        # al stream para que el parlante BT nunca caiga en standby a nivel
        # hardware. Independiente del wireplumber no-suspend (que actúa en
        # la capa pipewire pero el speaker decide solo a nivel codec).
        self._keepalive_enabled = True
        self._keepalive_after_s = 1.5  # tras 1.5s sin audio, empezar keepalive
        self._keepalive_chunk_s = 0.05  # 50ms de silencio por iteración
        self._last_audio_write_at: float = 0.0

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

    @property
    def backend(self) -> str:
        """Expose the backend currently in use for logging."""
        return self._backend

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
        # Pre-pad de silencio si el speaker BT pudo haber entrado en standby.
        # Solo si pasó suficiente tiempo desde el último encolado.
        now = time.monotonic()
        if (
            self._lead_in_silence_s > 0.0
            and (now - self._last_enqueue_at) > self._lead_in_idle_thr_s
        ):
            n_samples = int(fmt.sample_rate * self._lead_in_silence_s)
            sample_width = max(1, int(fmt.sample_width))
            channels = max(1, int(fmt.channels))
            silence = b"\x00" * (n_samples * channels * sample_width)
            if silence:
                self._queue.put(AudioChunk(pcm_data=silence, fmt=fmt))
        self._queue.put(AudioChunk(pcm_data=pcm_data, fmt=fmt))
        self._last_enqueue_at = now

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
                self._maybe_emit_keepalive()
                continue
            try:
                if chunk.fmt.sample_rate == 0:
                    continue
                self._set_playback_active(True)
                self._play_chunk(chunk)
                self._last_audio_write_at = time.monotonic()
            finally:
                if self._queue.empty():
                    self._set_playback_active(False)
                try:
                    self._queue.task_done()
                except ValueError:
                    pass

    def _maybe_emit_keepalive(self) -> None:
        """Escribe silencio al stream si pasó suficiente tiempo idle.

        Mantiene el parlante BT despierto a nivel hardware (no depende de
        que wireplumber/pulse hagan algo: escribe directamente al backend).
        Solo activo si el stream ya está abierto (no abre uno nuevo solo
        para keepalive).
        """
        if not self._keepalive_enabled:
            return
        if self._backend != "sounddevice":
            return  # solo soportado en sounddevice (stream persistente)
        if self._sd_stream is None or self._current_format is None:
            return
        now = time.monotonic()
        if (now - self._last_audio_write_at) < self._keepalive_after_s:
            return
        fmt = self._current_format
        try:
            n_samples = int(fmt.sample_rate * self._keepalive_chunk_s)
            sample_width = max(1, int(fmt.sample_width))
            channels = max(1, int(fmt.channels))
            silence = b"\x00" * (n_samples * channels * sample_width)
            if silence:
                self._sd_stream.write(silence)
                # NO actualizamos _last_audio_write_at: queremos que el
                # próximo loop emita otro keepalive y mantenga el flow.
        except Exception:
            # Si falla, no rompemos el worker; lo intentamos en el siguiente.
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
            self._report_error("sounddevice stream unavailable")
            return
        if self._sd_stream is None:
            self._report_error("sounddevice stream unavailable")
            return
        try:
            self._sd_stream.write(chunk.pcm_data)
        except Exception as exc:
            self._report_error(f"sounddevice playback failed: {exc}")

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
        except Exception as exc:
            self._sd_stream = None
            self._current_format = None
            self._report_error(f"sounddevice stream open failed: {exc}")
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
            self._report_error("pyaudio backend unavailable")
            return
        if self._pyaudio_stream is None or self._current_format != chunk.fmt:
            self._open_pyaudio_stream(chunk.fmt)
        if self._pyaudio_stream is None:
            self._report_error("pyaudio stream unavailable")
            return
        try:
            self._pyaudio_stream.write(chunk.pcm_data)
        except Exception as exc:
            self._report_error(f"pyaudio playback failed: {exc}")

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
        except Exception as exc:
            self._report_error(f"pyaudio stream open failed: {exc}")
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
        except Exception as exc:
            self._report_error(f"aplay playback failed: {exc}")

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

    def _set_playback_active(self, active: bool) -> None:
        if self._playback_active == active:
            return
        self._playback_active = active
        if self._state_callback is not None:
            try:
                self._state_callback(active)
            except Exception:
                pass

    def _report_error(self, message: str) -> None:
        if self._error_callback is not None:
            try:
                self._error_callback(message)
            except Exception:
                pass
