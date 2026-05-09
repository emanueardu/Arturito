import hashlib
import html
import itertools
import json
import os
import queue
import re
import shutil
import shlex
import subprocess
import threading
import time
from datetime import datetime, timedelta
import urllib.error
import urllib.request
import unicodedata
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from robertito.tts_ssml import build_ssml
from robertito.voice_style_formatter import format_jarvis_style
from robertito.audio_player import AudioFormat, AudioPlayer

try:  # optional Azure SDK
    import azure.cognitiveservices.speech as speechsdk  # type: ignore
except ImportError:  # pragma: no cover
    speechsdk = None  # type: ignore


@dataclass
class PiperConfig:
    binary: str
    model: str
    config: str
    speaker: int
    noise_scale: float
    length_scale: float
    noise_w: float
    sentence_silence: float
    espeak_data: str


class _PiperProcess:
    """Persistent piper subprocess — model loaded once, reused for all syntheses."""

    def __init__(self, cfg: PiperConfig, length_scale: float) -> None:
        self._cfg = cfg
        self._length_scale = max(0.01, length_scale)
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._env = os.environ.copy()
        self._env["LD_LIBRARY_PATH"] = os.path.dirname(cfg.binary)

    def _build_cmd(self) -> list:
        return [
            self._cfg.binary,
            "--model", self._cfg.model,
            "--config", self._cfg.config,
            "--output_file", "-",
            "--speaker", str(self._cfg.speaker),
            "--noise_scale", str(self._cfg.noise_scale),
            "--length_scale", str(self._length_scale),
            "--noise_w", str(self._cfg.noise_w),
            "--sentence_silence", str(self._cfg.sentence_silence),
            "--espeak_data", self._cfg.espeak_data,
            "--json-input",
        ]

    def start(self) -> None:
        self._proc = subprocess.Popen(
            self._build_cmd(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            env=self._env,
        )

    def synthesize(self, text: str) -> bytes:
        """Synthesize text and return WAV bytes. Thread-safe."""
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                self.start()
            payload = json.dumps({"text": text}) + "\n"
            self._proc.stdin.write(payload.encode("utf-8"))
            self._proc.stdin.flush()
            return self._read_wav()

    def _read_wav(self) -> bytes:
        # WAV: "RIFF" (4 bytes) + payload_size (4 bytes LE) + payload
        header = self._read_exact(8)
        if header[:4] != b"RIFF":
            raise RuntimeError(f"Piper stdout: expected RIFF, got {header[:4]!r}")
        payload_size = int.from_bytes(header[4:8], "little")
        return header + self._read_exact(payload_size)

    def _read_exact(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self._proc.stdout.read(n - len(buf))
            if not chunk:
                raise RuntimeError("Piper stdout cerrado inesperadamente")
            buf += chunk
        return buf

    def close(self) -> None:
        proc, self._proc = self._proc, None
        if proc is None:
            return
        try:
            proc.stdin.close()
        except OSError:
            pass
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()


_SOURCE_PREFIX_RE = re.compile(r"^\[([^\]]+)\]\s*(.*)")
_BAD_PHRASE = "ajuste adicional"
_RAW_TTS_SOURCES = {"robertito_eyes"}
_CUSTOM_BREAK_SPEECH = {
    "Sí, ¿en qué puedo ayudarte?": ("Sí,", 150, " ¿en qué puedo ayudarte?"),
    "Buenos días, soy Robertito, estoy listo para funcionar": (
        "Buenos días,",
        200,
        " soy Robertito, estoy listo para funcionar",
    ),
}
_PREWARM_PHRASES = (
    "Sí, te escucho.",
    "Decime.",
    "Acá estoy.",
    "Dale, contame.",
    "Sí, Emanuel.",
    "A ver, decime.",
    "Listo, te escucho.",
    "Listo.",
    "Dale.",
    "Perfecto.",
    "Ya está.",
    "Claro.",
    "No te escuché bien.",
    "No llegué a entenderte.",
    "Probá de nuevo.",
)
_SHORT_REPLY_CACHE_ALIASES = {
    "si te escucho": "Sí, te escucho.",
    "si, te escucho": "Sí, te escucho.",
    "decime": "Decime.",
    "aca estoy": "Acá estoy.",
    "dale contame": "Dale, contame.",
    "listo": "Listo.",
    "dale": "Dale.",
    "perfecto": "Perfecto.",
    "ya esta": "Ya está.",
    "claro": "Claro.",
    "no te escuche bien": "No te escuché bien.",
    "no llegue a entenderte": "No llegué a entenderte.",
    "proba de nuevo": "Probá de nuevo.",
}
_TIME_REPLY_RE = re.compile(r"^(son las|es la)\s+(\d{1,2}):(\d{2})\.?$", re.IGNORECASE)
_HIGH_PRIORITY_TTS_SOURCES = {
    "api_chat_node",
    "wake_word_listener",
    "robertito_node_manager",
}
_LOW_PRIORITY_TTS_PREFIXES = (
    "behavior_",
    "presence_",
    "robertito_eyes",
)


class VoiceSynthNode(Node):
    """ROS 2 node that formats assistant text and performs TTS playback."""

    def __init__(self) -> None:
        super().__init__("voice_synth_node")

        self._input_topic = (
            self.declare_parameter("input_topic", "/assistant/say")
            .get_parameter_value()
            .string_value
        )
        self._voice_mode = (
            self.declare_parameter("voice_mode", "normal")
            .get_parameter_value()
            .string_value
            .lower()
        )
        self._language = (
            self.declare_parameter("language", "es-AR")
            .get_parameter_value()
            .string_value
        )
        self._rate = float(self.declare_parameter("rate", 0.92).value)
        self._pitch = (
            self.declare_parameter("pitch", "-2st")
            .get_parameter_value()
            .string_value
        )
        self._pause_short_ms = int(self.declare_parameter("pause_short_ms", 200).value)
        self._pause_medium_ms = int(
            self.declare_parameter("pause_medium_ms", 300).value
        )
        self._tts_provider = (
            self.declare_parameter("tts_provider", "piper")
            .get_parameter_value()
            .string_value
            .lower()
        )
        default_azure_voice = os.getenv("AZURE_TTS_VOICE", "es-AR-TomasNeural")
        self._voice_id = (
            self.declare_parameter("voice_id", "").get_parameter_value().string_value
        )
        self._voice_name = (
            self.declare_parameter("voice_name", default_azure_voice)
            .get_parameter_value()
            .string_value
        )
        azure_voice_name = (
            self.declare_parameter("azure_voice_name", default_azure_voice)
            .get_parameter_value()
            .string_value
        )
        self._tts_voice_name = (
            self._voice_name or azure_voice_name or default_azure_voice
        )
        self._tts_rate = float(self.declare_parameter("tts_rate", 0.90).value)
        self._tts_pitch = (
            self.declare_parameter("tts_pitch", "-1st")
            .get_parameter_value()
            .string_value
        )
        azure_output_format = (
            self.declare_parameter("azure_output_format", "")
            .get_parameter_value()
            .string_value
        )
        tts_output_format = (
            self.declare_parameter("tts_output_format", "Riff24Khz16BitMonoPcm")
            .get_parameter_value()
            .string_value
        )
        self._tts_output_format = (
            tts_output_format or azure_output_format or "Riff24Khz16BitMonoPcm"
        )
        self._output_format_enum: Optional[
            "speechsdk.SpeechSynthesisOutputFormat"
        ] = None
        use_ssml_flag = bool(self.declare_parameter("use_ssml", True).value)
        enable_ssml_flag = bool(self.declare_parameter("enable_ssml", True).value)
        self._use_ssml = use_ssml_flag and enable_ssml_flag
        self._supports_ssml = bool(
            self.declare_parameter("supports_ssml", False).value
        )
        self._cache_enabled = bool(
            self.declare_parameter("cache_enabled", True).value
        )
        self._startup_message = (
            self.declare_parameter("startup_message", "")
            .get_parameter_value()
            .string_value
        )
        self._startup_delay = float(self.declare_parameter("startup_delay", 0.5).value)
        default_cache = os.path.join(
            os.path.expanduser("~"), ".cache", "robertito", "voice"
        )
        self._cache_dir = (
            self.declare_parameter("cache_dir", default_cache)
            .get_parameter_value()
            .string_value
        )
        self._fast_reply_time_horizon_min = int(
            self.declare_parameter("fast_reply_time_horizon_min", 20).value
        )
        self._audio_backend = (
            self.declare_parameter(
                "audio_backend", os.getenv("AUDIO_BACKEND", "alsa")
            )
            .get_parameter_value()
            .string_value
            .lower()
        )
        self._audio_device = (
            self.declare_parameter(
                "audio_device", os.getenv("AUDIO_DEVICE", "default")
            )
            .get_parameter_value()
            .string_value
        )

        self._play_command = (
            self.declare_parameter("play_command", "").get_parameter_value().string_value
        )
        self._play_command = self._resolve_play_command(self._play_command)
        # Pre-pad de silencio antes de cada frase si pasó tiempo desde la
        # última: enmascara el wake-up del speaker BT (si está dormido los
        # primeros ms se cortan). Si las frases vienen pegadas, no pre-padea.
        self._lead_in_silence_s = float(
            self.declare_parameter("lead_in_silence_s", 0.35).value
        )
        self._lead_in_idle_threshold_s = float(
            self.declare_parameter("lead_in_idle_threshold_s", 4.0).value
        )
        self._tts_command = (
            self.declare_parameter("tts_command", "").get_parameter_value().string_value
        )
        self._azure_key_env = (
            self.declare_parameter("azure_key_env", "AZURE_SPEECH_KEY")
            .get_parameter_value()
            .string_value
        )
        self._azure_region_env = (
            self.declare_parameter("azure_region_env", "AZURE_SPEECH_REGION")
            .get_parameter_value()
            .string_value
        )
        default_env_file = os.path.join(
            os.path.expanduser("~"), ".config", "robertito.env"
        )
        self._env_file = (
            self.declare_parameter("env_file", default_env_file)
            .get_parameter_value()
            .string_value
        )

        self._piper_cfg = self._load_piper_config()

        # ─── ElevenLabs provider (lazy init si tts_provider == elevenlabs) ───
        self._el_provider = None
        if self._tts_provider == "elevenlabs":
            try:
                from .elevenlabs_provider import (
                    ElevenLabsProvider, load_api_key_from_file
                )
                el_voice_id = (
                    self.declare_parameter("elevenlabs_voice_id", "")
                    .get_parameter_value().string_value
                )
                el_key_file = (
                    self.declare_parameter(
                        "elevenlabs_api_key_file",
                        "/home/robot/.config/robertito/elevenlabs.env"
                    )
                    .get_parameter_value().string_value
                )
                el_model = (
                    self.declare_parameter(
                        "elevenlabs_model_id", "eleven_multilingual_v2"
                    )
                    .get_parameter_value().string_value
                )
                api_key = load_api_key_from_file(el_key_file)
                if not api_key:
                    self.get_logger().error(
                        f"tts_provider=elevenlabs pero no encontre key en {el_key_file}; "
                        f"caigo a piper"
                    )
                    self._tts_provider = "piper"
                elif not el_voice_id:
                    self.get_logger().error(
                        "tts_provider=elevenlabs pero elevenlabs_voice_id vacio; "
                        "caigo a piper"
                    )
                    self._tts_provider = "piper"
                else:
                    self._el_provider = ElevenLabsProvider(
                        api_key=api_key,
                        voice_id=el_voice_id,
                        model_id=el_model,
                        logger=self.get_logger(),
                    )
                    self.get_logger().info(
                        f"TTS provider: elevenlabs "
                        f"(voice={el_voice_id[:8]}, model={el_model})"
                    )
            except Exception as exc:
                self.get_logger().error(
                    f"ElevenLabs init fallo: {exc}; caigo a piper"
                )
                self._tts_provider = "piper"
                self._el_provider = None
        self._piper_process: Optional[_PiperProcess] = None
        if self._tts_provider == "piper":
            self._piper_process = self._init_piper_process()

        self._queue: "queue.PriorityQueue[tuple[int, int, str]]" = queue.PriorityQueue()
        self._queue_sequence = itertools.count()
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._startup_timer = None
        self._speaking_active_topic = (
            self.declare_parameter("speaking_active_topic", "/behavior/speaking_active")
            .get_parameter_value()
            .string_value
        )
        self._health_topic = (
            self.declare_parameter("health_topic", "/behavior/health/voice")
            .get_parameter_value()
            .string_value
        )
        self._health_heartbeat_sec = float(
            self.declare_parameter("health_heartbeat_sec", 5.0).value
        )
        self._speaking_pub = self.create_publisher(Bool, self._speaking_active_topic, 10)
        self._health_pub = self.create_publisher(String, self._health_topic, 10)
        self._speaking_active = False
        self._health_state = "ok"
        self._audio_player = self._build_audio_player()
        self._azure_lock = threading.Lock()
        self._azure_synthesizer = None
        self._load_env_file_if_needed()

        self._subscription = self.create_subscription(
            String, self._input_topic, self._on_text, 10
        )
        self._stop_sub = self.create_subscription(
            Bool, "/assistant/stop", self._on_stop, 10
        )
        self.get_logger().info(
            f"Voice synth node listening on '{self._input_topic}' (mode={self._voice_mode})."
        )
        self._health_timer = self.create_timer(
            max(1.0, self._health_heartbeat_sec), self._publish_health_heartbeat
        )
        self._publish_health("ok")
        if self._startup_message:
            self._schedule_startup_message()
        self._prewarm_thread = threading.Thread(
            target=self._prewarm_common_phrases, name="tts-prewarm", daemon=True
        )
        self._prewarm_thread.start()
        self._time_prewarm_thread = threading.Thread(
            target=self._prewarm_time_replies_loop,
            name="tts-time-prewarm",
            daemon=True,
        )
        self._time_prewarm_thread.start()

    def _load_piper_config(self) -> PiperConfig:
        piper_dir = (
            self.declare_parameter("piper_dir", "/home/robot/ros2_ws/piper")
            .get_parameter_value()
            .string_value
        )
        piper_lib_dir = os.path.join(piper_dir, "piper")
        return PiperConfig(
            binary=self.declare_parameter(
                "piper_binary", os.path.join(piper_lib_dir, "piper")
            )
            .get_parameter_value()
            .string_value,
            model=self.declare_parameter(
                "piper_model", os.path.join(piper_dir, "es_MX-claude-high.onnx")
            )
            .get_parameter_value()
            .string_value,
            config=self.declare_parameter(
                "piper_config", os.path.join(piper_dir, "es_MX-claude-high.onnx.json")
            )
            .get_parameter_value()
            .string_value,
            speaker=int(self.declare_parameter("piper_speaker", 0).value),
            noise_scale=float(self.declare_parameter("piper_noise_scale", 0.667).value),
            length_scale=float(self.declare_parameter("piper_length_scale", 0.0).value),
            noise_w=float(self.declare_parameter("piper_noise_w", 0.8).value),
            sentence_silence=float(
                self.declare_parameter("piper_sentence_silence", 0.3).value
            ),
            espeak_data=self.declare_parameter(
                "piper_espeak_data", os.path.join(piper_lib_dir, "espeak-ng-data")
            )
            .get_parameter_value()
            .string_value,
        )

    def _init_piper_process(self) -> "Optional[_PiperProcess]":
        if not os.path.isfile(self._piper_cfg.binary):
            self.get_logger().error(
                f"Piper binary no encontrado: {self._piper_cfg.binary}"
            )
            return None
        if not os.path.isfile(self._piper_cfg.model):
            self.get_logger().error(
                f"Modelo Piper no encontrado: {self._piper_cfg.model}"
            )
            return None
        length_scale = self._piper_cfg.length_scale
        if length_scale <= 0:
            length_scale = self._rate_to_length_scale(self._effective_rate())
        proc = _PiperProcess(self._piper_cfg, length_scale)
        self.get_logger().info(
            f"Cargando modelo Piper desde {self._piper_cfg.model}..."
        )
        t0 = time.monotonic()
        try:
            proc.start()
            proc.synthesize("Hola.")  # warm-up: fuerza la carga del modelo ONNX
            dt = time.monotonic() - t0
            self.get_logger().info(f"Modelo Piper cargado en {dt:.2f}s, listo")
        except Exception as exc:
            self.get_logger().error(
                f"Error al inicializar proceso Piper: {exc}. "
                "Cayendo a subprocess por síntesis (comportamiento anterior)."
            )
            try:
                proc.close()
            except Exception:
                pass
            return None
        return proc

    def _schedule_startup_message(self) -> None:
        delay = max(0.0, float(self._startup_delay))
        if delay == 0.0:
            self._enqueue_startup_message()
            return
        self._startup_timer = self.create_timer(delay, self._on_startup_timer)

    def _on_startup_timer(self) -> None:
        self._enqueue_startup_message()
        if self._startup_timer is not None:
            timer = self._startup_timer
            self._startup_timer = None
            timer.cancel()
            self.destroy_timer(timer)

    def _enqueue_startup_message(self) -> None:
        try:
            self._enqueue_text(self._startup_message)
            self.get_logger().info("Startup message queued for speech.")
        except queue.Full:
            self.get_logger().warning("Speech queue full, dropping startup message.")

    @classmethod
    def _resolve_play_command(cls, play_command: str) -> str:
        cleaned = (play_command or "").strip().lower()
        if cleaned in {"internal", "none", "disabled"}:
            return ""
        if play_command:
            return cls._ensure_wav_placeholder(play_command)
        paplay_path = shutil.which("paplay")
        if paplay_path:
            return cls._ensure_wav_placeholder(f"{paplay_path} {{wav}}")
        aplay_path = shutil.which("aplay")
        if aplay_path:
            return cls._ensure_wav_placeholder(f"{aplay_path} {{wav}}")
        return ""

    @staticmethod
    def _ensure_wav_placeholder(command: str) -> str:
        cleaned = (command or "").strip()
        if not cleaned:
            return ""
        if "{wav}" not in cleaned:
            cleaned = f"{cleaned} {{wav}}"
        return cleaned

    def _build_audio_player(self) -> AudioPlayer:
        preferred = (self._audio_backend or "alsa").strip().lower()
        device = self._audio_device or "default"
        if preferred in ("sounddevice", "sd"):
            selected_backend = "sounddevice"
        elif preferred in ("pyaudio", "pa"):
            selected_backend = "pyaudio"
        else:
            selected_backend = "aplay"
        play_command_state = "configured" if self._play_command else "unset"
        self.get_logger().info(
            f"Audio player backend resolved to '{selected_backend}' "
            f"(requested='{preferred}', device='{device}', play_command={play_command_state})."
        )
        if selected_backend in ("sounddevice", "pyaudio"):
            return AudioPlayer(
                backend=selected_backend,
                device_name=device,
                state_callback=self._on_playback_state_change,
                error_callback=self._on_audio_player_error,
                lead_in_silence_s=self._lead_in_silence_s,
                lead_in_idle_threshold_s=self._lead_in_idle_threshold_s,
            )
        return AudioPlayer(
            backend="aplay",
            device_name=device,
            state_callback=self._on_playback_state_change,
            error_callback=self._on_audio_player_error,
            lead_in_silence_s=self._lead_in_silence_s,
            lead_in_idle_threshold_s=self._lead_in_idle_threshold_s,
        )

    def _load_env_file_if_needed(self) -> None:
        needed = (self._azure_key_env, self._azure_region_env, "AZURE_TTS_VOICE")
        if all(os.getenv(key) for key in needed):
            return
        env_path = self._env_file
        if not env_path or not os.path.isfile(env_path):
            return
        try:
            with open(env_path, "r", encoding="utf-8") as handle:
                for raw_line in handle:
                    line = raw_line.strip()
                    if not line or line.startswith("#"):
                        continue
                    if line.startswith("export "):
                        line = line[len("export ") :]
                    if "=" not in line:
                        continue
                    key, value = line.split("=", 1)
                    key = key.strip()
                    if not key or os.getenv(key):
                        continue
                    value = value.strip().strip("\"'").strip()
                    os.environ[key] = value
        except Exception:
            self.get_logger().warning(
                "No se pudo cargar credenciales desde env_file."
            )

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            self._audio_player.stop()
            self._set_speaking_active(False)
            while not self._queue.empty():
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                except queue.Empty:
                    break

    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        try:
            self._enqueue_text(text)
        except queue.Full:
            self.get_logger().warning("Voice queue full, dropping message.")

    def _enqueue_text(self, text: str) -> None:
        priority = self._priority_for_text(text)
        self._queue.put_nowait((priority, next(self._queue_sequence), text))

    def _worker_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                _, _, text = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._handle_text(text)
            finally:
                self._queue.task_done()

    @staticmethod
    def _priority_for_text(text: str) -> int:
        source, _ = VoiceSynthNode._extract_source(text)
        normalized_source = (source or "").strip().casefold()
        if normalized_source in _HIGH_PRIORITY_TTS_SOURCES:
            return 0
        if any(
            normalized_source.startswith(prefix) for prefix in _LOW_PRIORITY_TTS_PREFIXES
        ):
            return 20
        return 10

    def _handle_text(self, text: str) -> None:
        source, normalized_text = self._extract_source(text)
        if source:
            self.get_logger().info(f"Fuente TTS: {source}")
        filtered = self._filter_sensitive(normalized_text)
        if not filtered:
            self.get_logger().warning("Texto bloqueado por filtro de seguridad.")
            return
        formatted_text = self._format_text(filtered, source=source)
        ssml_text = None
        if self._use_ssml and self._supports_ssml:
            custom_ssml = self._custom_ssml(formatted_text)
            if custom_ssml:
                ssml_text = custom_ssml
            else:
                ssml_text = build_ssml(
                    formatted_text,
                    voice_name=self._tts_voice_name
                    if self._tts_provider == "azure"
                    else None,
                    rate_pct=self._tts_rate,
                    pitch=self._tts_pitch,
                    short_ms=self._pause_short_ms,
                    medium_ms=self._pause_medium_ms,
                    language=self._language,
                )

        start = time.monotonic()
        cache_hit = False
        try:
            if self._tts_provider == "piper":
                audio_path, cache_hit = self._synthesize_piper(formatted_text)
            elif self._tts_provider == "elevenlabs":
                audio_path, cache_hit = self._synthesize_elevenlabs(formatted_text)
            elif self._tts_provider == "azure":
                audio_path, cache_hit = self._synthesize_azure(formatted_text, ssml_text)
            else:
                audio_path = self._synthesize_via_command(formatted_text, ssml_text)
            if not audio_path:
                return
            latency_ms = int((time.monotonic() - start) * 1000)
            self.get_logger().info(
                f"TTS ready (provider={self._tts_provider}, cache_hit={cache_hit}, latency_ms={latency_ms})"
            )
            self._publish_health("ok")
            self._play_audio(audio_path)
        except Exception as exc:
            self.get_logger().error(f"TTS error: {exc}")
            self._publish_health(f"error: tts falló ({exc})")
            if ssml_text:
                self.get_logger().info("Retrying TTS without SSML.")
                try:
                    audio_path = self._synthesize_via_command(formatted_text, None)
                    if audio_path:
                        self._publish_health("ok")
                        self._play_audio(audio_path)
                except Exception as retry_exc:
                    self.get_logger().error(f"Fallback TTS failed: {retry_exc}")
                    self._publish_health(f"error: fallback tts falló ({retry_exc})")

    def _prewarm_common_phrases(self) -> None:
        if not self._cache_enabled:
            return
        phrases = list(_PREWARM_PHRASES)
        if self._startup_message:
            phrases.append(self._startup_message.strip())
        seen: set[str] = set()
        for phrase in phrases:
            cleaned = phrase.strip()
            if not cleaned or cleaned in seen:
                continue
            seen.add(cleaned)
            try:
                if self._tts_provider == "piper":
                    self._synthesize_piper(cleaned)
                elif self._tts_provider == "azure":
                    self._synthesize_azure(cleaned, None)
            except Exception as exc:
                self.get_logger().debug(f"Prewarm omitido para '{cleaned}': {exc}")

    def _prewarm_time_replies_loop(self) -> None:
        if not self._cache_enabled or self._tts_provider != "piper":
            return
        while not self._shutdown.is_set():
            try:
                if self._queue.empty():
                    self._prewarm_likely_time_replies()
            except Exception as exc:
                self.get_logger().debug(f"Prewarm de hora omitido: {exc}")
            self._shutdown.wait(30.0)

    def _prewarm_likely_time_replies(self) -> None:
        horizon = max(0, self._fast_reply_time_horizon_min)
        now = datetime.now()
        for minute_offset in range(horizon + 1):
            target = now + timedelta(minutes=minute_offset)
            self._synthesize_piper(f"Son las {target:%H:%M}.")

    def _format_text(self, text: str, source: Optional[str] = None) -> str:
        if source and source.casefold() in _RAW_TTS_SOURCES:
            return text
        if self._voice_mode != "jarvis":
            return text
        return format_jarvis_style(text)

    @staticmethod
    def _extract_source(text: str) -> tuple[Optional[str], str]:
        if not text:
            return None, ""
        match = _SOURCE_PREFIX_RE.match(text)
        if not match:
            return None, text
        source = match.group(1).strip()
        remainder = match.group(2) or ""
        return source or None, remainder.strip()

    @staticmethod
    def _normalize_text(text: str) -> str:
        normalized = unicodedata.normalize("NFD", text)
        stripped = "".join(
            ch for ch in normalized if not unicodedata.combining(ch)
        )
        return stripped

    def _filter_sensitive(self, text: str) -> str:
        cleaned = text.strip()
        if not cleaned:
            return ""
        parts = re.split(r"(?<=[.!?])\s+", cleaned)
        result = []
        for part in parts:
            if not part:
                continue
            normalized = self._normalize_text(part).casefold()
            if _BAD_PHRASE in normalized:
                continue
            result.append(part)
        return " ".join(result).strip()

    def _custom_ssml(self, text: str) -> Optional[str]:
        payload = _CUSTOM_BREAK_SPEECH.get(text)
        if not payload:
            return None
        left, pause_ms, right = payload
        return self._build_ssml_with_break(left, pause_ms, right)

    def _build_ssml_with_break(self, left: str, break_ms: int, right: str) -> str:
        rate = f"{int(self._tts_rate * 100)}%"
        pitch = html.escape(self._tts_pitch, quote=True)
        voice = html.escape(self._tts_voice_name, quote=True)
        lang = html.escape(self._language, quote=True)
        inner = (
            f"{html.escape(left)}"
            f'<break time="{break_ms}ms"/>'
            f"{html.escape(right)}"
        )
        return (
            f"<speak version=\"1.0\" xml:lang=\"{lang}\">"
            f"<voice name=\"{voice}\">"
            f"<prosody rate=\"{rate}\" pitch=\"{pitch}\">"
            f"{inner}"
            f"</prosody>"
            f"</voice>"
            f"</speak>"
        )

    def _synthesize_piper(self, text: str) -> tuple[Optional[str], bool]:
        cache_text = self._cache_text(text)
        cache_key = self._cache_key(cache_text)
        cache_hit = False
        cache_path = None
        if self._cache_enabled:
            os.makedirs(self._cache_dir, exist_ok=True)
            cache_path = os.path.join(self._cache_dir, f"{cache_key}.wav")
            if os.path.isfile(cache_path):
                cache_hit = True
                return cache_path, cache_hit

        if not os.path.isfile(self._piper_cfg.binary):
            raise RuntimeError(f"Piper binary not found: {self._piper_cfg.binary}")
        if not os.path.isfile(self._piper_cfg.model):
            raise RuntimeError(f"Piper model not found: {self._piper_cfg.model}")

        output_path = cache_path or self._temp_audio_path(cache_key)
        temp_path = output_path + ".raw.wav" if self._needs_pitch_shift() else output_path

        if self._piper_process is not None:
            self.get_logger().debug("Sintetizando con modelo Piper ya cargado...")
            t_infer = time.monotonic()
            wav_bytes = self._piper_process.synthesize(cache_text)
            infer_ms = int((time.monotonic() - t_infer) * 1000)
            self.get_logger().info(f"TTS sintetizó en {infer_ms}ms (modelo ya cargado)")
            with open(temp_path, "wb") as _f:
                _f.write(wav_bytes)
        else:
            # Fallback: subprocess nuevo por síntesis (comportamiento original)
            length_scale = self._piper_cfg.length_scale
            if length_scale <= 0:
                length_scale = self._rate_to_length_scale(self._effective_rate())
            cmd = [
                self._piper_cfg.binary,
                "--model", self._piper_cfg.model,
                "--config", self._piper_cfg.config,
                "--output_file", temp_path,
                "--speaker", str(self._piper_cfg.speaker),
                "--noise_scale", str(self._piper_cfg.noise_scale),
                "--length_scale", str(length_scale),
                "--noise_w", str(self._piper_cfg.noise_w),
                "--sentence_silence", str(self._piper_cfg.sentence_silence),
                "--espeak_data", self._piper_cfg.espeak_data,
            ]
            env = os.environ.copy()
            env["LD_LIBRARY_PATH"] = os.path.dirname(self._piper_cfg.binary)
            subprocess.run(cmd, input=cache_text, text=True, check=True, env=env)

        if self._needs_pitch_shift() and shutil.which("sox"):
            cents = self._pitch_to_cents(self._effective_pitch())
            subprocess.run(
                ["sox", temp_path, output_path, "pitch", str(cents)],
                check=True,
            )
            if temp_path != output_path:
                try:
                    os.remove(temp_path)
                except OSError:
                    pass
        elif temp_path != output_path:
            os.replace(temp_path, output_path)
        elif self._needs_pitch_shift():
            self.get_logger().warning("Pitch shift requested but 'sox' is not available.")

        return output_path, cache_hit

    def _synthesize_elevenlabs(
        self, text: str
    ) -> tuple[Optional[str], bool]:
        """Sintetiza con ElevenLabs y devuelve path al WAV resultante.

        Si la API falla, lanza excepcion (el _process_synthesis loggea error).
        Reusa la cache existente del nodo.
        """
        if self._el_provider is None:
            raise RuntimeError("ElevenLabs provider no inicializado")

        cache_text = self._cache_text(text)
        cache_key = self._cache_key(cache_text)
        cache_hit = False
        cache_path = None
        if self._cache_enabled:
            import os
            os.makedirs(self._cache_dir, exist_ok=True)
            cache_path = os.path.join(self._cache_dir, f"{cache_key}.wav")
            if os.path.isfile(cache_path):
                cache_hit = True
                return cache_path, cache_hit

        # Cache miss: llamar a la API
        wav_bytes = self._el_provider.synthesize(cache_text)

        output_path = cache_path or self._temp_audio_path(cache_key)
        with open(output_path, "wb") as f:
            f.write(wav_bytes)
        return output_path, cache_hit

    def _synthesize_azure(
        self, text: str, ssml_text: Optional[str]
    ) -> tuple[Optional[str], bool]:
        cache_key = self._cache_key(text)
        cache_hit = False
        cache_path = None
        if self._cache_enabled:
            os.makedirs(self._cache_dir, exist_ok=True)
            cache_path = os.path.join(self._cache_dir, f"{cache_key}.wav")
            if os.path.isfile(cache_path):
                cache_hit = True
                return cache_path, cache_hit

        api_key = os.getenv(self._azure_key_env)
        region = os.getenv(self._azure_region_env)
        if not api_key or not region:
            self.get_logger().error(
                f"Azure TTS requiere credenciales en {self._azure_key_env} y {self._azure_region_env}"
            )
            return None, cache_hit

        if not ssml_text:
            ssml_text = build_ssml(
                text,
                voice_name=self._tts_voice_name,
                rate_pct=self._tts_rate,
                pitch=self._tts_pitch,
                short_ms=self._pause_short_ms,
                medium_ms=self._pause_medium_ms,
                language=self._language,
            )

        output_path = cache_path or self._temp_audio_path(cache_key)
        audio_data: Optional[bytes] = None
        if speechsdk is not None:
            with self._azure_lock:
                if self._azure_synthesizer is None:
                    speech_config = speechsdk.SpeechConfig(
                        subscription=api_key, region=region
                    )
                    speech_config.speech_synthesis_voice_name = self._tts_voice_name
                    output_format = self._resolve_output_format_enum()
                    if output_format is not None:
                        speech_config.set_speech_synthesis_output_format(output_format)
                    self._azure_synthesizer = speechsdk.SpeechSynthesizer(
                        speech_config=speech_config, audio_config=None
                    )
                synthesizer = self._azure_synthesizer
            result = synthesizer.speak_ssml_async(ssml_text).get()
            if result.reason != speechsdk.ResultReason.SynthesizingAudioCompleted:
                raise RuntimeError(
                    f"Azure TTS failed: {result.reason}"
                )
            audio_data = result.audio_data
        else:
            url = f"https://{region}.tts.speech.microsoft.com/cognitiveservices/v1"
            headers = {
                "Ocp-Apim-Subscription-Key": api_key,
                "Content-Type": "application/ssml+xml",
                "X-Microsoft-OutputFormat": self._tts_output_format,
                "User-Agent": "robertito-voice-synth",
            }
            request = urllib.request.Request(
                url,
                data=ssml_text.encode("utf-8"),
                headers=headers,
                method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=30) as response:
                    audio_data = response.read()
            except urllib.error.HTTPError as exc:
                error_text = exc.read().decode("utf-8", errors="ignore")
                raise RuntimeError(
                    f"Azure TTS HTTP {exc.code}: {error_text}"
                ) from exc
            except urllib.error.URLError as exc:
                raise RuntimeError(f"Azure TTS connection error: {exc}") from exc

        if audio_data:
            with open(output_path, "wb") as output_file:
                output_file.write(audio_data)

        return output_path, cache_hit

    def _resolve_output_format_enum(
        self,
    ) -> Optional["speechsdk.SpeechSynthesisOutputFormat"]:
        if speechsdk is None:
            return None
        if self._output_format_enum is not None:
            return self._output_format_enum
        try:
            self._output_format_enum = getattr(
                speechsdk.SpeechSynthesisOutputFormat, self._tts_output_format
            )
        except AttributeError:
            self.get_logger().warning(
                f"Formato de Azure desconocido '{self._tts_output_format}', usando Riff24Khz16BitMonoPcm."
            )
            self._output_format_enum = (
                speechsdk.SpeechSynthesisOutputFormat.Riff24Khz16BitMonoPcm
            )
        return self._output_format_enum

    def _synthesize_via_command(
        self, text: str, ssml_text: Optional[str]
    ) -> Optional[str]:
        if not self._tts_command:
            self.get_logger().warning("No TTS command configured for provider.")
            return None

        payload = ssml_text if ssml_text else text
        if "{text}" in self._tts_command or "{ssml}" in self._tts_command:
            command = self._tts_command.replace("{text}", text).replace(
                "{ssml}", ssml_text or text
            )
            cmd = command.split()
            subprocess.run(cmd, check=True)
            return None

        subprocess.run(self._tts_command.split() + [payload], check=True)
        return None

    def _play_audio(self, audio_path: str) -> None:
        if not audio_path:
            self.get_logger().warning("Sin ruta de audio para reproducir.")
            return

        if self._play_command:
            args = self._command_from_template(self._play_command, audio_path)
            if not args:
                self.get_logger().warning(
                    "play_command quedó vacío después de formatear el archivo WAV."
                )
            else:
                self._set_speaking_active(True)
                if self._run_external_command(args, audio_path, "play_command"):
                    self._set_speaking_active(False)
                    self._publish_health("ok")
                    return
                self._set_speaking_active(False)
                fallback_args = self._paplay_fallback_args(audio_path, args)
                if fallback_args:
                    self._set_speaking_active(True)
                    if self._run_external_command(
                        fallback_args, audio_path, "paplay fallback"
                    ):
                        self._set_speaking_active(False)
                        self._publish_health("ok")
                        return
                    self._set_speaking_active(False)

        backend_label = getattr(self._audio_player, "backend", "desconocido")
        self.get_logger().info(
            f"Reproduciendo '{audio_path}' mediante backend interno '{backend_label}'."
        )
        try:
            with open(audio_path, "rb") as audio_file:
                wav_bytes = audio_file.read()
            self._audio_player.play_wav_bytes(wav_bytes)
        except Exception as exc:
            self.get_logger().warning(f"Audio playback failed: {exc}")
            self._publish_health(f"error: reproducción falló ({exc})")

    @staticmethod
    def _command_from_template(template: str, audio_path: str) -> list[str]:
        formatted = template.replace("{wav}", audio_path)
        if not formatted.strip():
            return []
        return shlex.split(formatted)

    def _paplay_fallback_args(
        self, audio_path: str, previous_args: list[str]
    ) -> list[str]:
        paplay_path = shutil.which("paplay")
        if not paplay_path or self._is_paplay_command(previous_args):
            return []
        return [paplay_path, audio_path]

    @staticmethod
    def _is_paplay_command(args: list[str]) -> bool:
        if not args:
            return False
        return os.path.basename(args[0]) == "paplay"

    def _run_external_command(
        self, cmd_args: list[str], audio_path: str, label: str
    ) -> bool:
        command_display = shlex.join(cmd_args)
        self.get_logger().info(
            f"Reproduciendo '{audio_path}' usando comando externo '{command_display}' ({label})."
        )
        try:
            subprocess.run(cmd_args, check=True)
            self._publish_health("ok")
            return True
        except Exception as exc:
            self.get_logger().error(
                f"Reproducción externa ({label}) falló ({cmd_args[0]}): {exc}"
            )
            self._publish_health(f"error: audio externo falló ({exc})")
            return False

    def _cache_key(self, text: str) -> str:
        key = "|".join(
            [
                text,
                self._voice_mode,
                self._language,
                f"{self._effective_rate():.3f}",
                self._effective_pitch(),
                self._voice_id,
                self._tts_voice_name,
                self._tts_provider,
            ]
        )
        return hashlib.sha256(key.encode("utf-8")).hexdigest()

    def _cache_text(self, text: str) -> str:
        cleaned = (text or "").strip()
        if not cleaned:
            return ""

        normalized = self._normalize_text(cleaned).casefold()
        normalized = re.sub(r"\s+", " ", normalized).strip(" .,!?:;")
        aliased = _SHORT_REPLY_CACHE_ALIASES.get(normalized)
        if aliased:
            return aliased

        match = _TIME_REPLY_RE.match(cleaned)
        if match:
            prefix = "Es la" if match.group(1).casefold().startswith("es la") else "Son las"
            hour = int(match.group(2))
            minute = int(match.group(3))
            if 0 <= hour <= 23 and 0 <= minute <= 59:
                return f"{prefix} {hour:02d}:{minute:02d}."

        return cleaned

    @staticmethod
    def _rate_to_length_scale(rate: float) -> float:
        if rate <= 0:
            return 1.0
        scale = 1.0 / rate
        return max(0.5, min(2.0, scale))

    def _needs_pitch_shift(self) -> bool:
        return self._pitch_to_cents(self._effective_pitch()) != 0

    @staticmethod
    def _pitch_to_cents(pitch: str) -> int:
        if not pitch:
            return 0
        if pitch.endswith("st"):
            try:
                value = float(pitch[:-2])
                return int(value * 100)
            except ValueError:
                return 0
        try:
            return int(float(pitch))
        except ValueError:
            return 0

    def _temp_audio_path(self, seed: str) -> str:
        cache_root = self._cache_dir or os.path.join("/tmp", "robertito_voice")
        os.makedirs(cache_root, exist_ok=True)
        return os.path.join(cache_root, f"{seed}.wav")

    def _effective_rate(self) -> float:
        if self._voice_mode != "jarvis":
            return 1.0
        return self._rate

    def _effective_pitch(self) -> str:
        if self._voice_mode != "jarvis":
            return "0"
        return self._pitch

    def destroy_node(self) -> bool:
        if self._startup_timer is not None:
            timer = self._startup_timer
            self._startup_timer = None
            timer.cancel()
            self.destroy_timer(timer)
        self._set_speaking_active(False)
        self._audio_player.close()
        self._shutdown.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _on_playback_state_change(self, active: bool) -> None:
        self._set_speaking_active(active)

    def _on_audio_player_error(self, message: str) -> None:
        self._publish_health(f"error: {message}")

    def _set_speaking_active(self, active: bool) -> None:
        if self._speaking_active == active:
            return
        self._speaking_active = active
        self._speaking_pub.publish(Bool(data=active))

    def _publish_health(self, text: str) -> None:
        cleaned = text.strip() if text else "ok"
        self._health_state = cleaned or "ok"
        self._health_pub.publish(String(data=self._health_state))

    def _publish_health_heartbeat(self) -> None:
        self._health_pub.publish(String(data=self._health_state))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceSynthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
