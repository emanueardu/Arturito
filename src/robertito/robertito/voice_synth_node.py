import hashlib
import html
import os
import queue
import re
import shutil
import subprocess
import threading
import time
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


_SOURCE_PREFIX_RE = re.compile(r"^\[([^\]]+)\]\s*(.*)")
_BAD_PHRASE = "ajuste adicional"
_CUSTOM_BREAK_SPEECH = {
    "Sí, ¿en qué puedo ayudarte?": ("Sí,", 150, " ¿en qué puedo ayudarte?"),
    "Buenos días, soy Robertito, estoy listo para funcionar": (
        "Buenos días,",
        200,
        " soy Robertito, estoy listo para funcionar",
    ),
}


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

        self._queue: "queue.Queue[str]" = queue.Queue()
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._startup_timer = None
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
        if self._startup_message:
            self._schedule_startup_message()

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
            self._queue.put_nowait(self._startup_message)
            self.get_logger().info("Startup message queued for speech.")
        except queue.Full:
            self.get_logger().warning("Speech queue full, dropping startup message.")

    @staticmethod
    def _resolve_play_command(play_command: str) -> str:
        if play_command:
            return play_command
        if shutil.which("paplay"):
            return "paplay"
        if shutil.which("aplay"):
            return "aplay"
        return ""

    def _build_audio_player(self) -> AudioPlayer:
        backend = self._audio_backend
        if backend in ("alsa", "aplay"):
            return AudioPlayer(backend="aplay", device_name=self._audio_device)
        if backend in ("pyaudio", "default"):
            return AudioPlayer(backend="pyaudio")
        return AudioPlayer(backend="aplay", device_name=self._audio_device)

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
            self._queue.put_nowait(text)
        except queue.Full:
            self.get_logger().warning("Voice queue full, dropping message.")

    def _worker_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                text = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._handle_text(text)
            finally:
                self._queue.task_done()

    def _handle_text(self, text: str) -> None:
        source, normalized_text = self._extract_source(text)
        if source:
            self.get_logger().info(f"Fuente TTS: {source}")
        filtered = self._filter_sensitive(normalized_text)
        if not filtered:
            self.get_logger().warning("Texto bloqueado por filtro de seguridad.")
            return
        formatted_text = self._format_text(filtered)
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
            self._play_audio(audio_path)
        except Exception as exc:
            self.get_logger().error(f"TTS error: {exc}")
            if ssml_text:
                self.get_logger().info("Retrying TTS without SSML.")
                try:
                    audio_path = self._synthesize_via_command(formatted_text, None)
                    if audio_path:
                        self._play_audio(audio_path)
                except Exception as retry_exc:
                    self.get_logger().error(f"Fallback TTS failed: {retry_exc}")

    def _format_text(self, text: str) -> str:
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
        cache_key = self._cache_key(text)
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

        length_scale = self._piper_cfg.length_scale
        if length_scale <= 0:
            length_scale = self._rate_to_length_scale(self._effective_rate())

        cmd = [
            self._piper_cfg.binary,
            "--model",
            self._piper_cfg.model,
            "--config",
            self._piper_cfg.config,
            "--output_file",
            temp_path,
            "--speaker",
            str(self._piper_cfg.speaker),
            "--noise_scale",
            str(self._piper_cfg.noise_scale),
            "--length_scale",
            str(length_scale),
            "--noise_w",
            str(self._piper_cfg.noise_w),
            "--sentence_silence",
            str(self._piper_cfg.sentence_silence),
            "--espeak_data",
            self._piper_cfg.espeak_data,
        ]

        env = os.environ.copy()
        piper_lib_dir = os.path.dirname(self._piper_cfg.binary)
        env["LD_LIBRARY_PATH"] = piper_lib_dir

        subprocess.run(cmd, input=text, text=True, check=True, env=env)

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
        try:
            with open(audio_path, "rb") as audio_file:
                wav_bytes = audio_file.read()
            self._audio_player.play_wav_bytes(wav_bytes)
        except Exception as exc:
            self.get_logger().warning(f"Audio playback failed: {exc}")

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
        self._audio_player.close()
        self._shutdown.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


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
