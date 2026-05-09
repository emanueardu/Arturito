import io
import json
import os
import queue
import random
import re
import threading
import time
import wave
import xml.sax.saxutils as saxutils
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional, Tuple
from zoneinfo import ZoneInfo

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String

from robertito.audio_player import AudioFormat, AudioPlayer
from robertito.family_companion import (
    build_family_weather_report,
    fetch_weather_snapshot,
    render_reminder_ack,
)
from robertito.metrics import Metrics, TurnTimer
from robertito.state_machine import AssistantState, AssistantStateMachine
from robertito.useful_memory import (
    UsefulMemoryStore,
    normalize_spanish,
    parse_memory_statement,
    parse_reminder_request,
)

try:
    import azure.cognitiveservices.speech as speechsdk  # type: ignore
except ImportError:  # pragma: no cover
    speechsdk = None  # type: ignore

try:
    import numpy as np  # type: ignore
except ImportError:  # pragma: no cover
    np = None  # type: ignore

try:  # optional dependency
    import webrtcvad  # type: ignore
except ImportError:  # pragma: no cover
    webrtcvad = None  # type: ignore

try:  # optional dependency
    import whisper  # type: ignore
except ImportError:  # pragma: no cover
    whisper = None  # type: ignore


def _lazy_import(module_name: str, friendly_name: str):
    try:
        return __import__(module_name)
    except ImportError as exc:  # pragma: no cover - dependency guidance
        raise RuntimeError(
            f"El módulo opcional '{friendly_name}' ({module_name}) es requerido. "
            "Instalalo con 'pip install {friendly_name}' y/o revisá la documentación."
        ) from exc


class AudioCapture:
    def __init__(self, sample_rate: int, frame_ms: int, device_index: Optional[int]) -> None:
        self.sample_rate = sample_rate
        self.frame_ms = frame_ms
        self.device_index = device_index
        self._queue: "queue.Queue[bytes]" = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._audio = None
        self._stream = None
        self._lock = threading.Lock()

    def start(self) -> None:
        try:
            pyaudio = _lazy_import("pyaudio", "pyaudio")
        except RuntimeError:
            return
        with self._lock:
            if self._stream is not None:
                return
            self._audio = pyaudio.PyAudio()
            frames_per_buffer = int(self.sample_rate * self.frame_ms / 1000)
            try:
                self._stream = self._audio.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=self.sample_rate,
                    input=True,
                    frames_per_buffer=frames_per_buffer,
                    input_device_index=self.device_index,
                )
            except Exception:
                self._audio.terminate()
                self._audio = None
                self._stream = None
                return
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        with self._lock:
            if self._stream is not None:
                try:
                    self._stream.stop_stream()
                    self._stream.close()
                except Exception:
                    pass
                self._stream = None
            if self._audio is not None:
                try:
                    self._audio.terminate()
                except Exception:
                    pass
                self._audio = None

    def read(self, timeout: float = 0.5) -> Optional[bytes]:
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                stream = self._stream
            if stream is None:
                break
            try:
                frame = stream.read(
                    int(self.sample_rate * self.frame_ms / 1000),
                    exception_on_overflow=False,
                )
            except Exception:
                time.sleep(0.01)
                continue
            try:
                self._queue.put(frame, timeout=0.1)
            except queue.Full:
                continue


class VadSegmenter:
    def __init__(self, sample_rate: int, frame_ms: int, vad_mode: int) -> None:
        self.sample_rate = sample_rate
        self.frame_ms = frame_ms
        self.vad_mode = vad_mode
        self._vad = webrtcvad.Vad(vad_mode) if webrtcvad else None

    def is_speech(self, frame: bytes) -> bool:
        if self._vad is None:
            return False
        return self._vad.is_speech(frame, self.sample_rate)

    def capture_utterance(
        self,
        capture: AudioCapture,
        stop_event: Optional[threading.Event],
        pre_roll_ms: int = 300,
        max_silence_ms: int = 900,
        max_total_ms: int = 10000,
    ) -> Optional[bytes]:
        if self._vad is None:
            return None
        frames: List[bytes] = []
        ring: List[bytes] = []
        pre_roll_frames = int(pre_roll_ms / self.frame_ms)
        max_silence_frames = int(max_silence_ms / self.frame_ms)
        max_total_frames = int(max_total_ms / self.frame_ms)
        speech_started = False
        silence_frames = 0
        total_frames = 0

        while total_frames < max_total_frames:
            if stop_event and stop_event.is_set():
                return None
            frame = capture.read(timeout=0.5)
            if frame is None:
                continue
            total_frames += 1
            if not speech_started:
                ring.append(frame)
                if len(ring) > pre_roll_frames:
                    ring.pop(0)
                if self.is_speech(frame):
                    speech_started = True
                    frames.extend(ring)
                    ring.clear()
            else:
                frames.append(frame)
                if self.is_speech(frame):
                    silence_frames = 0
                else:
                    silence_frames += 1
                    if silence_frames >= max_silence_frames:
                        break
        if not frames:
            return None
        return b"".join(frames)


class AzureSpeechRecognizer:
    def __init__(self, key_env: str, region_env: str, language: str = "es-AR") -> None:
        if speechsdk is None:
            raise RuntimeError("Azure Speech SDK no disponible para STT.")
        subscription = os.getenv(key_env)
        region = os.getenv(region_env)
        if not subscription or not region:
            raise RuntimeError(
                "Azure Speech STT requiere credenciales en las variables de entorno."
            )
        self._speech_config = speechsdk.SpeechConfig(subscription=subscription, region=region)
        self._speech_config.speech_recognition_language = language

    def recognize(self, pcm_audio: bytes) -> Optional[str]:
        audio_stream = speechsdk.audio.PushAudioInputStream()
        audio_stream.write(pcm_audio)
        audio_stream.close()
        audio_config = speechsdk.audio.AudioConfig(stream=audio_stream)
        recognizer = speechsdk.SpeechRecognizer(
            speech_config=self._speech_config,
            audio_config=audio_config,
            language=self._speech_config.speech_recognition_language,
        )
        result = recognizer.recognize_once_async().get()
        if result.reason == speechsdk.ResultReason.RecognizedSpeech:
            return result.text
        return None


class AzureTextToSpeech:
    # _OUTPUT_FORMATS initialized at runtime in __init__ to avoid referencing
    # speechsdk at import time when it's not available (prevents AttributeError).
    _OUTPUT_FORMATS: Dict[str, Any] = {}

    def __init__(
        self,
        key_env: str,
        region_env: str,
        voice_name: str,
        output_format: str = "riff-24khz-16bit-mono-pcm",
    ) -> None:
        if speechsdk is None:
            raise RuntimeError("Azure Speech SDK no disponible para TTS.")
        # populate formats now that speechsdk is available
        self._OUTPUT_FORMATS = {
            "riff-16khz-16bit-mono-pcm": speechsdk.SpeechSynthesisOutputFormat.Riff16Khz16BitMonoPcm,
            "riff-24khz-16bit-mono-pcm": speechsdk.SpeechSynthesisOutputFormat.Riff24Khz16BitMonoPcm,
            "riff-48khz-16bit-mono-pcm": speechsdk.SpeechSynthesisOutputFormat.Riff48Khz16BitMonoPcm,
            "raw-16khz-16bit-mono-pcm": speechsdk.SpeechSynthesisOutputFormat.Raw16Khz16BitMonoPcm,
        }
        subscription = os.getenv(key_env)
        region = os.getenv(region_env)
        if not subscription or not region:
            raise RuntimeError(
                "Azure Speech TTS requiere credenciales en las variables de entorno."
            )
        speech_config = speechsdk.SpeechConfig(subscription=subscription, region=region)
        speech_config.speech_synthesis_voice_name = voice_name
        speech_config.set_speech_synthesis_output_format(
            self._OUTPUT_FORMATS.get(
                output_format,
                speechsdk.SpeechSynthesisOutputFormat.Riff24Khz16BitMonoPcm,
            )
        )
        self._synthesizer = speechsdk.SpeechSynthesizer(
            speech_config=speech_config,
            audio_config=None,
        )

    def synthesize(self, ssml: str) -> Optional[Tuple[AudioFormat, bytes]]:
        result = self._synthesizer.speak_ssml_async(ssml).get()
        if result.reason != speechsdk.ResultReason.SynthesizingAudioCompleted:
            return None
        wav_bytes = result.audio_data
        if not wav_bytes:
            return None
        with wave.open(io.BytesIO(wav_bytes), "rb") as wav_file:
            fmt = AudioFormat(
                sample_rate=wav_file.getframerate(),
                channels=wav_file.getnchannels(),
                sample_width=wav_file.getsampwidth(),
            )
            pcm = wav_file.readframes(wav_file.getnframes())
        return fmt, pcm


class WhisperTranscriber:
    def __init__(self, model_name: str = "tiny") -> None:
        self._model_name = model_name
        self._model = None

    def transcribe(self, pcm_audio: bytes) -> Optional[str]:
        if whisper is None or np is None:
            return None
        if self._model is None:
            self._model = whisper.load_model(self._model_name)
        samples = np.frombuffer(pcm_audio, dtype=np.int16).astype(np.float32) / 32768.0
        result = self._model.transcribe(samples, language="es")
        return result.get("text")


_FOLLOW_ON_TRIGGERS = [
    "seguir persona",
    "seguir a la persona",
    "seguime",
    "seguir",
    "segui a la persona",
    "activar seguimiento",
    "modo seguimiento",
    "seguimiento de persona",
]
_FOLLOW_OFF_TRIGGERS = [
    "dejar de seguir",
    "deja de seguirme",
    "no sigas mas",
    "para de seguir",
    "para de seguirme",
    "desactivar seguimiento",
    "desactiva seguimiento",
    "no me sigas",
    "salir seguimiento",
    "salir de seguimiento",
]
_CLEAN_ON_TRIGGERS = [
    "activar modo limpieza rapida",
    "activar limpieza rapida",
    "modo limpieza rapida",
    "limpieza rapida",
    "iniciar limpieza",
    "empezar a limpiar",
    "empeza a limpiar",
    "ponete a limpiar",
    "limpia",
]
_CLEAN_OFF_TRIGGERS = [
    "desactivar modo limpieza rapida",
    "desactivar limpieza rapida",
    "desactivar limpieza",
    "desactiva limpieza",
    "desactiva la limpieza",
    "salir limpieza rapida",
    "salir de limpieza",
    "deja de limpiar",
    "dejá de limpiar",
    "para de limpiar",
    "pará de limpiar",
    "parar limpieza",
    "para limpieza",
    "terminar limpieza",
    "termina la limpieza",
    "termina de limpiar",
    "termina con la limpieza",
    "basta de limpiar",
    "basta limpieza",
    "no limpies mas",
    "no limpies más",
    "ya no limpies",
    "limpieza apagada",
    "apaga limpieza",
    "apaga la limpieza",
]
_DEFAULT_GROUND_MODE_ON_TRIGGERS = [
    "modo piso",
    "estoy en el piso",
    "estas en el piso",
    "ponete en el piso",
    "pasa a modo piso",
    "al piso",
    "en el suelo",
    "baje a robertito",
]
_DEFAULT_GROUND_MODE_OFF_TRIGGERS = [
    "modo mesa",
    "modo altura",
    "modo seguro",
    "estoy en altura",
    "ponete en modo seguro",
    "pasa a modo mesa",
    "en la repisa",
    "de vuelta arriba",
]
_DEFAULT_GROUND_MODE_ON_CONFIRMATION = "Listo, modo piso activo."
_DEFAULT_GROUND_MODE_OFF_CONFIRMATION = "Volví al modo altura."
_SMALL_TALK_TRIGGERS = ["como estas", "todo bien", "que tal"]
_SMALL_TALK_RESPONSES = [
    "¡Todo bien! Listo para limpiar.",
    "De diez. ¿Qué hacemos?",
    "Excelente, atento.",
    "Todo tranquilo por acá.",
    "Impecable. Decime.",
]
_WAKE_ACK_RESPONSES = [
    "Sí, te escucho.",
    "Decime.",
    "Acá estoy.",
    "Dale, contame.",
    "Sí, Emanuel.",
    "A ver, decime.",
    "Listo, te escucho.",
]
_MISHEARD_RESPONSES = [
    "No te escuché bien.",
    "No llegué a entenderte.",
    "Probá de nuevo.",
]
_TIME_TRIGGERS = [
    "que hora es",
    "decime la hora",
]
_WEATHER_TRIGGERS = [
    "como esta el clima",
    "que temperatura hace",
    "como esta el tiempo hoy",
    "como esta el tiempo",
    "como esta el clima",
]


class APIChatNode(Node):
    """Pipeline wake → STT → LLM → TTS de Robertito.

    Este nodo ya no controla expresiones de ojos ni movimiento de cuerpo
    directamente. Cuando un intent quiere mostrar una emoción o ejecutar
    un gesto, publica un JSON al topic ``/robertito/orchestrator_request``
    y el ``presence_orchestrator_node`` lo aplica respetando prioridades.
    """

    def __init__(self) -> None:
        super().__init__("robertito_api_chat")

        self._wake_topic = self.declare_parameter("wake_topic", "/wake_word/detected").value
        self._tts_topic = self.declare_parameter("tts_topic", "/assistant/say").value
        self._user_text_topic = self.declare_parameter(
            "user_text_topic", "/assistant/listen_text"
        ).value
        self._listening_timeout_topic = self.declare_parameter(
            "listening_timeout_topic", "/behavior/listening_timeout"
        ).value
        self._thinking_active_topic = self.declare_parameter(
            "thinking_active_topic", "/behavior/thinking_active"
        ).value
        self._follow_mode_topic = self.declare_parameter(
            "follow_mode_topic", "/assistant/mode/follow_person"
        ).value
        # Topic input del web bridge: si lo publicamos también desde acá,
        # control_bridge actualiza su estado interno y republica el status,
        # y uart_node deja pasar los cmd_vel_clean. Sin esto el modo limpieza
        # se activaba (clean_quick_node arrancaba) pero la nav no llegaba al motor.
        self._clean_enable_input_topic = self.declare_parameter(
            "clean_enable_input_topic", "/robot_web/clean_enable"
        ).value
        self._clean_mode_topic = self.declare_parameter(
            "clean_mode_topic", "/assistant/mode/cleaning_quick"
        ).value
        self._command_topic = self.declare_parameter("command_topic", "/tracker_control").value
        # Bus para pedir comportamientos al orchestrator (expresiones, gestos, etc.).
        # Antes había un publisher directo a robertito/eyes_expression; ahora todo
        # pasa por presence_orchestrator_node para evitar pisarse con otros nodos.
        self._orchestrator_request_topic = self.declare_parameter(
            "orchestrator_request_topic", "/robertito/orchestrator_request"
        ).value
        self._tilt_topic = self.declare_parameter("tilt_topic", "/head/tilt").value
        # Tilt al desactivar modos (limpieza/seguimiento). 25° matchea el
        # default que usa el orchestrator para detección de caras.
        self._mode_off_tilt_deg = float(
            self.declare_parameter("mode_off_tilt_deg", 25.0).value
        )
        self._enable_microphone = bool(
            self.declare_parameter("enable_microphone", False).value
        )
        self._stt_mode = (
            self.declare_parameter("stt_mode", os.getenv("STT_MODE", "azure")).value
        ).lower()
        self._voice_mode = (
            self.declare_parameter("voice_mode", os.getenv("VOICE_MODE", "jarvis")).value
        ).lower()
        self._break_ms = int(
            self.declare_parameter("break_ms", int(os.getenv("BREAK_MS", "200"))).value
        )
        self._min_chunk_chars = int(self.declare_parameter("min_chunk_chars", 80).value)
        self._vad_silence_ms = int(
            self.declare_parameter("vad_silence_ms", 800).value
        )
        self._vad_aggressiveness = int(
            self.declare_parameter("vad_aggressiveness", 2).value
        )
        self._audio_backend = (
            self.declare_parameter("audio_backend", os.getenv("AUDIO_BACKEND", "sounddevice")).value
        ).lower()
        self._audio_device = self.declare_parameter("audio_device", os.getenv("AUDIO_DEVICE", "default")).value
        self._mic_device_index = self.declare_parameter("microphone_device_index", None).value
        self._timezone_name = self.declare_parameter(
            "timezone", os.getenv("TZ", "America/Argentina/Buenos_Aires")
        ).value
        self._weather_enabled = bool(
            self.declare_parameter(
                "weather_enabled", os.getenv("WEATHER_ENABLED", "true").lower() == "true"
            ).value
        )
        self._weather_lat = float(
            self.declare_parameter("weather_lat", float(os.getenv("WEATHER_LAT", "-34.8270"))).value
        )
        self._weather_lon = float(
            self.declare_parameter("weather_lon", float(os.getenv("WEATHER_LON", "-58.3930"))).value
        )
        self._weather_location = self.declare_parameter(
            "weather_location", os.getenv("WEATHER_LOCATION", "Burzaco, Buenos Aires, Argentina")
        ).value
        self._weather_unit = self.declare_parameter(
            "weather_unit", os.getenv("WEATHER_UNIT", "celsius")
        ).value
        self._weather_cache_minutes = float(
            self.declare_parameter("weather_cache_minutes", 20.0).value
        )
        self._weather_timeout_sec = float(
            self.declare_parameter("weather_timeout_sec", 4.0).value
        )
        self._memory_store_path = self.declare_parameter(
            "memory_store_path", "~/.ros/robertito_memory_store.json"
        ).value
        self._reminder_file_path = self.declare_parameter(
            "reminder_file_path", "~/.ros/robertito_reminders.json"
        ).value
        self._daily_greeting_event_topic = self.declare_parameter(
            "daily_greeting_event_topic", "/behavior/daily_greeting_emitted"
        ).value
        self._reminder_check_interval_sec = float(
            self.declare_parameter("reminder_check_interval_sec", 10.0).value
        )
        self._due_reminder_repeat_interval_sec = float(
            self.declare_parameter("due_reminder_repeat_interval_sec", 0.0).value
        )
        self._max_due_announcements = int(
            self.declare_parameter("max_due_announcements", 1).value
        )
        self._max_reminders_to_list = int(
            self.declare_parameter("max_reminders_to_list", 3).value
        )
        self._allow_unscheduled_notes = bool(
            self.declare_parameter("allow_unscheduled_notes", True).value
        )
        self._default_morning_hour = int(
            self.declare_parameter("default_morning_hour", 9).value
        )
        self._default_afternoon_hour = int(
            self.declare_parameter("default_afternoon_hour", 15).value
        )
        self._default_night_hour = int(
            self.declare_parameter("default_night_hour", 20).value
        )
        self._user_name = str(self.declare_parameter("user_name", "Emanuel").value).strip()
        raw_due_templates = self.declare_parameter(
            "due_reminder_templates",
            [
                "{user_name}, te recuerdo que {text}.",
                "Te recuerdo {text}.",
                "Acordate de {text}.",
            ],
        ).value
        if not isinstance(raw_due_templates, (list, tuple)):
            raw_due_templates = [
                "{user_name}, te recuerdo que {text}.",
                "Te recuerdo {text}.",
                "Acordate de {text}.",
            ]
        self._due_reminder_templates = [
            str(item).strip() for item in raw_due_templates if str(item).strip()
        ] or ["Te recuerdo {text}."]
        raw_scheduled_ack_templates = self.declare_parameter(
            "scheduled_reminder_ack_templates",
            [
                "Dale, te lo recuerdo {spoken_when}.",
                "Listo, te aviso {spoken_when}.",
                "Dale, te hago acordar {spoken_when}.",
            ],
        ).value
        if not isinstance(raw_scheduled_ack_templates, (list, tuple)):
            raw_scheduled_ack_templates = ["Dale, te lo recuerdo {spoken_when}."]
        self._scheduled_reminder_ack_templates = [
            str(item).strip() for item in raw_scheduled_ack_templates if str(item).strip()
        ] or ["Dale, te lo recuerdo {spoken_when}."]
        raw_unscheduled_ack_templates = self.declare_parameter(
            "unscheduled_reminder_ack_templates",
            [
                "Dale, me lo guardo.",
                "Listo, me lo anoto.",
                "Bueno, me acuerdo de eso.",
            ],
        ).value
        if not isinstance(raw_unscheduled_ack_templates, (list, tuple)):
            raw_unscheduled_ack_templates = ["Dale, me lo guardo."]
        self._unscheduled_reminder_ack_templates = [
            str(item).strip() for item in raw_unscheduled_ack_templates if str(item).strip()
        ] or ["Dale, me lo guardo."]
        self._nod_down_deg = float(self.declare_parameter("wake_nod_down_deg", -12.0).value)
        self._nod_up_deg = float(self.declare_parameter("wake_nod_up_deg", 8.0).value)
        self._nod_step_sec = float(self.declare_parameter("wake_nod_step_sec", 0.30).value)
        self._azure_key_env = self.declare_parameter("azure_key_env", "AZURE_SPEECH_KEY").value
        self._azure_region_env = self.declare_parameter("azure_region_env", "AZURE_SPEECH_REGION").value
        self._azure_voice_env = self.declare_parameter("azure_voice_env", "AZURE_TTS_VOICE").value
        self._beep_path = self.declare_parameter(
            "beep_path",
            os.path.join(os.path.dirname(__file__), "../assets/beep.wav"),
        ).value
        self._model = self.declare_parameter("model", "gpt-4o-mini").value
        self._api_url = self.declare_parameter(
            "api_url", "https://api.openai.com/v1/chat/completions"
        ).value
        self._temperature = float(self.declare_parameter("temperature", 0.7).value)
        self._max_tokens = int(self.declare_parameter("max_tokens", 256).value)
        self._history_size = int(self.declare_parameter("history_messages", 6).value)
        self._fallback_text = self.declare_parameter(
            "fallback_text",
            "Disculpa, no pude procesarlo. ¿Podés repetirlo?",
        ).value
        self._external_tts = bool(
            self.declare_parameter("external_tts", True).value
        )
        self._startup_message = self.declare_parameter(
            "startup_message", ""
        ).value
        self._startup_delay = float(
            self.declare_parameter("startup_delay", 0.6).value
        )
        self._jarvis_rate = (
            self.declare_parameter("jarvis_rate", os.getenv("JARVIS_RATE", "98%")).value
        )
        self._jarvis_pitch = (
            self.declare_parameter("jarvis_pitch", os.getenv("JARVIS_PITCH", "-1st")).value
        )
        self._normal_rate = (
            self.declare_parameter("normal_rate", os.getenv("NORMAL_RATE", "90%")).value
        )
        self._normal_pitch = (
            self.declare_parameter("normal_pitch", os.getenv("NORMAL_PITCH", "-2st")).value
        )

        self._publisher = self.create_publisher(String, self._tts_topic, 10)
        self._stop_publisher = self.create_publisher(Bool, "/assistant/stop", 10)
        self._command_pub = (
            self.create_publisher(String, self._command_topic, 10)
            if self._command_topic
            else None
        )
        self._orchestrator_request_pub = (
            self.create_publisher(String, self._orchestrator_request_topic, 10)
            if self._orchestrator_request_topic
            else None
        )
        self._tilt_pub = (
            self.create_publisher(Float32, self._tilt_topic, 10)
            if self._tilt_topic
            else None
        )
        self._follow_mode_pub = (
            self.create_publisher(Bool, self._follow_mode_topic, 10)
            if self._follow_mode_topic
            else None
        )
        self._thinking_active_pub = (
            self.create_publisher(Bool, self._thinking_active_topic, 10)
            if self._thinking_active_topic
            else None
        )
        self._clean_enable_input_pub = (
            self.create_publisher(Bool, self._clean_enable_input_topic, 10)
            if self._clean_enable_input_topic
            else None
        )
        self._clean_mode_pub = (
            self.create_publisher(Bool, self._clean_mode_topic, 10)
            if self._clean_mode_topic
            else None
        )
        self.create_subscription(Bool, self._wake_topic, self._on_wake_event, 10)
        self.create_subscription(String, self._user_text_topic, self._on_user_text, 10)
        self.create_subscription(
            Bool, self._listening_timeout_topic, self._on_listening_timeout, 10
        )
        self.create_subscription(
            Bool, self._daily_greeting_event_topic, self._on_daily_greeting, 10
        )

        # Stack "living": publisher de ground_mode + endpoint de frases para
        # living_presence_node (request/response sobre topics).
        # QoS TRANSIENT_LOCAL para que expressive_motion (owner) y cualquier
        # late-joiner reciban el último valor al arrancar.
        _ground_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ground_mode_pub = self.create_publisher(
            Bool, "/robertito/ground_mode", _ground_qos,
        )
        self._phrase_response_pub = self.create_publisher(
            String, "/assistant/generate_phrase_response", 10
        )
        self.create_subscription(
            String, "/assistant/generate_phrase_request",
            self._on_phrase_request, 10,
        )

        # Triggers y confirmaciones de ground_mode: editables vía YAML.
        phrases_yaml_path = str(
            self.declare_parameter("phrases_yaml_path", "").value
        )
        (
            self._ground_mode_on_triggers,
            self._ground_mode_off_triggers,
            self._ground_mode_on_confirmation,
            self._ground_mode_off_confirmation,
        ) = self._load_ground_mode_config(phrases_yaml_path)

        self._state_machine = AssistantStateMachine()
        self._metrics = Metrics()
        self._turn_timer = TurnTimer()
        self._session_lock = threading.Lock()
        self._session_id = 0
        self._active_session = 0
        self._session_cancel_event = threading.Event()
        self._awaiting_followup = False
        self._last_wake_monotonic = 0.0
        self._reminder_lock = threading.Lock()
        self._weather_cache: Dict[str, Any] = {}
        self._tz = ZoneInfo(str(self._timezone_name or "America/Argentina/Buenos_Aires"))
        self._memory_store = UsefulMemoryStore(
            str(self._memory_store_path or self._reminder_file_path)
        )

        self._http_session = requests.Session()
        self._history: List[Dict[str, str]] = []
        self._request_queue: "queue.Queue[Tuple[str, int]]" = queue.Queue()
        self._worker_stop = threading.Event()
        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            name="api-chat-worker",
            daemon=True,
        )

        device_index = self._parse_device_index(self._audio_device)
        self._audio_player = AudioPlayer(
            backend=self._audio_backend,
            device_name=None if device_index is not None else self._audio_device,
            device_index=device_index,
        )
        self._audio_capture: Optional[AudioCapture] = None
        self._vad: Optional[VadSegmenter] = None
        if self._enable_microphone:
            mic_index = self._parse_device_index(self._mic_device_index)
            self._audio_capture = AudioCapture(
                sample_rate=16000, frame_ms=30, device_index=mic_index
            )
            self._audio_capture.start()
            self._vad = VadSegmenter(
                sample_rate=16000, frame_ms=30, vad_mode=self._vad_aggressiveness
            )

        self._speech_recognizer: Optional[AzureSpeechRecognizer] = None
        if self._enable_microphone and self._stt_mode != "whisper":
            try:
                self._speech_recognizer = AzureSpeechRecognizer(
                    self._azure_key_env, self._azure_region_env
                )
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
                self._speech_recognizer = None
        self._whisper: Optional[WhisperTranscriber] = None
        if self._enable_microphone and whisper and np:
            self._whisper = WhisperTranscriber(model_name="tiny")
        self._speech_synthesizer: Optional[AzureTextToSpeech] = None
        self._ensure_tts()

        self._load_history_prompts()
        self._worker_thread.start()
        self._beep_chunk = self._load_beep()
        self._startup_timer: Optional[threading.Timer] = None
        self._schedule_startup_message()
        self.get_logger().info(
            "APIChatNode listo: wake=%s user_text=%s mic_local=%s"
            % (self._wake_topic, self._user_text_topic, self._enable_microphone)
        )

    def _load_history_prompts(self) -> None:
        system_prompt = self.declare_parameter(
            "system_prompt",
            "Eres Robertito, un asistente amable que responde en español.",
        ).value
        jarvis_prompt = self.declare_parameter(
            "jarvis_system_prompt",
            "Modo cabina estilo Jarvis: frases cortas y proactivas en español.",
        ).value
        if system_prompt:
            self._history.append({"role": "system", "content": system_prompt})
        if self._voice_mode == "jarvis" and jarvis_prompt:
            self._history.append({"role": "system", "content": jarvis_prompt})

    def _schedule_startup_message(self) -> None:
        if not self._startup_message:
            return
        self._startup_timer = threading.Timer(
            self._startup_delay, self._speak_startup_message
        )
        self._startup_timer.daemon = True
        self._startup_timer.start()

    def _speak_startup_message(self) -> None:
        session_id = self._start_new_session()
        self._set_assistant_state(AssistantState.SPEAKING)
        self._metrics.t_tts_first_chunk_ready = time.monotonic()
        self._speak_text(self._startup_message, session_id)
        self._set_assistant_state(AssistantState.IDLE)
        self._log_metrics()

    def _ensure_tts(self) -> None:
        if self._speech_synthesizer is not None:
            return
        try:
            voice_name = os.getenv(self._azure_voice_env, "es-AR-TomasNeural")
            self._speech_synthesizer = AzureTextToSpeech(
                self._azure_key_env,
                self._azure_region_env,
                voice_name,
            )
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            self._speech_synthesizer = None

    def _load_beep(self) -> Optional[Tuple[AudioFormat, bytes]]:
        if not os.path.exists(self._beep_path):
            return None
        with wave.open(self._beep_path, "rb") as wav_file:
            fmt = AudioFormat(
                sample_rate=wav_file.getframerate(),
                channels=wav_file.getnchannels(),
                sample_width=wav_file.getsampwidth(),
            )
            pcm = wav_file.readframes(wav_file.getnframes())
        return fmt, pcm

    def _parse_device_index(self, value: Any) -> Optional[int]:
        if isinstance(value, int):
            return value
        if isinstance(value, str) and value.isdigit():
            return int(value)
        return None

    def _on_wake_event(self, msg: Bool) -> None:
        if not msg.data:
            return
        self._stop_publisher.publish(Bool(data=True))
        session_id = self._start_new_session()
        self._awaiting_followup = True
        self._last_wake_monotonic = time.monotonic()
        self._publish_external_tts(random.choice(_WAKE_ACK_RESPONSES))
        self._trigger_ack_nod()
        if self._enable_microphone:
            threading.Thread(
                target=self._listen_and_respond,
                args=(session_id,),
                name=f"listen-session-{session_id}",
                daemon=True,
            ).start()

    def _on_user_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        session_id = self._active_session
        if session_id <= 0:
            return
        if not self._awaiting_followup and (time.monotonic() - self._last_wake_monotonic) > 12.0:
            return
        self._awaiting_followup = False
        try:
            self._request_queue.put_nowait((text, session_id))
        except queue.Full:
            self.get_logger().warn("Cola de peticiones llena; descartando texto reconocido.")

    def _on_listening_timeout(self, msg: Bool) -> None:
        if not msg.data or not self._awaiting_followup:
            return
        session_id = self._active_session
        self._awaiting_followup = False
        if session_id > 0:
            self._respond_with_voice(random.choice(_MISHEARD_RESPONSES), session_id)

    def _on_daily_greeting(self, msg: Bool) -> None:
        if not msg.data:
            return
        today = datetime.now(self._tz).date().isoformat()
        with self._reminder_lock:
            self._memory_store.set_daily_context("last_greeting_date", today)

    def _start_new_session(self) -> int:
        with self._session_lock:
            self._session_id += 1
            self._active_session = self._session_id
            self._session_cancel_event.set()
            self._session_cancel_event = threading.Event()
            self._metrics.reset()
            self._metrics.t_wake = time.monotonic()
            self._turn_timer.reset()
            self._turn_timer.start("wake")
            self._turn_timer.stop("wake")
            self._audio_player.reset()
            self._set_assistant_state(AssistantState.LISTENING)
            return self._session_id

    def _listen_and_respond(self, session_id: int) -> None:
        if not self._enable_microphone:
            return
        if session_id != self._active_session:
            return
        self._metrics.t_listen_start = time.monotonic()
        self._play_feedback_beep()
        self._turn_timer.start("vad")
        utterance = self._capture_utterance(session_id)
        self._turn_timer.stop("vad")
        if not utterance:
            self._set_assistant_state(AssistantState.IDLE)
            return
        self._metrics.t_user_end = time.monotonic()
        transcript = self._run_stt(utterance)
        if not transcript:
            self._set_assistant_state(AssistantState.IDLE)
            return
        if session_id != self._active_session:
            return
        self._awaiting_followup = False
        try:
            self._request_queue.put_nowait((transcript, session_id))
        except queue.Full:
            self.get_logger().warn("Cola de peticiones llena; descartando texto.")

    def _capture_utterance(self, session_id: int) -> Optional[bytes]:
        if self._audio_capture is None or self._vad is None:
            return None
        cancel_event = self._session_cancel_event
        utterance = self._vad.capture_utterance(
            self._audio_capture,
            cancel_event,
            max_silence_ms=self._vad_silence_ms,
        )
        if cancel_event.is_set() or session_id != self._active_session:
            return None
        return utterance

    def _play_feedback_beep(self) -> None:
        if not self._beep_chunk:
            return
        fmt, pcm = self._beep_chunk
        self._audio_player.play_pcm(pcm, fmt)
        self._metrics.t_feedback_play = time.monotonic()

    def _trigger_ack_nod(self) -> None:
        if self._tilt_pub is None:
            return
        threading.Thread(target=self._run_ack_nod, name="wake-ack-nod", daemon=True).start()

    def _run_ack_nod(self) -> None:
        for angle in (self._nod_down_deg, self._nod_up_deg, 0.0):
            if self._tilt_pub is None:
                return
            self._tilt_pub.publish(Float32(data=float(angle)))
            time.sleep(max(0.05, self._nod_step_sec))

    def _run_stt(self, pcm_audio: bytes) -> Optional[str]:
        start = time.monotonic()
        self._turn_timer.start("stt")
        text: Optional[str] = None
        if self._stt_mode != "whisper" and self._speech_recognizer:
            text = self._speech_recognizer.recognize(pcm_audio)
        if (not text and self._stt_mode != "azure") or (not text and self._whisper):
            if self._whisper is None:
                self._whisper = WhisperTranscriber(model_name="tiny")
            text = self._whisper.transcribe(pcm_audio)
        if text:
            self._metrics.t_stt_done = time.monotonic()
            self._turn_timer.stop("stt")
            latency_ms = int((self._metrics.t_stt_done - start) * 1000)
            self.get_logger().info(f"STT latency_ms={latency_ms}")
        return text

    def _worker_loop(self) -> None:
        while not self._worker_stop.is_set():
            try:
                user_text, session_id = self._request_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._process_text(user_text, session_id)
            finally:
                self._request_queue.task_done()

    def _process_text(self, user_text: str, session_id: int) -> None:
        if session_id != self._active_session:
            return
        self._set_assistant_state(AssistantState.THINKING)
        normalized = normalize_spanish(user_text)
        time_response = self._maybe_handle_time(normalized)
        if time_response:
            self._respond_with_voice(time_response, session_id)
            return
        weather_response = self._maybe_handle_weather(normalized)
        if weather_response:
            self._respond_with_voice(weather_response, session_id)
            return
        if self._handle_memory_query_intent(normalized, session_id):
            return
        if self._handle_reminder_intent(user_text, normalized, session_id):
            return
        if self._handle_memory_store_intent(user_text, normalized, session_id):
            return
        if self._handle_follow_intent(normalized, session_id):
            return
        if self._handle_clean_intent(normalized, session_id):
            return
        if self._handle_ground_mode_intent(normalized, session_id):
            return
        if self._handle_robot_function_intent(normalized, session_id):
            return
        if self._handle_small_talk(normalized, session_id):
            return
        self._history.append({"role": "user", "content": user_text})
        payload_messages = self._history[-self._history_size :]
        first_chunk = True
        full_response = ""
        for chunk in self._stream_openai(payload_messages, session_id):
            if session_id != self._active_session:
                return
            if first_chunk:
                self._metrics.t_tts_first_chunk_ready = time.monotonic()
                self._set_assistant_state(AssistantState.SPEAKING)
                first_chunk = False
            full_response += chunk
            self._speak_text(chunk, session_id)
        if first_chunk:
            self._metrics.t_tts_first_chunk_ready = time.monotonic()
            self._set_assistant_state(AssistantState.SPEAKING)
            full_response = self._fallback_text
            self._speak_text(self._fallback_text, session_id)
        self._history.append({"role": "assistant", "content": full_response})
        self._set_assistant_state(AssistantState.IDLE)
        self._log_metrics()

    def _speak_text(self, text: str, session_id: int) -> None:
        if session_id != self._active_session:
            return
        if self._external_tts:
            self._publish_external_tts(text)
            return
        if not self._speech_synthesizer:
            return
        ssml = self._build_ssml(text)
        _first_tts = not self._metrics.t_audio_play_start
        if _first_tts:
            self._turn_timer.start("tts")
        result = self._speech_synthesizer.synthesize(ssml)
        if not result:
            return
        fmt, pcm = result
        if not self._metrics.t_audio_play_start:
            self._metrics.t_audio_play_start = time.monotonic()
            self._turn_timer.stop("tts")
            self._turn_timer.start("play")
            self._turn_timer.stop("play")
        self._audio_player.play_pcm(pcm, fmt)

    def _publish_external_tts(self, text: str) -> None:
        payload = f"[api_chat_node] {text.strip()}"
        msg = String()
        msg.data = payload
        self._publisher.publish(msg)

    def _build_ssml(self, text: str) -> str:
        escaped = saxutils.escape(text)
        rate = self._jarvis_rate if self._voice_mode == "jarvis" else self._normal_rate
        pitch = self._jarvis_pitch if self._voice_mode == "jarvis" else self._normal_pitch
        ssml = [
            "<speak xml:lang=\"es-AR\">",
            "<voice name=\"es-AR-TomasNeural\">",
            f"<prosody rate=\"{rate}\" pitch=\"{pitch}\">{escaped}</prosody>",
            "</voice>",
        ]
        if self._voice_mode == "jarvis":
            ssml.append(f"<break time=\"{self._break_ms}ms\"/>")
        ssml.append("</speak>")
        return "".join(ssml)

    def _stream_openai(
        self, payload_messages: List[Dict[str, str]], session_id: int
    ):
        headers = {"Content-Type": "application/json"}
        api_key = os.getenv("OPENAI_API_KEY")
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"
        payload = {
            "model": self._model,
            "messages": payload_messages,
            "temperature": self._temperature,
            "stream": True,
        }
        if self._max_tokens > 0:
            payload["max_tokens"] = self._max_tokens
        try:
            self._turn_timer.start("llm")
            response = self._http_session.post(
                self._api_url,
                headers=headers,
                json=payload,
                timeout=30,
                stream=True,
            )
            response.raise_for_status()
        except requests.RequestException as exc:
            self.get_logger().error(f"Error de red consultando OpenAI: {exc}")
            return
        buffer = ""
        first_token = True
        full_response = ""
        for line in response.iter_lines():
            if session_id != self._active_session:
                return
            if not line:
                continue
            if line.startswith(b"data: "):
                payload = line[6:]
                if payload == b"[DONE]":
                    break
                try:
                    data = json.loads(payload.decode("utf-8"))
                    delta = data["choices"][0]["delta"].get("content", "")
                except Exception:
                    continue
                if delta:
                    if first_token:
                        self._metrics.t_llm_first_token = time.monotonic()
                        self._turn_timer.stop("llm")
                        first_token = False
                    buffer += delta
                    sentences, buffer = self._split_chunks(buffer)
                    for sentence in sentences:
                        full_response += sentence
                        yield sentence
        if buffer.strip():
            yield buffer.strip()
            full_response += buffer.strip()
        return

    def _split_chunks(self, text: str) -> Tuple[List[str], str]:
        sentences: List[str] = []
        while len(text) >= self._min_chunk_chars:
            match = re.search(r"([.!?;:])\s", text)
            if not match:
                break
            end = match.end()
            chunk = text[:end].strip()
            sentences.append(chunk)
            text = text[end:]
        if not sentences and len(text) >= self._min_chunk_chars:
            sentences.append(text.strip())
            text = ""
        return sentences, text

    def _maybe_handle_time(self, normalized_text: str) -> Optional[str]:
        if not any(trigger in normalized_text for trigger in _TIME_TRIGGERS):
            return None
        now = datetime.now(self._tz)
        return f"Son las {now.hour:02d}:{now.minute:02d}."

    def _maybe_handle_weather(self, normalized_text: str) -> Optional[str]:
        if not any(trigger in normalized_text for trigger in _WEATHER_TRIGGERS):
            return None
        if not self._weather_enabled and self._weather_lat == 0.0 and self._weather_lon == 0.0:
            return "Necesito tu ubicación para decirte el clima."
        cached = self._get_cached_weather()
        if cached is None:
            cached = self._fetch_weather()
            if cached is None:
                return "No pude consultar el clima ahora."
        return build_family_weather_report(cached, location=self._weather_location or "tu zona")

    def _get_cached_weather(self) -> Optional[Dict[str, Any]]:
        fetched_at = self._weather_cache.get("fetched_at")
        if not fetched_at:
            return None
        if (time.time() - float(fetched_at)) > (self._weather_cache_minutes * 60.0):
            return None
        return self._weather_cache

    def _fetch_weather(self) -> Optional[Dict[str, Any]]:
        snapshot = fetch_weather_snapshot(
            latitude=self._weather_lat,
            longitude=self._weather_lon,
            temperature_unit=self._weather_unit,
            timeout_sec=self._weather_timeout_sec,
            timezone_name="auto",
        )
        if snapshot is None:
            return None
        snapshot["fetched_at"] = time.time()
        self._weather_cache = snapshot
        return self._weather_cache

    @staticmethod
    def _weather_description(code: Optional[int]) -> Optional[str]:
        mapping = {
            0: "despejado",
            1: "mayormente despejado",
            2: "parcialmente nublado",
            3: "nublado",
            45: "con niebla",
            48: "con niebla",
            51: "con llovizna leve",
            53: "con llovizna",
            55: "con llovizna intensa",
            61: "con lluvia leve",
            63: "con lluvia",
            65: "con lluvia intensa",
            80: "con chaparrones leves",
            81: "con chaparrones",
            82: "con chaparrones intensos",
            95: "con tormenta",
        }
        return mapping.get(code)

    def _publish_bool(self, publisher: Optional[Any], value: bool) -> None:
        if publisher is None:
            return
        publisher.publish(Bool(data=value))

    def _set_assistant_state(self, new_state: AssistantState) -> None:
        previous = self._state_machine.transition(new_state)
        if previous is new_state:
            return
        if self._thinking_active_pub is not None:
            self._thinking_active_pub.publish(
                Bool(data=(new_state is AssistantState.THINKING))
            )

    def _publish_command(self, text: str) -> None:
        if self._command_pub is None or not text:
            return
        self._command_pub.publish(String(data=text))

    def _publish_expression(self, expression: str, duration_s: float = 0.0) -> None:
        """Pide al orchestrator que muestre una expresión.

        Antes publicábamos directo a ``robertito/eyes_expression``; ahora
        enviamos un JSON a ``/robertito/orchestrator_request`` para que el
        orchestrator decida cómo aplicarlo (respeta prioridades de triggers
        y no se pisa con random behaviors).
        """
        if self._orchestrator_request_pub is None or not expression:
            return
        payload = {"action": "set_expression", "value": expression}
        if duration_s and duration_s > 0:
            payload["duration_s"] = float(duration_s)
        self._orchestrator_request_pub.publish(String(data=json.dumps(payload)))

    def _respond_with_voice(self, message: str, session_id: int) -> None:
        self._set_assistant_state(AssistantState.SPEAKING)
        self._metrics.t_tts_first_chunk_ready = time.monotonic()
        self._speak_text(message, session_id)
        self._set_assistant_state(AssistantState.IDLE)
        self._log_metrics()

    def _publish_tilt(self, deg: float) -> None:
        if self._tilt_pub is not None:
            self._tilt_pub.publish(Float32(data=float(deg)))

    def _handle_follow_intent(self, normalized_text: str, session_id: int) -> bool:
        if any(trigger in normalized_text for trigger in _FOLLOW_ON_TRIGGERS):
            # Tilt a 30° antes de activar para que la cámara apunte
            # a la altura de la cara (sino el tracker no encuentra a la persona).
            self._publish_tilt(30.0)
            self._publish_bool(self._follow_mode_pub, True)
            self._respond_with_voice("Perfecto, activo seguimiento.", session_id)
            return True
        if any(trigger in normalized_text for trigger in _FOLLOW_OFF_TRIGGERS):
            self._publish_bool(self._follow_mode_pub, False)
            # Tilt vuelve a posición de búsqueda de cara (default del orchestrator).
            self._publish_tilt(self._mode_off_tilt_deg)
            self._respond_with_voice("Listo, dejo de seguir.", session_id)
            return True
        return False

    def _handle_clean_intent(self, normalized_text: str, session_id: int) -> bool:
        if any(trigger in normalized_text for trigger in _CLEAN_ON_TRIGGERS):
            # 1) Tilt 60° hacia arriba (cámara/ultrasonido bien arriba para
            #    evitar chocar con superficies bajas durante la limpieza).
            self._publish_tilt(60.0)
            # 2) Activar la lógica de navegación (clean_quick_node escucha).
            self._publish_bool(self._clean_mode_pub, True)
            # 3) Sincronizar con el control_bridge / web UI: actualiza el
            #    estado de limpieza y dispara que uart_node deje pasar los
            #    cmd_vel_clean (sin esto la nav no llegaba al motor).
            self._publish_bool(self._clean_enable_input_pub, True)
            self._respond_with_voice("Modo limpieza rápida activado.", session_id)
            return True
        if any(trigger in normalized_text for trigger in _CLEAN_OFF_TRIGGERS):
            self._publish_bool(self._clean_mode_pub, False)
            self._publish_bool(self._clean_enable_input_pub, False)
            self._publish_tilt(self._mode_off_tilt_deg)
            self._respond_with_voice(
                "Modo limpieza rápida desactivado.", session_id
            )
            return True
        return False

    def _handle_robot_function_intent(self, normalized_text: str, session_id: int) -> bool:
        if "modo atento" in normalized_text:
            self._publish_expression("focus")
            self._respond_with_voice("Listo, me pongo atento.", session_id)
            return True
        if "modo reposo" in normalized_text:
            self._publish_bool(self._follow_mode_pub, False)
            self._publish_bool(self._clean_mode_pub, False)
            self._publish_expression("sleepy")
            self._respond_with_voice("Entendido, paso a reposo.", session_id)
            return True
        if "saluda" in normalized_text or "saludar" in normalized_text:
            self._publish_command("saludar")
            self._respond_with_voice("Dale.", session_id)
            return True
        if "ponete feliz" in normalized_text or "ponete contento" in normalized_text:
            self._publish_expression("happy")
            self._respond_with_voice("Ahí va.", session_id)
            return True
        return False

    def _handle_reminder_intent(
        self, original_text: str, normalized_text: str, session_id: int
    ) -> bool:
        parsed = parse_reminder_request(
            original_text,
            normalized_text,
            now=datetime.now(self._tz),
            tz=self._tz,
            allow_unscheduled_notes=self._allow_unscheduled_notes,
            default_morning_hour=self._default_morning_hour,
            default_afternoon_hour=self._default_afternoon_hour,
            default_night_hour=self._default_night_hour,
        )
        if parsed is None:
            return False
        if parsed["kind"] == "clarify":
            self._respond_with_voice(str(parsed["question"]), session_id)
            return True
        with self._reminder_lock:
            reminder = self._memory_store.add_reminder(
                text=str(parsed["text"]),
                due_at=parsed.get("due_at"),
                kind=str(parsed["kind"]),
            )
        ack = render_reminder_ack(
            kind=str(parsed["kind"]),
            spoken_when=str(parsed.get("spoken_when") or ""),
            scheduled_templates=self._scheduled_reminder_ack_templates,
            unscheduled_templates=self._unscheduled_reminder_ack_templates,
        )
        self._respond_with_voice(ack, session_id)
        return True

    def _handle_memory_store_intent(
        self, original_text: str, normalized_text: str, session_id: int
    ) -> bool:
        memory = parse_memory_statement(original_text, normalized_text)
        if memory is None:
            return False
        with self._reminder_lock:
            self._memory_store.add_memory(
                text=str(memory["text"]),
                category=str(memory["category"]),
            )
        self._respond_with_voice("Dale, me lo guardo.", session_id)
        return True

    def _handle_memory_query_intent(self, normalized_text: str, session_id: int) -> bool:
        if any(
            phrase in normalized_text
            for phrase in (
                "que tengo pendiente hoy",
                "que me tengo que acordar hoy",
                "que tengo hoy",
            )
        ):
            self._respond_with_voice(self._build_today_summary(), session_id)
            return True
        if any(
            phrase in normalized_text
            for phrase in (
                "tengo recordatorios",
                "que recordatorios tengo",
                "que me tenias que recordar",
                "que me tenes que recordar",
            )
        ):
            self._respond_with_voice(self._build_reminder_summary(), session_id)
            return True
        if any(
            phrase in normalized_text
            for phrase in (
                "que recordas",
                "que te pedi que recuerdes",
                "que te dije que recuerdes",
            )
        ):
            self._respond_with_voice(self._build_memory_summary(), session_id)
            return True
        return False

    def _build_today_summary(self) -> str:
        with self._reminder_lock:
            today_items = self._memory_store.today_pending(now=datetime.now(self._tz))
        if not today_items:
            return "No tengo nada pendiente para hoy."
        limited = today_items[: max(1, self._max_reminders_to_list)]
        parts = [self._format_scheduled_reminder(reminder) for reminder in limited]
        if len(today_items) == 1:
            return f"Hoy tenés {parts[0]}."
        if len(today_items) <= self._max_reminders_to_list:
            return f"Hoy tenés {len(today_items)} recordatorios: " + "; ".join(parts) + "."
        remaining = len(today_items) - len(limited)
        return (
            f"Hoy tenés {len(today_items)} recordatorios. "
            + "; ".join(parts)
            + f"; y {remaining} más."
        )

    def _build_reminder_summary(self) -> str:
        now = datetime.now(self._tz)
        with self._reminder_lock:
            scheduled = self._memory_store.upcoming_pending(
                now=now, limit=max(1, self._max_reminders_to_list)
            )
            unscheduled = self._memory_store.unscheduled_reminders(
                limit=max(1, self._max_reminders_to_list)
            )
        if not scheduled and not unscheduled:
            return "No tengo recordatorios pendientes por ahora."
        parts: list[str] = []
        if scheduled:
            parts.append("; ".join(self._format_scheduled_reminder(item) for item in scheduled))
        if unscheduled:
            texts = ", ".join(str(item.get("text", "")).strip() for item in unscheduled)
            parts.append(f"sin horario: {texts}")
        return "Tengo " + " | ".join(parts) + "."

    def _build_memory_summary(self) -> str:
        with self._reminder_lock:
            memories = self._memory_store.recent_memories(
                limit=max(1, self._max_reminders_to_list)
            )
        if not memories:
            return "No tengo notas guardadas todavía."
        parts = [str(item.get("text", "")).strip() for item in memories if item.get("text")]
        if not parts:
            return "No tengo notas guardadas todavía."
        return "Me acuerdo de esto: " + "; ".join(parts) + "."

    def _format_scheduled_reminder(self, reminder: Dict[str, Any]) -> str:
        text = str(reminder.get("text", "")).strip()
        due_at_raw = reminder.get("due_at")
        if not due_at_raw:
            return text
        try:
            due_at = datetime.fromisoformat(str(due_at_raw)).astimezone(self._tz)
        except ValueError:
            return text
        now = datetime.now(self._tz)
        if due_at.date() == now.date():
            return f"{text} a las {due_at.hour:02d}:{due_at.minute:02d}"
        tomorrow = now.date() + timedelta(days=1)
        if due_at.date() == tomorrow:
            return f"{text} mañana a las {due_at.hour:02d}:{due_at.minute:02d}"
        return f"{text} el {due_at.day:02d}/{due_at.month:02d} a las {due_at.hour:02d}:{due_at.minute:02d}"

    def _handle_small_talk(self, normalized_text: str, session_id: int) -> bool:
        if any(trigger in normalized_text for trigger in _SMALL_TALK_TRIGGERS):
            response = random.choice(_SMALL_TALK_RESPONSES)
            self._respond_with_voice(response, session_id)
            return True
        return False

    # ---------------------------------------------------------------- living stack
    def _load_ground_mode_config(
        self, yaml_path: str
    ) -> Tuple[List[str], List[str], str, str]:
        """Carga triggers y confirmaciones de ground_mode desde un flat YAML.

        Si el archivo no existe o no tiene la sección `ground_mode`, retorna
        los defaults hardcodeados. No rompe: cualquier error cae al fallback.
        """
        on_triggers = [t.lower().strip() for t in _DEFAULT_GROUND_MODE_ON_TRIGGERS]
        off_triggers = [t.lower().strip() for t in _DEFAULT_GROUND_MODE_OFF_TRIGGERS]
        on_confirm = _DEFAULT_GROUND_MODE_ON_CONFIRMATION
        off_confirm = _DEFAULT_GROUND_MODE_OFF_CONFIRMATION
        if not yaml_path or not os.path.isfile(yaml_path):
            self.get_logger().info(
                "ground_mode: YAML no disponible, usando triggers default."
            )
            return on_triggers, off_triggers, on_confirm, off_confirm
        try:
            import yaml  # dependencia ya instalada en el python del ROS
            with open(yaml_path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            section = data.get("ground_mode") if isinstance(data, dict) else None
            if isinstance(section, dict):
                raw_on = section.get("on_triggers")
                raw_off = section.get("off_triggers")
                if isinstance(raw_on, list) and raw_on:
                    on_triggers = [str(x).lower().strip() for x in raw_on if x]
                if isinstance(raw_off, list) and raw_off:
                    off_triggers = [str(x).lower().strip() for x in raw_off if x]
                on_confirm = str(section.get("on_confirmation") or on_confirm)
                off_confirm = str(section.get("off_confirmation") or off_confirm)
                self.get_logger().info(
                    f"ground_mode: triggers cargados desde YAML "
                    f"({len(on_triggers)} on / {len(off_triggers)} off)"
                )
        except Exception as exc:
            self.get_logger().warn(
                f"ground_mode: error cargando YAML '{yaml_path}': {exc}. "
                "Usando defaults."
            )
        return on_triggers, off_triggers, on_confirm, off_confirm

    def _handle_ground_mode_intent(self, normalized_text: str, session_id: int) -> bool:
        text_norm = (normalized_text or "").lower().strip()
        if not text_norm:
            return False
        if any(trigger in text_norm for trigger in self._ground_mode_on_triggers):
            self._ground_mode_pub.publish(Bool(data=True))
            self._respond_with_voice(self._ground_mode_on_confirmation, session_id)
            return True
        if any(trigger in text_norm for trigger in self._ground_mode_off_triggers):
            self._ground_mode_pub.publish(Bool(data=False))
            self._respond_with_voice(self._ground_mode_off_confirmation, session_id)
            return True
        return False

    def _on_phrase_request(self, msg: String) -> None:
        context = (msg.data or "").strip()
        if not context:
            return
        # La llamada al LLM puede tardar ~1-2s; no bloqueamos el executor.
        threading.Thread(
            target=self._generate_phrase, args=(context,),
            name="phrase-gen", daemon=True,
        ).start()

    def _generate_phrase(self, context: str) -> None:
        phrase = ""
        try:
            system_prompt = (
                "Sos Robertito, un robot cariñoso. Devolvé UNA sola frase corta "
                "(máximo 15 palabras) en español rioplatense para saludar en "
                "este contexto. No uses comillas. No expliques nada. "
                "Devolvé solo la frase."
            )
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": context},
            ]
            headers = {"Content-Type": "application/json"}
            api_key = os.getenv("OPENAI_API_KEY")
            if api_key:
                headers["Authorization"] = f"Bearer {api_key}"
            payload = {
                "model": self._model,
                "messages": messages,
                "temperature": 0.9,
                "max_tokens": 40,
                "stream": False,
            }
            resp = self._http_session.post(
                self._api_url, headers=headers, json=payload, timeout=5,
            )
            resp.raise_for_status()
            data = resp.json()
            content = data["choices"][0]["message"].get("content") or ""
            phrase = self._clean_phrase(content)
        except Exception as exc:
            self.get_logger().warn(f"generate_phrase falló: {exc}")
            phrase = ""
        self._phrase_response_pub.publish(String(data=phrase))

    @staticmethod
    def _clean_phrase(text: str) -> str:
        cleaned = (text or "").strip()
        if len(cleaned) >= 2 and cleaned[0] == cleaned[-1] and cleaned[0] in ('"', "'"):
            cleaned = cleaned[1:-1].strip()
        words = cleaned.split()
        if len(words) > 15:
            cleaned = " ".join(words[:15])
        return cleaned

    def _log_metrics(self) -> None:
        self._metrics.log(self.get_logger(), round_end=time.monotonic())
        self.get_logger().info(self._turn_timer.summary())

    def destroy_node(self) -> bool:
        self._worker_stop.set()
        if self._thinking_active_pub is not None:
            self._thinking_active_pub.publish(Bool(data=False))
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        if self._audio_capture is not None:
            self._audio_capture.stop()
        self._audio_player.close()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = APIChatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
