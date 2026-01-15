import io
import json
import os
import queue
import random
import re
import threading
import time
import unicodedata
import wave
import xml.sax.saxutils as saxutils
from typing import Any, Dict, List, Optional, Tuple

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from robertito.audio_player import AudioFormat, AudioPlayer
from robertito.metrics import Metrics
from robertito.state_machine import AssistantState, AssistantStateMachine

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
]
_FOLLOW_OFF_TRIGGERS = [
    "dejar de seguir",
    "no sigas mas",
    "para de seguir",
]
_CLEAN_ON_TRIGGERS = ["activar modo limpieza rapida", "modo limpieza rapida"]
_CLEAN_OFF_TRIGGERS = ["desactivar modo limpieza rapida", "salir limpieza rapida"]
_SMALL_TALK_TRIGGERS = ["como estas", "todo bien", "que tal"]
_SMALL_TALK_RESPONSES = [
    "¡Todo bien! Listo para limpiar.",
    "De diez. ¿Qué hacemos?",
    "Excelente, atento.",
    "Todo tranquilo por acá.",
    "Impecable. Decime.",
]


class APIChatNode(Node):
    def __init__(self) -> None:
        super().__init__("robertito_api_chat")

        self._wake_topic = self.declare_parameter("wake_topic", "/wake_word/detected").value
        self._tts_topic = self.declare_parameter("tts_topic", "/assistant/say").value
        self._follow_mode_topic = self.declare_parameter(
            "follow_mode_topic", "/assistant/mode/follow_person"
        ).value
        self._clean_mode_topic = self.declare_parameter(
            "clean_mode_topic", "/assistant/mode/cleaning_quick"
        ).value
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
            "startup_message", "Buenos días"
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
        self._follow_mode_pub = (
            self.create_publisher(Bool, self._follow_mode_topic, 10)
            if self._follow_mode_topic
            else None
        )
        self._clean_mode_pub = (
            self.create_publisher(Bool, self._clean_mode_topic, 10)
            if self._clean_mode_topic
            else None
        )
        self.create_subscription(Bool, self._wake_topic, self._on_wake_event, 10)

        self._state_machine = AssistantStateMachine()
        self._metrics = Metrics()
        self._session_lock = threading.Lock()
        self._session_id = 0
        self._active_session = 0
        self._session_cancel_event = threading.Event()

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
        mic_index = self._parse_device_index(self._mic_device_index)
        self._audio_capture = AudioCapture(sample_rate=16000, frame_ms=30, device_index=mic_index)
        self._audio_capture.start()
        self._vad = VadSegmenter(sample_rate=16000, frame_ms=30, vad_mode=self._vad_aggressiveness)

        self._speech_recognizer: Optional[AzureSpeechRecognizer] = None
        if self._stt_mode != "whisper":
            try:
                self._speech_recognizer = AzureSpeechRecognizer(
                    self._azure_key_env, self._azure_region_env
                )
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
                self._speech_recognizer = None
        self._whisper: Optional[WhisperTranscriber] = None
        if whisper and np:
            self._whisper = WhisperTranscriber(model_name="tiny")
        self._speech_synthesizer: Optional[AzureTextToSpeech] = None
        self._ensure_tts()

        self._load_history_prompts()
        self._worker_thread.start()
        self._beep_chunk = self._load_beep()
        self._startup_timer: Optional[threading.Timer] = None
        self._schedule_startup_message()
        self.get_logger().info("APIChatNode listo con Azure STT/TTS y playback persistente.")

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
        self._state_machine.transition(AssistantState.SPEAKING)
        self._metrics.t_tts_first_chunk_ready = time.monotonic()
        self._speak_text(self._startup_message, session_id)
        self._state_machine.transition(AssistantState.IDLE)
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
        threading.Thread(
            target=self._listen_and_respond,
            args=(session_id,),
            name=f"listen-session-{session_id}",
            daemon=True,
        ).start()

    def _start_new_session(self) -> int:
        with self._session_lock:
            self._session_id += 1
            self._active_session = self._session_id
            self._session_cancel_event.set()
            self._session_cancel_event = threading.Event()
            self._metrics.reset()
            self._metrics.t_wake = time.monotonic()
            self._audio_player.reset()
            self._state_machine.transition(AssistantState.LISTENING)
            return self._session_id

    def _listen_and_respond(self, session_id: int) -> None:
        if session_id != self._active_session:
            return
        self._metrics.t_listen_start = time.monotonic()
        self._play_feedback_beep()
        utterance = self._capture_utterance(session_id)
        if not utterance:
            self._state_machine.transition(AssistantState.IDLE)
            return
        self._metrics.t_user_end = time.monotonic()
        transcript = self._run_stt(utterance)
        if not transcript:
            self._state_machine.transition(AssistantState.IDLE)
            return
        if session_id != self._active_session:
            return
        try:
            self._request_queue.put_nowait((transcript, session_id))
        except queue.Full:
            self.get_logger().warn("Cola de peticiones llena; descartando texto.")

    def _capture_utterance(self, session_id: int) -> Optional[bytes]:
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

    def _run_stt(self, pcm_audio: bytes) -> Optional[str]:
        start = time.monotonic()
        text: Optional[str] = None
        if self._stt_mode != "whisper" and self._speech_recognizer:
            text = self._speech_recognizer.recognize(pcm_audio)
        if (not text and self._stt_mode != "azure") or (not text and self._whisper):
            if self._whisper is None:
                self._whisper = WhisperTranscriber(model_name="tiny")
            text = self._whisper.transcribe(pcm_audio)
        if text:
            self._metrics.t_stt_done = time.monotonic()
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
        self._state_machine.transition(AssistantState.THINKING)
        weather_response = self._maybe_handle_weather(user_text)
        if weather_response:
            self._state_machine.transition(AssistantState.SPEAKING)
            self._metrics.t_tts_first_chunk_ready = time.monotonic()
            self._speak_text(weather_response, session_id)
            self._state_machine.transition(AssistantState.IDLE)
            self._log_metrics()
            return
        normalized = self._normalize_intent(user_text)
        if self._handle_follow_intent(normalized, session_id):
            return
        if self._handle_clean_intent(normalized, session_id):
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
                self._state_machine.transition(AssistantState.SPEAKING)
                first_chunk = False
            full_response += chunk
            self._speak_text(chunk, session_id)
        if first_chunk:
            self._metrics.t_tts_first_chunk_ready = time.monotonic()
            self._state_machine.transition(AssistantState.SPEAKING)
            full_response = self._fallback_text
            self._speak_text(self._fallback_text, session_id)
        self._history.append({"role": "assistant", "content": full_response})
        self._state_machine.transition(AssistantState.IDLE)
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
        result = self._speech_synthesizer.synthesize(ssml)
        if not result:
            return
        fmt, pcm = result
        if not self._metrics.t_audio_play_start:
            self._metrics.t_audio_play_start = time.monotonic()
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

    def _maybe_handle_weather(self, user_text: str) -> Optional[str]:
        if "temperatura" not in user_text.lower():
            return None
        lat = float(os.getenv("WEATHER_LAT", "0.0"))
        lon = float(os.getenv("WEATHER_LON", "0.0"))
        if lat == 0.0 and lon == 0.0:
            return "Necesito tu ubicación para decir la temperatura."
        params = {
            "latitude": lat,
            "longitude": lon,
            "current": "temperature_2m",
            "temperature_unit": os.getenv("WEATHER_UNIT", "celsius"),
        }
        try:
            response = requests.get("https://api.open-meteo.com/v1/forecast", params=params, timeout=6)
            response.raise_for_status()
            data = response.json()
            temp = data.get("current", {}).get("temperature_2m")
            units = data.get("current_units", {}).get("temperature_2m", "°C")
            if temp is None:
                return "No pude obtener la temperatura en este momento."
        except requests.RequestException:
            return "No pude consultar la temperatura ahora."
        location = os.getenv("WEATHER_LOCATION", "tu zona")
        return f"En {location} la temperatura es {temp}{units}."

    def _normalize_intent(self, text: str) -> str:
        normalized = unicodedata.normalize("NFD", text)
        stripped = "".join(
            ch for ch in normalized if not unicodedata.combining(ch)
        )
        filtered = "".join(
            (ch.lower() if ch.isalnum() or ch.isspace() else " ")
            for ch in stripped
        )
        return " ".join(filtered.split())

    def _publish_bool(self, publisher: Optional[Any], value: bool) -> None:
        if publisher is None:
            return
        publisher.publish(Bool(data=value))

    def _respond_with_voice(self, message: str, session_id: int) -> None:
        self._state_machine.transition(AssistantState.SPEAKING)
        self._metrics.t_tts_first_chunk_ready = time.monotonic()
        self._speak_text(message, session_id)
        self._state_machine.transition(AssistantState.IDLE)
        self._log_metrics()

    def _handle_follow_intent(self, normalized_text: str, session_id: int) -> bool:
        if any(trigger in normalized_text for trigger in _FOLLOW_ON_TRIGGERS):
            self._publish_bool(self._follow_mode_pub, True)
            self._respond_with_voice("Perfecto, activo seguimiento.", session_id)
            return True
        if any(trigger in normalized_text for trigger in _FOLLOW_OFF_TRIGGERS):
            self._publish_bool(self._follow_mode_pub, False)
            self._respond_with_voice("Listo, dejo de seguir.", session_id)
            return True
        return False

    def _handle_clean_intent(self, normalized_text: str, session_id: int) -> bool:
        if any(trigger in normalized_text for trigger in _CLEAN_ON_TRIGGERS):
            self._publish_bool(self._clean_mode_pub, True)
            self._respond_with_voice("Modo limpieza rápida activado.", session_id)
            return True
        if any(trigger in normalized_text for trigger in _CLEAN_OFF_TRIGGERS):
            self._publish_bool(self._clean_mode_pub, False)
            self._respond_with_voice(
                "Modo limpieza rápida desactivado.", session_id
            )
            return True
        return False

    def _handle_small_talk(self, normalized_text: str, session_id: int) -> bool:
        if any(trigger in normalized_text for trigger in _SMALL_TALK_TRIGGERS):
            response = random.choice(_SMALL_TALK_RESPONSES)
            self._respond_with_voice(response, session_id)
            return True
        return False

    def _log_metrics(self) -> None:
        self._metrics.log(self.get_logger(), round_end=time.monotonic())

    def destroy_node(self) -> bool:
        self._worker_stop.set()
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
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
