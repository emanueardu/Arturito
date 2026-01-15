import json
import os
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _lazy_import(module_name: str, friendly_name: str):
    """Import helper that raises a descriptive RuntimeError when missing."""
    try:
        return __import__(module_name)
    except ImportError as exc:  # pragma: no cover - dependency guidance
        raise RuntimeError(
            f"El módulo opcional '{friendly_name}' ({module_name}) es requerido. "
            f"Instalalo con 'pip install {friendly_name}' y/o revisá la documentación."
        ) from exc


@dataclass
class WakeWordConfig:
    sample_rate: int = 16000
    block_size: int = 4096
    wake_word: str = "robertito"
    command_window_sec: float = 5.0
    command_silence_sec: float = 1.8
    vosk_model_path: str = os.path.expanduser("~/models/vosk-model-small-es-0.42")
    api_base: str = os.getenv("ROBERTITO_CODEX_API_BASE", "https://api.openai.com/v1/responses")
    api_model: str = os.getenv("ROBERTITO_CODEX_MODEL", "gpt-4o-mini")
    api_key_env: str = "OPENAI_API_KEY"


class WakeWordAssistantNode(Node):
    """Escucha el micrófono, detecta la palabra clave y consulta la API de Codex."""

    def __init__(self) -> None:
        super().__init__("wake_word_assistant")

        self.cfg = WakeWordConfig(
            sample_rate=int(self.declare_parameter("sample_rate", 16000).value),
            block_size=int(self.declare_parameter("block_size", 4096).value),
            wake_word=self.declare_parameter("wake_word", "robertito").value.lower(),
            command_window_sec=float(self.declare_parameter("command_window_sec", 5.0).value),
            command_silence_sec=float(self.declare_parameter("command_silence_sec", 1.8).value),
            vosk_model_path=os.path.expanduser(
                self.declare_parameter(
                    "vosk_model_path", WakeWordConfig.vosk_model_path
                ).value
            ),
            api_base=self.declare_parameter("api_base", WakeWordConfig.api_base).value,
            api_model=self.declare_parameter("api_model", WakeWordConfig.api_model).value,
            api_key_env=self.declare_parameter("api_key_env", WakeWordConfig.api_key_env).value,
        )

        self._audio_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=64)
        self._stop_event = threading.Event()
        self._recognition_thread = threading.Thread(
            target=self._recognition_loop, name="wakeword-recognition", daemon=True
        )
        self._listening_for_command = False
        self._last_speech_time = 0.0
        self._command_buffer: list[str] = []
        self._command_started_at = 0.0

        self._tts_pub = self.create_publisher(String, "arturito/say", 10)

        self._sd = None
        self._vosk = None
        self._recognizer = None
        self._model = None
        self._stream = None

        self._setup_audio()
        self._setup_asr()
        self._recognition_thread.start()
        self.get_logger().info(
            f"WakeWordAssistant listo; escuchando '{self.cfg.wake_word}'..."
        )

    # ------------------------------------------------------------------ Setup
    def _setup_audio(self) -> None:
        sounddevice = _lazy_import("sounddevice", "sounddevice")
        self._sd = sounddevice

        try:
            self._stream = sounddevice.RawInputStream(
                samplerate=self.cfg.sample_rate,
                blocksize=self.cfg.block_size,
                dtype="int16",
                channels=1,
                callback=self._audio_callback,
            )
            self._stream.start()
        except Exception as exc:  # pragma: no cover - hardware setup
            raise RuntimeError(
                "No se pudo abrir el dispositivo de audio. Revisá que el micrófono esté disponible."
            ) from exc

    def _setup_asr(self) -> None:
        vosk = _lazy_import("vosk", "vosk")
        self._vosk = vosk

        model_path = self.cfg.vosk_model_path
        if not os.path.isdir(model_path):
            raise RuntimeError(
                f"Modelo de Vosk no encontrado en '{model_path}'. "
                "Descargá un modelo (por ejemplo 'vosk-model-small-es-0.42') "
                "y colocá la ruta en el parámetro 'vosk_model_path'."
            )
        self._model = vosk.Model(model_path)
        self._recognizer = vosk.KaldiRecognizer(self._model, self.cfg.sample_rate)
        self._recognizer.SetWords(True)

    # ---------------------------------------------------------------- Audio path
    def _audio_callback(self, indata, frames, time_info, status):  # pragma: no cover - callback
        if status:
            self.get_logger().warn("Audio callback status: %s", status)
        try:
            self._audio_queue.put_nowait(bytes(indata))
        except queue.Full:
            self.get_logger().warn("Buffer de audio lleno; descartando muestras.")

    def _recognition_loop(self) -> None:
        recognizer = self._recognizer
        while not self._stop_event.is_set():
            try:
                chunk = self._audio_queue.get(timeout=0.1)
            except queue.Empty:
                self._check_command_timeout()
                continue

            if recognizer.AcceptWaveform(chunk):
                result = json.loads(recognizer.Result())
                self._handle_transcript(result.get("text", "").strip())
            else:
                partial = json.loads(recognizer.PartialResult()).get("partial", "").strip()
                if partial:
                    self._handle_partial(partial)

    # -------------------------------------------------------------- Wake logic
    def _handle_partial(self, text: str) -> None:
        lower = text.lower()
        if not self._listening_for_command and self.cfg.wake_word in lower:
            self.get_logger().info(f"Wake word detectada en parcial: {text}")
            self._start_command_mode()

    def _handle_transcript(self, text: str) -> None:
        if not text:
            self._check_command_timeout()
            return

        lower = text.lower()
        now = time.monotonic()

        if self._listening_for_command:
            if self.cfg.wake_word in lower and not self._command_buffer:
                # Ignoramos la palabra clave inicial
                cleaned = lower.replace(self.cfg.wake_word, "").strip()
                if cleaned:
                    self._command_buffer.append(cleaned)
            else:
                self._command_buffer.append(text)
            self._last_speech_time = now
            self._check_command_timeout(force=False)
        elif self.cfg.wake_word in lower:
            self.get_logger().info(f"Wake word detectada: {text}")
            remainder = lower.replace(self.cfg.wake_word, "").strip()
            self._start_command_mode()
            if remainder:
                self._command_buffer.append(remainder)
                self._last_speech_time = now
                self._check_command_timeout(force=True)

    def _start_command_mode(self) -> None:
        self._listening_for_command = True
        self._command_buffer.clear()
        self._command_started_at = time.monotonic()
        self._last_speech_time = self._command_started_at

    def _check_command_timeout(self, force: bool = False) -> None:
        if not self._listening_for_command:
            return

        now = time.monotonic()
        silence_elapsed = now - self._last_speech_time
        total_elapsed = now - self._command_started_at
        if force or silence_elapsed >= self.cfg.command_silence_sec or total_elapsed >= self.cfg.command_window_sec:
            command = " ".join(self._command_buffer).strip()
            self._command_buffer.clear()
            self._listening_for_command = False
            if command:
                self.get_logger().info(f"Transcripción final: {command}")
                self._handle_command(command)
            else:
                self.get_logger().info("No se detectó comando después del wake word.")

    # ---------------------------------------------------------------- OpenAI
    def _handle_command(self, command: str) -> None:
        try:
            response = self._call_codex_api(command)
        except RuntimeError as exc:
            self.get_logger().error("OpenAI unavailable: %s", exc)
            return

        if response:
            self._publish_tts(response)

    def _call_codex_api(self, prompt: str) -> Optional[str]:
        api_key = os.getenv(self.cfg.api_key_env, "")
        if not api_key:
            message = (
                f"Environment variable '{self.cfg.api_key_env}' is required "
                "to call the OpenAI/Codex assistant."
            )
            self.get_logger().error(message)
            raise RuntimeError(message)

        requests = _lazy_import("requests", "requests")
        payload = {
            "model": self.cfg.api_model,
            "input": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": (
                                "Eres el asistente de voz de Robertito. "
                                "Responde de forma breve y amistosa en español.\n\n"
                                f"Consulta: {prompt}"
                            ),
                        }
                    ],
                }
            ],
        }

        headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        }

        try:
            response = requests.post(
                self.cfg.api_base, headers=headers, json=payload, timeout=20
            )
            response.raise_for_status()
        except requests.RequestException as exc:
            self.get_logger().error("Error llamando a la API Codex: %s", exc)
            return None

        data = response.json()
        try:
            output = data["output"][0]["content"][0]["text"]
            return output.strip()
        except (KeyError, IndexError, TypeError):
            self.get_logger().error("Respuesta inesperada de la API: %s", data)
            return None

    def _publish_tts(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._tts_pub.publish(msg)
        self.get_logger().info("Respuesta enviada al tópico arturito/say.")

    # ----------------------------------------------------------------- Cleanup
    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:  # pragma: no cover - ROS entry point
    rclpy.init(args=args)
    try:
        node = WakeWordAssistantNode()
    except RuntimeError as exc:
        rclpy.logging.get_logger("wake_word_assistant").error(str(exc))
        return

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
