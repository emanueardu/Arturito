import os
import re
import signal  # noqa: F401  # disponible para handlers futuros
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


try:  # pragma: no cover - dependency is optional at runtime
    import speech_recognition as sr
except ImportError as exc:  # pragma: no cover - guidance for missing dependency
    raise RuntimeError(
        "El paquete 'speech_recognition' es requerido para wake_word_listener_node. "
        "Instalalo con 'pip install SpeechRecognition'."
    ) from exc


class WakeWordListenerNode(Node):
    """Nodo ROS 2 que escucha el micrófono y publica cuando detecta una palabra clave."""

    def __init__(self) -> None:
        super().__init__("wake_word_listener")

        self._wake_word = (
            self.declare_parameter("wake_word", "robertito").value.lower().strip()
        )
        if not self._wake_word:
            raise ValueError("El parámetro 'wake_word' no puede estar vacío")
        self._wake_word_re = re.compile(rf"\b{re.escape(self._wake_word)}\b", re.IGNORECASE)

        self._language = self.declare_parameter("language", "es-ES").value
        self._phrase_time_limit = float(
            self.declare_parameter("phrase_time_limit", 3.0).value
        )
        self._listen_timeout = self.declare_parameter("listen_timeout", None).value
        self._energy_threshold = float(
            self.declare_parameter("energy_threshold", 250.0).value
        )
        self._dynamic_energy = bool(
            self.declare_parameter("dynamic_energy", True).value
        )
        self._device_index = self._parse_device_index(
            self.declare_parameter("microphone_device_index", None).value
        )
        self._cooldown = float(self.declare_parameter("cooldown_sec", 3.0).value)
        self._command_window_sec = float(
            self.declare_parameter("command_window_sec", 15.0).value
        )
        self._command_requires_wake = bool(
            self.declare_parameter("command_requires_wake", True).value
        )
        self._listen_retry_sec = float(
            self.declare_parameter("listen_retry_sec", 1.0).value
        )
        self._recalibrate_interval_sec = float(
            self.declare_parameter("recalibrate_interval_sec", 0.0).value
        )
        self._microphone_reopen_interval_sec = float(
            self.declare_parameter("microphone_reopen_interval_sec", 900.0).value
        )
        self._recognizer_backend = (
            self.declare_parameter("recognizer_backend", "google").value.lower().strip()
        )
        self._sample_rate = int(self.declare_parameter("sample_rate", 16000).value)
        self._chunk_size = int(self.declare_parameter("chunk_size", 1024).value)
        sensitivity = float(self.declare_parameter("sensitivity", 0.6).value)
        self._sensitivity = max(0.0, min(1.0, sensitivity))
        self._tts_topic = (
            self.declare_parameter("tts_topic", "/assistant/say")
            .get_parameter_value()
            .string_value
        )
        self._tts_suppress_enabled = bool(
            self.declare_parameter("tts_suppress_enabled", True).value
        )
        self._tts_mute_sec_per_char = float(
            self.declare_parameter("tts_mute_sec_per_char", 0.045).value
        )
        self._tts_mute_min_sec = float(
            self.declare_parameter("tts_mute_min_sec", 1.2).value
        )
        self._tts_mute_max_sec = float(
            self.declare_parameter("tts_mute_max_sec", 8.0).value
        )
        self._tts_wake_word_extra_sec = float(
            self.declare_parameter("tts_wake_word_extra_sec", 1.5).value
        )

        self._recognizer = sr.Recognizer()
        self._recognizer.energy_threshold = self._energy_threshold
        self._recognizer.dynamic_energy_threshold = self._dynamic_energy
        self._apply_sensitivity()

        self._mic: Optional[sr.Microphone] = None
        self._mic_lock = threading.Lock()
        self._mic_opened_monotonic = 0.0

        self._wake_topic = (
            self.declare_parameter("wake_topic", "/wake_word/detected")
            .get_parameter_value()
            .string_value
        )
        self._publisher = self.create_publisher(Bool, self._wake_topic, 10)
        self._score_publisher = self.create_publisher(Float32, "/wake_word/score", 10)
        self._command_topic = (
            self.declare_parameter("command_topic", "/tracker_control")
            .get_parameter_value()
            .string_value
        )
        self._recognized_text_topic = (
            self.declare_parameter("recognized_text_topic", "/assistant/listen_text")
            .get_parameter_value()
            .string_value
        )
        self._direct_command_routing_enabled = bool(
            self.declare_parameter("direct_command_routing_enabled", False).value
        )
        self._stop_phrases = self._parse_phrase_list(
            self.declare_parameter("stop_phrase", "dejar de seguir")
            .get_parameter_value()
            .string_value
        )
        self._start_phrases = self._parse_phrase_list(
            self.declare_parameter("start_phrase", "seguir")
            .get_parameter_value()
            .string_value
        )
        self._salute_phrases = self._parse_phrase_list(
            self.declare_parameter("salute_phrase", "saludar")
            .get_parameter_value()
            .string_value
        )
        self._clean_phrases = self._parse_phrase_list(
            self.declare_parameter("clean_phrase", "limpiar")
            .get_parameter_value()
            .string_value
        )
        self._conversation_end_phrases = self._parse_phrase_list(
            self.declare_parameter("conversation_end_phrases", "gracias,chau,listo")
            .get_parameter_value()
            .string_value
        )
        self._clean_stop_phrases = self._parse_phrase_list(
            self.declare_parameter("clean_stop_phrase", "dejar de limpiar")
            .get_parameter_value()
            .string_value
        )
        self._command_pub = self.create_publisher(String, self._command_topic, 10)
        self._recognized_text_pub = self.create_publisher(
            String, self._recognized_text_topic, 10
        )
        self._listening_active_topic = (
            self.declare_parameter("listening_active_topic", "/behavior/listening_active")
            .get_parameter_value()
            .string_value
        )
        self._health_topic = (
            self.declare_parameter("health_topic", "/behavior/health/wake_word")
            .get_parameter_value()
            .string_value
        )
        self._listening_timeout_topic = (
            self.declare_parameter("listening_timeout_topic", "/behavior/listening_timeout")
            .get_parameter_value()
            .string_value
        )
        self._health_heartbeat_sec = float(
            self.declare_parameter("health_heartbeat_sec", 5.0).value
        )
        self._listening_pub = self.create_publisher(Bool, self._listening_active_topic, 10)
        self._health_pub = self.create_publisher(String, self._health_topic, 10)
        self._listening_timeout_pub = self.create_publisher(Bool, self._listening_timeout_topic, 10)

        if self._tts_suppress_enabled and self._tts_topic:
            self.create_subscription(String, self._tts_topic, self._on_tts_input, 10)

        self._stop_event = threading.Event()
        # Timestamp monotonic actualizado por _listen_loop en cada iteración
        # útil. Lo lee _check_listener_stuck para detectar zombi vivo.
        self._listener_alive_ts: float = time.monotonic()
        self._listener_thread = threading.Thread(
            target=self._listen_loop, name="wake-word-listener", daemon=True
        )
        self._last_detection_ts = 0.0
        self._awaiting_command = False
        self._command_listen_until = 0.0
        self._suppress_until = 0.0
        self._listening_state = False
        self._health_state = "ok"

        self._listener_thread.start()
        self._health_timer = self.create_timer(
            max(1.0, self._health_heartbeat_sec), self._publish_health_heartbeat
        )
        # Timer dedicado a detectar listener thread zombi (separado del
        # heartbeat de health). Si el thread no actualiza alive_ts en >60s,
        # mata el proceso para que systemd lo reinicie y libere el mic.
        self._stuck_timer = self.create_timer(30.0, self._check_listener_stuck)
        self._publish_health("ok")
        self.get_logger().info(
            f"WakeWordListener listo. Esperando la palabra clave '{self._wake_word}'..."
        )

    @staticmethod
    def _parse_device_index(value) -> Optional[int]:
        if value is None or value == "":
            return None
        if isinstance(value, int):
            return value
        if isinstance(value, float):
            return int(value)
        if isinstance(value, str):
            stripped = value.strip()
            if stripped.isdigit():
                return int(stripped)
        return None

    @staticmethod
    def _parse_phrase_list(value: str) -> list[str]:
        if not value:
            return []
        sep = "|" if "|" in value else ","
        phrases = [part.strip().lower() for part in value.split(sep)]
        return [phrase for phrase in phrases if phrase]

    def _ensure_microphone(self) -> sr.Microphone:
        with self._mic_lock:
            if self._mic is None:
                self._mic = sr.Microphone(
                    device_index=self._device_index,
                    sample_rate=self._sample_rate,
                    chunk_size=self._chunk_size,
                )
                self._mic_opened_monotonic = time.monotonic()
                self._publish_health("ok")
                self.get_logger().info(
                    f"Micrófono abierto (device_index={self._device_index})"
                )
        return self._mic

    def _apply_sensitivity(self) -> None:
        energy = max(40.0, min(500.0, self._energy_threshold * (1.0 - self._sensitivity * 0.5)))
        self._recognizer.energy_threshold = energy

    def _calibrate_with_timeout(
        self, source: "sr.AudioSource", duration: float, timeout_sec: float
    ) -> bool:
        """Ejecuta adjust_for_ambient_noise con timeout duro.

        Retorna True si la calibración terminó limpia. False si terminó con
        excepción (semántica equivalente al `except Exception` previo). Si la
        calibración no termina en `timeout_sec` segundos, hace os._exit(2)
        para que systemd reinicie el proceso y libere el micrófono — un
        sys.exit() lo intercepta rclpy y el proceso queda zombi.
        """
        err: list[Exception] = []

        def _run() -> None:
            try:
                self._recognizer.adjust_for_ambient_noise(source, duration=duration)
            except Exception as exc:  # pragma: no cover - error path
                err.append(exc)

        worker = threading.Thread(target=_run, name="wake-cal", daemon=True)
        worker.start()
        worker.join(timeout=timeout_sec)
        if worker.is_alive():
            self.get_logger().error(
                f"Calibración de ruido ambiente colgada >{timeout_sec:.0f}s. "
                "Matando proceso para liberar micrófono (systemd reiniciará)."
            )
            self._publish_health("error: calibration_stuck")
            os._exit(2)
        if err:
            self.get_logger().warn(
                f"Fallo calibrando el ruido ambiente ({err[0]}). "
                "Continuando sin calibración."
            )
            return False
        return True

    def _check_listener_stuck(self) -> None:
        """Timer cada 30s: si el listener thread no actualizó alive_ts en >60s,
        asumimos zombi y matamos el proceso para reinicio limpio."""
        idle = time.monotonic() - self._listener_alive_ts
        if idle > 60.0:
            self.get_logger().error(
                f"Listener thread sin actividad >{idle:.0f}s, terminando"
            )
            self._publish_health("error: listener_stuck")
            os._exit(3)

    def _calculate_score(self) -> float:
        return float(max(0.0, min(1.0, 0.4 + self._sensitivity * 0.6)))

    def _listen_loop(self) -> None:  # pragma: no cover - hilo dependiente de hardware
        while not self._stop_event.is_set():
            try:
                microphone = self._ensure_microphone()
            except Exception as exc:
                self.get_logger().error(f"No se pudo abrir el micrófono: {exc}")
                self._publish_health(f"error: no se pudo abrir el micrófono ({exc})")
                self._set_listening_active(False)
                time.sleep(self._listen_retry_sec)
                continue

            with microphone as source:
                last_calibration = time.monotonic()
                self.get_logger().info("Calibrando ruido ambiente...")
                # Wrap con timeout duro: si PipeWire abrió el stream "raro" al
                # boot, adjust_for_ambient_noise se queda pidiendo samples para
                # siempre. El helper hace os._exit(2) si supera el timeout.
                if self._calibrate_with_timeout(source, duration=1.0, timeout_sec=15.0):
                    self._apply_sensitivity()

                while not self._stop_event.is_set():
                    if (
                        self._microphone_reopen_interval_sec > 0.0
                        and self._mic_opened_monotonic > 0.0
                        and (time.monotonic() - self._mic_opened_monotonic)
                        >= self._microphone_reopen_interval_sec
                    ):
                        self.get_logger().info(
                            "Reabriendo micrófono por mantenimiento preventivo."
                        )
                        self._set_listening_active(False)
                        with self._mic_lock:
                            self._mic = None
                            self._mic_opened_monotonic = 0.0
                        break

                    if self._should_suppress():
                        # Mientras TTS habla NO escuchamos, pero el thread
                        # SIGUE vivo: actualizamos alive_ts igual que en el
                        # path normal, sino el watchdog _check_listener_stuck
                        # mata el proceso tras 60s de TTS continuo y se pierde
                        # el micrófono hasta que algo lo respawnee.
                        self._listener_alive_ts = time.monotonic()
                        time.sleep(0.05)
                        continue

                    # Señal de "thread vivo" para _check_listener_stuck.
                    self._listener_alive_ts = time.monotonic()

                    if (
                        self._recalibrate_interval_sec > 0.0
                        and (time.monotonic() - last_calibration) >= self._recalibrate_interval_sec
                    ):
                        try:
                            self.get_logger().debug("Recalibrando ruido ambiente...")
                            self._recognizer.adjust_for_ambient_noise(source, duration=0.6)
                            self._apply_sensitivity()
                            last_calibration = time.monotonic()
                        except Exception as exc:
                            self.get_logger().warn(
                                f"Fallo recalibrando ruido ambiente ({exc})."
                            )

                    try:
                        audio = self._recognizer.listen(
                            source,
                            timeout=self._listen_timeout,
                            phrase_time_limit=self._phrase_time_limit,
                        )
                        # listen() retornó con audio: thread sigue vivo.
                        self._listener_alive_ts = time.monotonic()
                    except sr.WaitTimeoutError:
                        # Timeout normal de listen: igualmente cuenta como vida.
                        self._listener_alive_ts = time.monotonic()
                        self._expire_command_window()
                        continue
                    except Exception as exc:
                        self.get_logger().error(f"Error capturando audio: {exc}")
                        self._publish_health(f"error: captura de audio falló ({exc})")
                        self._set_listening_active(False)
                        with self._mic_lock:
                            self._mic = None
                        time.sleep(self._listen_retry_sec)
                        break

                    if self._should_suppress():
                        continue

                    transcript = self._transcribe(audio)
                    if not transcript:
                        self._expire_command_window()
                        continue

                    self._publish_health("ok")
                    self.get_logger().debug(f"Transcripción parcial: {transcript}")
                    self._handle_transcript(transcript)
            time.sleep(self._listen_retry_sec)

    def _should_suppress(self) -> bool:
        return time.monotonic() < self._suppress_until

    def _expire_command_window(self) -> None:
        if self._awaiting_command and time.monotonic() > self._command_listen_until:
            self._awaiting_command = False
            self._set_listening_active(False)
            self._listening_timeout_pub.publish(Bool(data=True))

    def _transcribe(self, audio: sr.AudioData) -> Optional[str]:
        try:
            if self._recognizer_backend == "sphinx":
                return self._recognizer.recognize_sphinx(audio, language=self._language)
            if self._recognizer_backend == "google":
                return self._recognizer.recognize_google(audio, language=self._language)
            self.get_logger().error(
                f"Backend de reconocimiento '{self._recognizer_backend}' no soportado"
            )
        except sr.UnknownValueError:
            self.get_logger().debug("No se entendió el audio capturado.")
        except sr.RequestError as exc:
            self.get_logger().error(f"Error del servicio de reconocimiento: {exc}")
            self._publish_health(f"error: reconocimiento no disponible ({exc})")
        except Exception as exc:  # pragma: no cover - path de error
            self.get_logger().error(f"Error inesperado reconociendo audio: {exc}")
            self._publish_health(f"error: reconocimiento falló ({exc})")
        return None

    def _handle_transcript(self, transcript: str) -> None:
        lower = transcript.lower()
        now = time.monotonic()

        detected_wake = bool(self._wake_word_re.search(lower))
        remainder = self._wake_word_re.sub("", lower).strip() if detected_wake else lower
        spoken_remainder = (
            self._wake_word_re.sub("", transcript).strip(" ,.;:!?")
            if detected_wake
            else transcript.strip()
        )

        if detected_wake:
            if now - self._last_detection_ts < self._cooldown:
                remaining = self._cooldown - (now - self._last_detection_ts)
                self.get_logger().debug(
                    f"Wake word repetida en cooldown ({remaining:.2f}s); "
                    "ignoro re-trigger pero proceso remainder"
                )
                # Si la ventana NO estaba abierta, no hay nada que procesar.
                if not self._awaiting_command:
                    return
                # Si ya estábamos awaiting_command, dejamos caer al branch
                # follow-up de abajo. Esto evita perder texto cuando el
                # listener detecta la wake word por error (eco del TTS, ruido)
                # durante una ventana ya abierta.
            else:
                self._last_detection_ts = now
                score = self._calculate_score()
                self._publisher.publish(Bool(data=True))
                self._score_publisher.publish(Float32(data=score))
                self.get_logger().info(
                    f"¡Wake word '{self._wake_word}' detectada! score={score:.2f}"
                )
                self._awaiting_command = True
                self._command_listen_until = now + self._command_window_sec
                self._set_listening_active(True)

        if self._awaiting_command and now <= self._command_listen_until:
            if spoken_remainder:
                low_text = spoken_remainder.lower().strip(" ,.;:!?")
                is_end = any(
                    p and p in low_text
                    for p in self._conversation_end_phrases
                )
                if is_end:
                    self._publish_recognized_text(spoken_remainder)
                    self._awaiting_command = False
                    self._set_listening_active(False)
                    self.get_logger().info(
                        f"Cierre detectado: '''{spoken_remainder}'''"
                    )
                    return
                command_text = self._extract_command(remainder)
                if command_text and self._direct_command_routing_enabled:
                    self._publish_command(command_text)
                self._publish_recognized_text(spoken_remainder)
                self._command_listen_until = now + self._command_window_sec
                self.get_logger().info(
                    f"Ventana renovada {self._command_window_sec}s"
                )
                return
        elif not self._command_requires_wake and self._awaiting_command:
            # Follow-up: ventana de conversación abierta, sin wake word esta
            # vez. PR4.1 fix: publicamos el texto SIEMPRE y renovamos ventana,
            # no solo cuando matchea un comando hardcoded.
            if not spoken_remainder:
                return
            low_text = spoken_remainder.lower().strip(" ,.;:!?")
            is_end = any(
                p and p in low_text for p in self._conversation_end_phrases
            )
            if is_end:
                self._publish_recognized_text(spoken_remainder)
                self._awaiting_command = False
                self._set_listening_active(False)
                self.get_logger().info(
                    f"Cierre detectado en follow-up: '{spoken_remainder}'"
                )
                return
            command_text = self._extract_command(remainder)
            if command_text and self._direct_command_routing_enabled:
                self._publish_command(command_text)
            self._publish_recognized_text(spoken_remainder)
            self._command_listen_until = now + self._command_window_sec
            self.get_logger().info(
                f"Follow-up renovado {self._command_window_sec}s: "
                f"'{spoken_remainder}'"
            )
            return

        self._expire_command_window()

    def _extract_command(self, text: str) -> Optional[str]:
        if not text:
            return None
        for phrase in self._stop_phrases:
            if phrase and phrase in text:
                return phrase
        for phrase in self._start_phrases:
            if phrase and phrase in text:
                if not any(stop in text for stop in self._stop_phrases):
                    return phrase
        for phrase in self._salute_phrases:
            if phrase and phrase in text:
                return phrase
        for phrase in self._clean_stop_phrases:
            if phrase and phrase in text:
                return phrase
        for phrase in self._clean_phrases:
            if phrase and phrase in text:
                return phrase
        return None

    def _publish_command(self, text: str) -> None:
        if not text:
            return
        msg = String()
        msg.data = text
        self._command_pub.publish(msg)
        self.get_logger().info(f"Comando de voz detectado: '{text}'")

    def _publish_recognized_text(self, text: str) -> None:
        cleaned = text.strip()
        if not cleaned:
            return
        self._recognized_text_pub.publish(String(data=cleaned))
        self.get_logger().info(f"Texto reconocido enviado: '{cleaned}'")

    def _on_tts_input(self, msg: String) -> None:
        if not self._tts_suppress_enabled:
            return
        text = msg.data.strip()
        if not text:
            return
        now = time.monotonic()
        estimated = max(self._tts_mute_min_sec, len(text) * self._tts_mute_sec_per_char)
        if self._wake_word in text.lower():
            estimated += self._tts_wake_word_extra_sec
        estimated = min(estimated, self._tts_mute_max_sec)
        self._suppress_until = max(self._suppress_until, now + estimated)
        if self._awaiting_command:
            self._command_listen_until = max(
                self._command_listen_until, self._suppress_until + self._command_window_sec
            )

    def _set_listening_active(self, active: bool) -> None:
        if self._listening_state == active:
            return
        self._listening_state = active
        self._listening_pub.publish(Bool(data=active))

    def _publish_health(self, text: str) -> None:
        cleaned = text.strip() if text else "ok"
        self._health_state = cleaned or "ok"
        self._health_pub.publish(String(data=self._health_state))

    def _publish_health_heartbeat(self) -> None:
        self._health_pub.publish(String(data=self._health_state))

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._set_listening_active(False)
        if self._listener_thread.is_alive():
            self._listener_thread.join(timeout=2.0)
        with self._mic_lock:
            self._mic = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WakeWordListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - comportamiento interactivo
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
