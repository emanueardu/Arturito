"""presence_orchestrator_node.

El "cerebro" reactivo de Robertito. Centraliza:

* Estados primarios (engaged, idle, drowsy, sleeping, working) y sus
  transiciones por presencia/ausencia de cara y por solicitudes externas.
* Triggers de interrupción (cliff, bump, startled, shaken, wake, bedtime,
  daily_greeting) con prioridad y cooldowns.
* Random behaviors en ENGAGED (micro-expresiones / mini-gestos motores) y
  en IDLE (suspiros, lookaround).
* Único publisher del topic ``robertito/eyes_expression`` para evitar el
  solapamiento que tenían los nodos anteriores.

Los demás nodos (api_chat, behavior_state, etc.) NO deben publicar a
expresiones ni a /cmd_vel directamente. En su lugar deben enviar JSON al
topic ``/robertito/orchestrator_request``.
"""
from __future__ import annotations

import json
import math
import random
import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist  # noqa: F401  (used indirectly by MotorGestures)
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger as TriggerSrv
from vision_msgs.msg import Detection2DArray

from robertito.orchestrator import (
    MotorGestures,
    PersistenceStore,
    State,
    StateMachine,
    Trigger,
    TriggerDetector,
    TtsPool,
)
from robertito.orchestrator.triggers import PRIORITIES, TriggerConfig

# Helpers reusados de api_chat / behavior_state para enriquecer saludos.
try:
    from robertito.family_companion import (
        build_family_weather_comment,
        fetch_weather_snapshot,
    )
except Exception:  # pragma: no cover
    fetch_weather_snapshot = None  # type: ignore
    build_family_weather_comment = None  # type: ignore
try:
    from robertito.useful_memory import UsefulMemoryStore
except Exception:  # pragma: no cover
    UsefulMemoryStore = None  # type: ignore


_RECENT_PUBLISH_BUFFER = 6


class PresenceOrchestrator(Node):
    """Nodo orchestrator. Mantenido bajo 600 LOC; lógica delega en submódulos."""

    def __init__(self) -> None:
        super().__init__("presence_orchestrator")

        # ─────────── Parámetros ───────────
        p = self._declare_params

        # Topics
        self._eyes_topic = p("eyes_expression_topic", "robertito/eyes_expression", str)
        self._tts_topic = p("tts_topic", "/assistant/say", str)
        self._tilt_topic = p("head_tilt_topic", "/head/tilt", str)
        self._cmd_vel_topic = p("cmd_vel_topic", "/cmd_vel", str)
        self._face_topic = p("face_detection_topic", "arturito/camera/faces", str)
        self._brightness_topic = p("brightness_topic", "arturito/camera/brightness", str)
        self._ultrasonic_topic = p("ultrasonic_topic", "arturito/ultrasonic", str)
        self._bumper_l_topic = p("bumper_left_topic", "arturito/bumper_left", str)
        self._bumper_r_topic = p("bumper_right_topic", "arturito/bumper_right", str)
        self._cliff_topic = p("cliff_topic", "arturito/cliff", str)
        self._imu_topic = p("imu_topic", "arturito/imu", str)
        self._wake_topic = p("wake_word_topic", "/wake_word/detected", str)
        self._speaking_topic = p("speaking_active_topic", "/behavior/speaking_active", str)
        self._ground_mode_topic = p("ground_mode_topic", "/robertito/ground_mode", str)
        self._req_topic = p("orchestrator_request_topic", "/robertito/orchestrator_request", str)

        # Persistence
        self._state_file = p("state_file", "/home/robot/.ros/robertito_orchestrator_state.json", str)

        # Timings
        self._no_face_to_idle_s = p("no_face_to_idle_s", 60, float)
        self._no_face_to_drowsy_s = p("no_face_to_drowsy_s", 300, float)
        self._no_face_to_sleeping_s = p("no_face_to_sleeping_s", 1800, float)
        self._fsm_period = p("fsm_tick_period_s", 0.5, float)

        # Greetings
        self._greet_casual_thr = p("greeting_casual_threshold_s", 300, float)
        self._greet_efusivo_thr = p("greeting_efusivo_threshold_s", 1800, float)
        self._greet_efusivo_cooldown = p("greeting_efusivo_cooldown_s", 1800, float)
        self._greet_casual_cooldown = p("greeting_casual_cooldown_s", 600, float)

        # Engaged randoms
        self._eng_expr_min = p("engaged_micro_expr_min_s", 15, float)
        self._eng_expr_max = p("engaged_micro_expr_max_s", 25, float)
        self._eng_expr_prob = p("engaged_micro_expr_probability", 0.3, float)
        self._eng_expr_dur = p("engaged_micro_expr_duration_s", 2.0, float)
        self._eng_expr_pool = list(p("engaged_micro_expr_pool", ["feliz"], list))
        self._eng_motion_min = p("engaged_micro_motion_min_s", 30, float)
        self._eng_motion_max = p("engaged_micro_motion_max_s", 60, float)
        self._eng_motion_prob = p("engaged_micro_motion_probability", 0.2, float)
        self._eng_motion_pool = list(p("engaged_micro_motion_pool", ["mini_spin_left"], list))

        # Engaged proactive questions
        self._eng_q_min = p("engaged_question_min_s", 60.0, float)
        self._eng_q_max = p("engaged_question_max_s", 150.0, float)
        self._eng_q_prob = p("engaged_question_probability", 0.8, float)
        self._eng_q_dur = p("engaged_question_duration_s", 6.0, float)
        self._eng_q_expr_pool = list(p("engaged_question_expr_pool", ["atento"], list))

        # Idle
        self._idle_look_min = p("idle_lookaround_min_s", 30, float)
        self._idle_look_max = p("idle_lookaround_max_s", 60, float)
        self._idle_sigh_min = p("idle_sigh_min_s", 300, float)
        self._idle_sigh_max = p("idle_sigh_max_s", 600, float)

        # Wake
        self._wake_pulse_dur = p("wake_pulse_duration_s", 0.4, float)

        # Bump / startled / cliff
        self._bump_reverse_dist = p("bump_reverse_distance_m", 0.05, float)
        self._bump_spin_deg = p("bump_spin_angle_deg", 30.0, float)
        self._startled_retreat = p("startled_retreat_distance_m", 0.10, float)
        self._cliff_reverse_dist = p("cliff_reverse_distance_m", 0.10, float)

        # Bedtime sleep lock: tras "buenas noches" la FSM queda forzada en
        # SLEEPING ignorando rostros para evitar despertar inmediato.
        self._bedtime_sleep_lock_s = p("bedtime_sleep_lock_s", 28800.0, float)
        self._bedtime_settle_delay_s = p("bedtime_settle_delay_s", 3.0, float)

        # Garantía visual "dormido en oscuridad + noche": independiente
        # del trigger bedtime (que dispara una vez por día), si pasada
        # cierta hora la cámara ve oscuridad, los ojos quedan en "dormido".
        # Threshold > el de bedtime trigger porque la cámara con gain alta
        # reporta valores mayores en "oscuro real".
        self._bedtime_start_hour_param = p("bedtime_start_hour", 23, int)
        self._bedtime_end_hour_param = p("bedtime_end_hour", 6, int)
        self._sleep_eyes_brightness_thr = p(
            "sleep_eyes_brightness_threshold", 70.0, float
        )
        self._sleep_eyes_repub_s = p("sleep_eyes_republish_s", 5.0, float)

        # School farewell (lun-vie 7:20)
        self._school_farewell_cold_thr = p("school_farewell_cold_threshold_c", 14.0, float)
        self._school_farewell_cold_tip = p(
            "school_farewell_cold_tip", "Está fresco, abríguense bien.", str
        )

        # Weather (Burzaco default — el usuario no necesita pasarlo)
        self._weather_enabled = p("weather_enabled", True, bool)
        self._weather_lat = p("weather_lat", -34.8270, float)
        self._weather_lon = p("weather_lon", -58.3930, float)
        self._weather_unit = p("weather_unit", "celsius", str)
        self._weather_tz = p("weather_timezone", "America/Argentina/Buenos_Aires", str)
        self._weather_timeout = p("weather_timeout_s", 6.0, float)
        self._weather_cache_s = p("weather_cache_s", 1800.0, float)
        self._weather_cache: Optional[dict] = None
        self._weather_cache_at: float = 0.0

        # Memory / reminders
        self._memory_file = p("memory_file", "/home/robot/.ros/robertito_memory.json", str)
        self._memory_tz = p("memory_timezone", "America/Argentina/Buenos_Aires", str)
        self._morning_rem_limit = p("morning_reminders_limit", 5, int)
        self._memory_store: Optional[object] = None
        if UsefulMemoryStore is not None:
            try:
                self._memory_store = UsefulMemoryStore(self._memory_file)
                self._memory_store.load()
            except Exception:
                self.get_logger().warning(
                    f"No pude inicializar UsefulMemoryStore en {self._memory_file}"
                )

        # Tilt inicial al boot (cámara apunta hacia donde suele haber caras)
        self._default_tilt_deg = p("default_tilt_deg", 25.0, float)
        self._default_tilt_delay = p("default_tilt_delay_s", 3.0, float)

        # Random TTS probabilities
        self._micro_expr_tts_prob = p("micro_expr_tts_probability", 0.4, float)
        self._idle_sigh_tts_prob = p("idle_sigh_tts_probability", 0.7, float)

        # Per-state expressions
        self._expr_engaged = p("expr_engaged", "calma", str)
        self._expr_idle = p("expr_idle", "calma", str)
        self._expr_drowsy = p("expr_drowsy", "dormitando", str)
        self._expr_sleeping = p("expr_sleeping", "dormido", str)
        self._expr_working = p("expr_working", "atento", str)
        self._expr_greet_efusivo = p("expr_greeting_efusivo", "corazones", str)
        self._expr_greet_casual = p("expr_greeting_casual", "feliz", str)
        self._expr_daily_greeting = p("expr_daily_greeting", "feliz", str)
        self._expr_bedtime = p("expr_bedtime", "feliz", str)
        self._expr_bump = p("expr_bump", "sorprendido", str)
        self._expr_startled = p("expr_startled", "sorprendido", str)
        self._expr_shaken = p("expr_shaken", "risa_fuerte", str)
        self._expr_cliff = p("expr_cliff", "sorprendido", str)
        self._expr_wake_ack = p("expr_wake_ack", "atento", str)

        # Motor speeds
        motor_lin = p("motor_linear_speed_mps", 0.08, float)
        motor_ang = p("motor_angular_speed_radps", 0.5, float)

        # External-publisher courtesy window
        self._external_respect_s = p("external_expression_respect_s", 10, float)

        # ─────────── Modo limpieza (mute total) ───────────
        # Cuando clean_quick toma control, el orchestrator NO publica cmd_vel,
        # NO dispara motor_gestures (cliff/bump/startled/shaken) y SOLO permite
        # la expresión fija `_clean_mode_expr`. clean_quick maneja toda la
        # lógica reactiva durante la limpieza con su propio loop.
        self._clean_mode_active: bool = False
        self._clean_mode_expr = p("clean_mode_expression", "atento", str)
        self._clean_status_topic = p("clean_status_topic", "/robot_web/clean_status", str)

        # ─────────── Conversación (PR4.1) ───────────
        # Cuando wake_word_listener abre ventana de follow-up, mostramos una
        # expresión de "escuchando" (variante de atento con glint más grande).
        # Limpieza > conversación: si _clean_mode_active, no se cambia.
        self._listening_active: bool = False
        self._listening_active_topic = p(
            "listening_active_topic", "/behavior/listening_active", str
        )
        self._listening_expr = p("listening_expression", "escuchando", str)

        # ─────────── Submódulos ───────────
        now = self._now()
        trig_cfg = TriggerConfig(
            bump_cooldown_s=p("bump_cooldown_s", 2.0, float),
            startled_distance_min_cm=p("startled_distance_min_cm", 5.0, float),
            startled_distance_max_cm=p("startled_distance_max_cm", 25.0, float),
            startled_cooldown_s=p("startled_cooldown_s", 5.0, float),
            shaken_imu_threshold_g=p("shaken_imu_threshold_g", 1.5, float),
            shaken_imu_release_g=p("shaken_imu_release_g", 0.8, float),
            shaken_min_duration_s=p("shaken_min_duration_s", 0.5, float),
            shaken_release_duration_s=p("shaken_release_duration_s", 1.0, float),
            shaken_cooldown_s=p("shaken_cooldown_s", 3.0, float),
            cliff_cooldown_s=p("cliff_cooldown_s", 3.0, float),
            bedtime_enabled=p("bedtime_enabled", True, bool),
            bedtime_start_hour=p("bedtime_start_hour", 23, int),
            bedtime_brightness_threshold=p("bedtime_brightness_threshold", 40.0, float),
            bedtime_brightness_consistent_s=p("bedtime_brightness_consistent_s", 30.0, float),
            bedtime_cooldown_s=p("bedtime_cooldown_s", 600.0, float),
            daily_greeting_enabled=p("daily_greeting_enabled", True, bool),
            daily_greeting_start_hour=p("daily_greeting_start_hour", 6, int),
            daily_greeting_end_hour=p("daily_greeting_end_hour", 11, int),
            daily_greeting_cooldown_s=p("daily_greeting_cooldown_s", 600.0, float),
            school_farewell_enabled=p("school_farewell_enabled", True, bool),
            school_farewell_hour=p("school_farewell_hour", 7, int),
            school_farewell_minute=p("school_farewell_minute", 20, int),
            school_farewell_window_min=p("school_farewell_window_min", 15, int),
            school_farewell_weekdays=tuple(
                int(d) for d in p("school_farewell_weekdays", [1, 2, 3, 4, 5], list)
            ),
            school_farewell_cooldown_s=p("school_farewell_cooldown_s", 600.0, float),
        )
        self._persistence = PersistenceStore(self._state_file, self.get_logger())
        self._triggers = TriggerDetector(trig_cfg, self._persistence)
        self._fsm = StateMachine(State.WAKING_UP, now, self.get_logger())

        tts_pools = {
            "daily_greeting": [s for s in p("tts_daily_greeting", ["_"], list) if s != "_"],
            "greeting_efusivo": [s for s in p("tts_greeting_efusivo", ["_"], list) if s != "_"],
            "greeting_casual": [s for s in p("tts_greeting_casual", ["_"], list) if s != "_"],
            "micro_expr": [s for s in p("tts_micro_expr", ["_"], list) if s != "_"],
            "idle_sigh": [s for s in p("tts_idle_sigh", ["_"], list) if s != "_"],
            "bump": [s for s in p("tts_bump", ["_"], list) if s != "_"],
            "startled": [s for s in p("tts_startled", ["_"], list) if s != "_"],
            "shaken": [s for s in p("tts_shaken", ["_"], list) if s != "_"],
            "cliff": [s for s in p("tts_cliff", ["_"], list) if s != "_"],
            "wake_ack": [s for s in p("tts_wake_ack", ["_"], list) if s != "_"],
            "bedtime": [s for s in p("tts_bedtime", ["_"], list) if s != "_"],
            "engaged_questions": [s for s in p("tts_engaged_questions", ["_"], list) if s != "_"],
            "school_farewell": [s for s in p("tts_school_farewell", ["_"], list) if s != "_"],
        }
        self._tts = TtsPool(tts_pools)

        # ─────────── Publishers ───────────
        self._eyes_pub = self.create_publisher(String, self._eyes_topic, 10)
        self._tts_pub = self.create_publisher(String, self._tts_topic, 10)
        self._tilt_pub = self.create_publisher(Float32, self._tilt_topic, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        self._motor = MotorGestures(
            cmd_vel_publisher=self._cmd_vel_pub,
            head_tilt_publisher=self._tilt_pub,
            linear_speed_mps=motor_lin,
            angular_speed_radps=motor_ang,
            logger=self.get_logger(),
        )

        # Tilt al boot: lo enviamos en un thread con delay para que el
        # subscriber del uart_bridge esté ya conectado y no se pierda.
        def _send_default_tilt() -> None:
            time.sleep(max(0.5, self._default_tilt_delay))
            try:
                self._motor.tilt_to(self._default_tilt_deg)
                self.get_logger().info(
                    f"Tilt inicial enviado: {self._default_tilt_deg:.1f}°"
                )
            except Exception:
                self.get_logger().exception("default tilt falló")

        threading.Thread(target=_send_default_tilt, name="default-tilt", daemon=True).start()

        # ─────────── Estado runtime ───────────
        self._lock = threading.Lock()
        self._last_face_seen_at: float = self._persistence.get_last_face_seen_at() or now
        self._speaking_active: bool = False
        self._working_until: Optional[float] = None
        self._random_pause_until: float = 0.0
        self._bedtime_lock_until: float = 0.0
        self._next_engaged_expr_at: float = now + random.uniform(self._eng_expr_min, self._eng_expr_max)
        self._next_engaged_motion_at: float = now + random.uniform(self._eng_motion_min, self._eng_motion_max)
        self._next_engaged_question_at: float = now + random.uniform(self._eng_q_min, self._eng_q_max)
        self._next_idle_look_at: float = now + random.uniform(self._idle_look_min, self._idle_look_max)
        self._next_idle_sigh_at: float = now + random.uniform(self._idle_sigh_min, self._idle_sigh_max)
        self._recent_expressions: list[str] = []  # frases que pubicamos nosotros
        self._last_self_publish_at: float = 0.0
        # Mientras esto vale > now, _update_primary_state NO publica la
        # expresión base del estado al transicionar (evita pisar saludos,
        # preguntas y otras expresiones de "vida" que duran varios segundos).
        self._expression_hold_until: float = 0.0
        # Última vez que forzamos "dormido" por oscuridad+noche.
        self._last_sleep_eyes_at: float = 0.0

        # ─────────── Subscriptions ───────────
        sub_group = ReentrantCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(Detection2DArray, self._face_topic, self._on_faces, 10, callback_group=sub_group)
        self.create_subscription(Float32, self._brightness_topic, self._on_brightness, 10, callback_group=sub_group)
        self.create_subscription(Range, self._ultrasonic_topic, self._on_ultrasonic, 10, callback_group=sub_group)
        self.create_subscription(Bool, self._bumper_l_topic, self._on_bumper_l, 10, callback_group=sub_group)
        self.create_subscription(Bool, self._bumper_r_topic, self._on_bumper_r, 10, callback_group=sub_group)
        self.create_subscription(Bool, self._cliff_topic, self._on_cliff, 10, callback_group=sub_group)
        self.create_subscription(Imu, self._imu_topic, self._on_imu, 10, callback_group=sub_group)
        self.create_subscription(Bool, self._wake_topic, self._on_wake, 10, callback_group=sub_group)
        self.create_subscription(Bool, self._speaking_topic, self._on_speaking, 10, callback_group=sub_group)
        self.create_subscription(String, self._eyes_topic, self._on_eyes_published, 10, callback_group=sub_group)
        self.create_subscription(String, self._req_topic, self._on_request, 10, callback_group=sub_group)
        self.create_subscription(
            Bool, self._clean_status_topic, self._on_clean_status, 10,
            callback_group=sub_group,
        )
        self.create_subscription(
            Bool, self._listening_active_topic, self._on_listening_active, 10,
            callback_group=sub_group,
        )

        # ─────────── Timer FSM ───────────
        self._fsm_timer = self.create_timer(self._fsm_period, self._fsm_tick, callback_group=timer_group)

        # ─────────── Service de admin ───────────
        self._svc_get_state = self.create_service(
            TriggerSrv, "/orchestrator/get_state", self._on_get_state_srv
        )

        self.get_logger().info(
            f"presence_orchestrator listo (tick={self._fsm_period:.2f}s, expr_topic={self._eyes_topic})"
        )

    # ─────────────────── Helpers ───────────────────

    def _declare_params(self, name: str, default, _type) -> object:
        try:
            if _type is list and (default is None or len(default) == 0):
                from rcl_interfaces.msg import ParameterDescriptor, ParameterType
                desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
                value = self.declare_parameter(name, [], desc).value
                if value is None:
                    value = []
            else:
                # Si el caller pidió tipo float, forzamos el default a float
                # antes de declarar — sino ROS infiere INTEGER del default y
                # rechaza luego los valores DOUBLE del YAML (warning de tipos).
                effective_default = default
                if _type is float and isinstance(default, (int, float)) and not isinstance(default, bool):
                    effective_default = float(default)
                value = self.declare_parameter(name, effective_default).value
        except Exception as exc:
            self.get_logger().warning(
                f"declare_parameter('{name}') fallo: {exc}; usando default"
            )
            value = default
        if _type is bool:
            return bool(value)
        if _type is int:
            try:
                return int(value)
            except (TypeError, ValueError):
                return int(default)
        if _type is float:
            try:
                return float(value)
            except (TypeError, ValueError):
                return float(default)
        if _type is str:
            return str(value) if value is not None else str(default)
        if _type is list:
            return list(value) if isinstance(value, (list, tuple)) else list(default)
        return value

    def _now(self) -> float:
        return time.monotonic()

    def _publish_expression(self, expr: str) -> None:
        if not expr:
            return
        # Mute durante limpieza: solo permitimos la expresión fija del modo
        # limpieza. Esto bloquea expresiones reactivas (sorprendido por bump,
        # risa_fuerte por shaken, etc.) y deja a clean_quick mostrar lo que
        # quiera. wake_ack default = "atento" coincide con clean_mode_expr y
        # pasa OK.
        if self._clean_mode_active and expr != self._clean_mode_expr:
            return
        self._eyes_pub.publish(String(data=expr))
        self._last_self_publish_at = self._now()
        self._recent_expressions.append(expr)
        if len(self._recent_expressions) > _RECENT_PUBLISH_BUFFER:
            self._recent_expressions = self._recent_expressions[-_RECENT_PUBLISH_BUFFER:]

    def _say(self, category: str) -> None:
        phrase = self._tts.sample(category)
        if phrase:
            self._tts_pub.publish(String(data=phrase))

    def _say_text(self, text: str) -> None:
        if text:
            self._tts_pub.publish(String(data=text))

    def _pause_randoms(self, seconds: float) -> None:
        target = self._now() + max(0.0, float(seconds))
        if target > self._random_pause_until:
            self._random_pause_until = target

    # ─────────────────── Subscription handlers ───────────────────

    def _on_faces(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            return
        now = self._now()
        with self._lock:
            prev = self._last_face_seen_at
            self._last_face_seen_at = now
        # "Pulse de te vi": si una cara aparece después de un huequito (10s+)
        # pero NO suficiente para haber ido a IDLE/DROWSY, mostrá una expresión
        # corta de feliz para que el robot se sienta atento. Solo en ENGAGED
        # actual, sin TTS (las frases las maneja la lógica de greetings).
        gap = now - prev
        if (
            self._fsm.current_state == State.ENGAGED
            and 10.0 <= gap < self._greet_casual_thr
            and now >= self._expression_hold_until
        ):
            self._publish_expression(self._expr_greet_casual)
            self._expression_hold_until = now + 2.0

            def _restore() -> None:
                time.sleep(2.0)
                if self._fsm.current_state == State.ENGAGED:
                    self._publish_expression(self._expression_for_state(State.ENGAGED))

            threading.Thread(target=_restore, name="face-pulse-restore", daemon=True).start()

        self._persistence.set_last_face_seen_at(now)

    def _on_brightness(self, msg: Float32) -> None:
        self._triggers.update_brightness(float(msg.data), self._now())

    def _on_ultrasonic(self, msg: Range) -> None:
        try:
            d_cm = float(msg.range) * 100.0
        except Exception:
            return
        if not math.isfinite(d_cm):
            return
        self._triggers.update_ultrasonic(d_cm)

    def _on_bumper_l(self, msg: Bool) -> None:
        self._triggers.update_bumper_l(bool(msg.data), self._now())

    def _on_bumper_r(self, msg: Bool) -> None:
        self._triggers.update_bumper_r(bool(msg.data), self._now())

    def _on_cliff(self, msg: Bool) -> None:
        self._triggers.update_cliff(bool(msg.data))

    def _on_imu(self, msg: Imu) -> None:
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        magnitude_g = math.sqrt(ax * ax + ay * ay + az * az) / 9.81
        self._triggers.update_imu_magnitude(magnitude_g, self._now())

    def _on_clean_status(self, msg: Bool) -> None:
        """Modo limpieza: mute total de reflejos del orchestrator.

        clean_quick maneja sus propios bumpers/proximidad/cliff. Si el
        orchestrator también reacciona, se pisan dos lógicas y el robot
        hace movimientos imprevistos durante la limpieza.
        """
        new_state = bool(msg.data)
        if new_state == self._clean_mode_active:
            return
        action = "muting" if new_state else "resuming"
        self.get_logger().info(
            f"clean_mode_active: {self._clean_mode_active} -> {new_state} "
            f"({action} gestures, cmd_vel and reactive expressions)"
        )
        self._clean_mode_active = new_state
        if new_state:
            # Forzar expresión fija del modo limpieza (atento por default).
            # Esto pasa por _publish_expression, que respeta el flag.
            self._publish_expression(self._clean_mode_expr)

    def _on_listening_active(self, msg: Bool) -> None:
        """Cambia expresión cuando el listener abre/cierra ventana de conversación.

        Limpieza tiene prioridad: si _clean_mode_active, no toca la expresión.
        Cuando la ventana cierra, deja al FSM normal recuperar control en el
        próximo tick.
        """
        new_state = bool(msg.data)
        if new_state == self._listening_active:
            return
        self._listening_active = new_state
        if self._clean_mode_active:
            return  # limpieza > conversación
        if new_state:
            self.get_logger().info(
                f"Ventana de conversación abierta -> expresión "
                f"'{self._listening_expr}'"
            )
            self._publish_expression(self._listening_expr)
        else:
            self.get_logger().info(
                "Ventana de conversación cerrada -> expresión normal"
            )
            # No forzamos expresión: dejamos al FSM tomar control en el
            # próximo tick.

    def _on_wake(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        self._triggers.update_wake(self._now())

    def _on_speaking(self, msg: Bool) -> None:
        with self._lock:
            self._speaking_active = bool(msg.data)

    def _on_eyes_published(self, msg: String) -> None:
        """Detecta mensajes de expresión que NO publicamos nosotros."""
        if not msg.data:
            return
        # Si la frase coincide con algo publicado por nosotros recientemente, ignorar.
        if msg.data in self._recent_expressions:
            return
        # Externo: courtesy pause
        self._pause_randoms(self._external_respect_s)

    def _on_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().debug(f"orchestrator_request: payload no-JSON: {msg.data!r}")
            return
        action = str(payload.get("action") or "").strip().lower()
        value = payload.get("value")
        duration = payload.get("duration_s")
        if action == "set_expression" and isinstance(value, str):
            self._publish_expression(value)
            try:
                dur = float(duration)
            except (TypeError, ValueError):
                dur = 0.0
            if dur > 0:
                self._working_until = self._now() + dur
        elif action == "say" and isinstance(value, str):
            self._say_text(value)
        elif action == "trigger" and isinstance(value, str):
            self._handle_named_trigger(value)
        elif action == "set_state" and isinstance(value, str):
            self._request_state(value)
        else:
            self.get_logger().debug(f"orchestrator_request desconocido: {payload}")

    def _handle_named_trigger(self, name: str) -> None:
        n = name.strip().lower()
        if n in {"wake_ack", "wake"}:
            self._publish_expression(self._expr_wake_ack)
            self._say("wake_ack")
        elif n == "bump":
            self._handle_bump("")
        elif n == "startled":
            self._handle_startled()
        elif n == "shaken":
            self._handle_shaken()
        elif n == "cliff":
            self._handle_cliff()

    def _request_state(self, name: str) -> None:
        try:
            new_state = State(name.lower())
        except ValueError:
            return
        if self._fsm.transition_to(new_state, self._now()):
            self._publish_expression(self._expression_for_state(new_state))

    # ─────────────────── FSM tick ───────────────────

    def _fsm_tick(self) -> None:
        now = self._now()

        # 1) Triggers de interrupción por prioridad descendente
        if self._handle_high_priority_triggers(now):
            return

        # 2) Ajustar estado primario según presencia/ausencia + speaking
        self._update_primary_state(now)

        # 2.5) GARANTÍA "dormido cuando oscuro + tarde":
        # independiente del estado FSM y del trigger bedtime (que solo
        # dispara una vez por día), si está oscuro Y es horario de noche
        # forzamos los ojos a "dormido". Esto desacopla la expresión del
        # estado de FSM y resiste publicaciones externas.
        self._enforce_sleep_eyes_when_dark_late(now)

        # 3) Si estamos en pausa por publisher externo, no random behaviors
        if now < self._random_pause_until:
            return
        if self._speaking_active:
            return

        # 4) Random behaviors según estado
        state = self._fsm.current_state
        if state == State.ENGAGED:
            self._maybe_engaged_micro_expr(now)
            self._maybe_engaged_micro_motion(now)
            self._maybe_engaged_question(now)
        elif state == State.IDLE:
            self._maybe_idle_lookaround(now)
            self._maybe_idle_sigh(now)

    def _enforce_sleep_eyes_when_dark_late(self, now: float) -> None:
        """Si está oscuro y es noche, los ojos quedan en 'dormido' siempre.

        No requiere que bedtime haya disparado: la expresión visual sigue
        las condiciones físicas (luz + hora). Re-publica cada `_sleep_eyes_repub_s`
        para sobreescribir cualquier publicador externo intermitente.
        """
        import datetime as _dt
        local_hour = _dt.datetime.now().hour
        in_night = (
            local_hour >= self._bedtime_start_hour_param
            or local_hour <= self._bedtime_end_hour_param
        )
        if not in_night:
            return
        snap = self._triggers.snapshot()
        brightness = snap.get("brightness")
        if brightness is None:
            return
        if float(brightness) > self._sleep_eyes_brightness_thr:
            return
        # Cumple condición. Republicar dormido si pasó el período.
        if (now - self._last_sleep_eyes_at) < self._sleep_eyes_repub_s:
            return
        self._publish_expression(self._expr_sleeping)
        self._last_sleep_eyes_at = now
        # También extendemos el hold para que update_primary_state no pise.
        self._expression_hold_until = max(
            self._expression_hold_until, now + self._sleep_eyes_repub_s + 1.0
        )

    def _handle_high_priority_triggers(self, now: float) -> bool:
        if self._triggers.cliff_active(now):
            self._handle_cliff()
            self._triggers.mark_handled(Trigger.CLIFF, now)
            return True
        active, side = self._triggers.bump_active(now)
        if active:
            self._handle_bump(side)
            self._triggers.mark_handled(Trigger.BUMP, now)
            return True
        if self._triggers.startled_active(now):
            self._handle_startled()
            self._triggers.mark_handled(Trigger.STARTLED, now)
            return True
        if self._triggers.shaken_active(now):
            self._handle_shaken()
            self._triggers.mark_handled(Trigger.SHAKEN, now)
            return True
        if self._triggers.wake_active(now):
            self._handle_wake_ack()
            self._triggers.mark_handled(Trigger.WAKE, now)
            return True
        if self._triggers.bedtime_active(now):
            self._handle_bedtime(now)
            self._triggers.mark_handled(Trigger.BEDTIME, now)
            return True
        if self._triggers.school_farewell_due(now):
            self._handle_school_farewell()
            self._triggers.mark_handled(Trigger.SCHOOL_FAREWELL, now)
            return True
        if self._triggers.daily_greeting_due(now):
            self._handle_daily_greeting()
            self._triggers.mark_handled(Trigger.DAILY_GREETING, now)
            return True
        return False

    # ─────────────────── Trigger handlers ───────────────────

    def _handle_cliff(self) -> None:
        if self._clean_mode_active:
            return  # clean_quick ya lo maneja con su propia lógica.
        self._publish_expression(self._expr_cliff)
        self._say("cliff")
        self._motor.reverse(self._cliff_reverse_dist)

    def _handle_bump(self, side: str) -> None:
        if self._clean_mode_active:
            return  # clean_quick ya lo maneja con su propia lógica.
        self._publish_expression(self._expr_bump)
        self._say("bump")
        self._motor.bump_reaction(side, self._bump_reverse_dist, self._bump_spin_deg)

    def _handle_startled(self) -> None:
        if self._clean_mode_active:
            return  # clean_quick ya gestiona la proximidad por ultrasónico.
        self._publish_expression(self._expr_startled)
        self._say("startled")
        self._motor.reverse(self._startled_retreat)

    def _handle_shaken(self) -> None:
        if self._clean_mode_active:
            return  # No reír / sacudirse durante una limpieza pacífica.
        self._publish_expression(self._expr_shaken)
        self._say("shaken")

    def _handle_wake_ack(self) -> None:
        self._publish_expression(self._expr_wake_ack)
        self._say("wake_ack")

    def _handle_bedtime(self, now: float) -> None:
        # 1) Saluda con expresión "feliz" (expr_bedtime) mientras dice "buenas noches".
        # 2) Marca el lock para que la FSM no salga de SLEEPING aunque vea un rostro.
        # 3) Tras unos segundos pasa a expr_sleeping (dormido).
        self._publish_expression(self._expr_bedtime)
        self._say("bedtime")
        self._persistence.mark_bedtime_announced(now)
        self._bedtime_lock_until = now + self._bedtime_sleep_lock_s
        self._fsm.transition_to(State.SLEEPING, now)

        def _settle_to_sleeping() -> None:
            time.sleep(max(0.5, self._bedtime_settle_delay_s))
            if self._fsm.current_state == State.SLEEPING:
                self._publish_expression(self._expr_sleeping)

        threading.Thread(target=_settle_to_sleeping, name="bedtime-settle", daemon=True).start()

    def _handle_daily_greeting(self) -> None:
        """Saludo matutino enriquecido: clima de Burzaco + recordatorios pendientes.

        Libera el bedtime lock para que la FSM vuelva a depender de presencia.
        El fetch de clima se hace en thread para no bloquear el FSM tick.
        """
        self._publish_expression(self._expr_daily_greeting)
        self._persistence.mark_daily_greeting_done()
        self._bedtime_lock_until = 0.0
        self._expression_hold_until = self._now() + 8.0

        def _compose_and_say() -> None:
            base = self._tts.sample("daily_greeting") or "Buenos días."
            weather_line = self._weather_line()
            reminders_line = self._reminders_line_morning()
            parts = [base]
            if weather_line:
                parts.append(weather_line)
            if reminders_line:
                parts.append(reminders_line)
            text = " ".join(p for p in parts if p)
            self._say_text(text)

        threading.Thread(target=_compose_and_say, name="morning-greet", daemon=True).start()

    def _handle_school_farewell(self) -> None:
        """Despedida 7:20 lun-vie: frase amigable + tip de abrigo si está fresco."""
        self._publish_expression(self._expr_greet_casual)
        self._persistence.mark_school_farewell_done()
        self._expression_hold_until = self._now() + 6.0

        def _compose_and_say() -> None:
            base = self._tts.sample("school_farewell") or "Que tengan un lindo día"
            snapshot = self._fetch_weather_cached()
            tip = ""
            if snapshot:
                temp = snapshot.get("temperature_rounded")
                if temp is not None and float(temp) <= self._school_farewell_cold_thr:
                    tip = self._school_farewell_cold_tip
            text = f"{base}. {tip}".strip().rstrip(".") + "."
            self._say_text(text)

        threading.Thread(target=_compose_and_say, name="school-farewell", daemon=True).start()

    # ─────────────────── Weather + memory helpers ───────────────────

    def _fetch_weather_cached(self) -> Optional[dict]:
        """Devuelve el snapshot cacheado si está fresco; sino refetch."""
        if not self._weather_enabled or fetch_weather_snapshot is None:
            return None
        now = self._now()
        if (
            self._weather_cache is not None
            and (now - self._weather_cache_at) < self._weather_cache_s
        ):
            return self._weather_cache
        try:
            snap = fetch_weather_snapshot(
                latitude=self._weather_lat,
                longitude=self._weather_lon,
                temperature_unit=self._weather_unit,
                timeout_sec=self._weather_timeout,
                timezone_name=self._weather_tz,
            )
        except Exception:
            self.get_logger().warning("fetch_weather_snapshot lanzó excepción")
            return None
        if snap:
            self._weather_cache = snap
            self._weather_cache_at = now
        return snap

    def _weather_line(self) -> str:
        if build_family_weather_comment is None:
            return ""
        snap = self._fetch_weather_cached()
        if not snap:
            return ""
        try:
            return build_family_weather_comment(snap) or ""
        except Exception:
            return ""

    def _reminders_line_morning(self) -> str:
        """Lista corta de recordatorios pendientes para hoy (si los hay)."""
        if self._memory_store is None:
            return ""
        try:
            from datetime import datetime as _dtdt
            from zoneinfo import ZoneInfo as _ZI
            tz = _ZI(self._memory_tz)
            now_local = _dtdt.now(tz)
            self._memory_store.load()
            today_pending = self._memory_store.today_pending(now=now_local)
            items = today_pending[: max(1, int(self._morning_rem_limit))]
            if not items:
                return ""
            texts = [str(it.get("text", "")).strip() for it in items if it.get("text")]
            texts = [t for t in texts if t]
            if not texts:
                return ""
            if len(texts) == 1:
                return f"Te recuerdo: {texts[0]}."
            joined = "; ".join(texts[:-1]) + f"; y {texts[-1]}"
            return f"Te recuerdo para hoy: {joined}."
        except Exception:
            self.get_logger().warning("No pude armar la línea de recordatorios")
            return ""

    # ─────────────────── Estado primario ───────────────────

    def _update_primary_state(self, now: float) -> None:
        # WORKING tiene precedencia mientras esté activo
        if self._working_until is not None:
            if now < self._working_until:
                if self._fsm.current_state != State.WORKING:
                    self._fsm.transition_to(State.WORKING, now)
                    self._publish_expression(self._expression_for_state(State.WORKING))
                return
            self._working_until = None

        # Bedtime lock: tras "buenas noches" la FSM queda forzada en SLEEPING
        # ignorando rostros. Se libera al disparar daily_greeting o cuando
        # vence el timeout (default: 8h, debería cubrir la noche).
        if now < self._bedtime_lock_until:
            if self._fsm.current_state != State.SLEEPING:
                self._fsm.transition_to(State.SLEEPING, now)
                self._publish_expression(self._expression_for_state(State.SLEEPING))
            return

        with self._lock:
            face_age = now - self._last_face_seen_at

        # Estado deseado en función de presencia
        if face_age < self._no_face_to_idle_s:
            desired = State.ENGAGED
        elif face_age < self._no_face_to_drowsy_s:
            desired = State.IDLE
        elif face_age < self._no_face_to_sleeping_s:
            desired = State.DROWSY
        else:
            desired = State.SLEEPING

        # Si estamos dentro de un "hold" de expresión expresiva (saludo,
        # pregunta, micro-expr), no pisamos la cara con la base del estado.
        prev_resting = self._fsm.is_resting()
        held = now < self._expression_hold_until
        if self._fsm.transition_to(desired, now):
            if not held:
                self._publish_expression(self._expression_for_state(desired))
            self._reschedule_randoms(now)

        if desired == State.ENGAGED and prev_resting:
            self._maybe_greet_returning(now, face_age)

    def _maybe_greet_returning(self, now: float, face_age: float) -> None:
        # face_age es el tiempo desde la última cara; usamos tiempo desde drowsy/sleep
        time_since_face = face_age
        if time_since_face >= self._greet_efusivo_thr and not self._persistence.greeting_efusivo_in_cooldown(
            self._greet_efusivo_cooldown, now
        ):
            self._publish_expression(self._expr_greet_efusivo)
            self._say("greeting_efusivo")
            self._persistence.mark_greeting_efusivo(now)
            self._expression_hold_until = now + 5.0
        elif time_since_face >= self._greet_casual_thr and not self._persistence.greeting_casual_in_cooldown(
            self._greet_casual_cooldown, now
        ):
            self._publish_expression(self._expr_greet_casual)
            self._say("greeting_casual")
            self._persistence.mark_greeting_casual(now)
            self._expression_hold_until = now + 4.0

    def _expression_for_state(self, state: State) -> str:
        return {
            State.ENGAGED: self._expr_engaged,
            State.IDLE: self._expr_idle,
            State.DROWSY: self._expr_drowsy,
            State.SLEEPING: self._expr_sleeping,
            State.WORKING: self._expr_working,
            State.WAKING_UP: self._expr_engaged,
        }.get(state, self._expr_engaged)

    def _reschedule_randoms(self, now: float) -> None:
        self._next_engaged_expr_at = now + random.uniform(self._eng_expr_min, self._eng_expr_max)
        self._next_engaged_motion_at = now + random.uniform(self._eng_motion_min, self._eng_motion_max)
        self._next_engaged_question_at = now + random.uniform(self._eng_q_min, self._eng_q_max)
        self._next_idle_look_at = now + random.uniform(self._idle_look_min, self._idle_look_max)
        self._next_idle_sigh_at = now + random.uniform(self._idle_sigh_min, self._idle_sigh_max)

    # ─────────────────── Random behaviors ───────────────────

    def _maybe_engaged_micro_expr(self, now: float) -> None:
        if now < self._next_engaged_expr_at:
            return
        self._next_engaged_expr_at = now + random.uniform(self._eng_expr_min, self._eng_expr_max)
        if random.random() > self._eng_expr_prob:
            return
        if not self._eng_expr_pool:
            return
        expr = random.choice(self._eng_expr_pool)
        base = self._expression_for_state(State.ENGAGED)
        dur = max(0.2, self._eng_expr_dur)
        self._publish_expression(expr)
        # Hold para que ningún transition pise la expresión durante la frase.
        self._expression_hold_until = now + dur
        if random.random() < self._micro_expr_tts_prob:
            self._say("micro_expr")

        def _restore() -> None:
            time.sleep(dur)
            if self._fsm.current_state == State.ENGAGED:
                self._publish_expression(base)

        threading.Thread(target=_restore, name="micro-expr-restore", daemon=True).start()

    def _maybe_engaged_question(self, now: float) -> None:
        """Después de un tiempo viendo personas, lanza una pregunta proactiva
        con una expresión coherente. Se siente más vivo que solo micro-gestos.
        """
        if now < self._next_engaged_question_at:
            return
        self._next_engaged_question_at = now + random.uniform(self._eng_q_min, self._eng_q_max)
        if random.random() > self._eng_q_prob:
            return
        if not self._tts.has("engaged_questions"):
            return
        expr_pool = self._eng_q_expr_pool or [self._expr_engaged]
        expr = random.choice(expr_pool)
        base = self._expression_for_state(State.ENGAGED)
        dur = max(1.0, self._eng_q_dur)
        self._publish_expression(expr)
        self._expression_hold_until = now + dur
        self._say("engaged_questions")

        def _restore() -> None:
            time.sleep(dur)
            if self._fsm.current_state == State.ENGAGED:
                self._publish_expression(base)

        threading.Thread(target=_restore, name="engaged-question-restore", daemon=True).start()

    def _maybe_engaged_micro_motion(self, now: float) -> None:
        if now < self._next_engaged_motion_at:
            return
        self._next_engaged_motion_at = now + random.uniform(self._eng_motion_min, self._eng_motion_max)
        if random.random() > self._eng_motion_prob:
            return
        if not self._eng_motion_pool or self._motor.busy:
            return
        gesture = random.choice(self._eng_motion_pool)
        if gesture == "mini_spin_left":
            self._motor.mini_spin_left()
        elif gesture == "mini_spin_right":
            self._motor.mini_spin_right()
        elif gesture == "tilt_random":
            self._motor.tilt_random()
        elif gesture == "tiny_advance":
            self._motor.tiny_advance(0.03)

    def _maybe_idle_lookaround(self, now: float) -> None:
        if now < self._next_idle_look_at:
            return
        self._next_idle_look_at = now + random.uniform(self._idle_look_min, self._idle_look_max)
        if self._motor.busy:
            return
        self._motor.tilt_random()

    def _maybe_idle_sigh(self, now: float) -> None:
        if now < self._next_idle_sigh_at:
            return
        self._next_idle_sigh_at = now + random.uniform(self._idle_sigh_min, self._idle_sigh_max)
        if random.random() < self._idle_sigh_tts_prob:
            self._say("idle_sigh")

    # ─────────────────── Servicios ───────────────────

    def _on_get_state_srv(self, _request, response):
        snap = self._triggers.snapshot()
        payload = {
            "state": self._fsm.current_state.value,
            "time_in_state_s": round(self._fsm.time_in_state(self._now()), 2),
            "speaking_active": self._speaking_active,
            "random_pause_remaining_s": round(max(0.0, self._random_pause_until - self._now()), 2),
            "triggers": snap,
            "priorities": {t.value: PRIORITIES[t] for t in PRIORITIES},
        }
        response.success = True
        response.message = json.dumps(payload)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PresenceOrchestrator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
