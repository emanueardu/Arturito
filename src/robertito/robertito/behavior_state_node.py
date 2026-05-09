"""Behavior state node — publica /behavior/state.

NOTA: Desde el refactor de orchestrator (v2) este nodo NO publica más:
    /head/tilt    → ahora dueño: presence_orchestrator + eyes_node
    /cmd_vel      → ahora dueño: presence_orchestrator
    /robertito/eyes_expression → ahora dueño: presence_orchestrator
La FSM interna se mantiene para backward-compat de /behavior/state.
Las llamadas a self._tilt_pub.publish() y self._cmd_pub.publish() son
no-op (los pubs son None) por diseño.
"""
import json
import os
import random
import socket
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from vision_msgs.msg import Detection2DArray

from robertito.family_companion import (
    build_family_morning_greeting,
    fetch_weather_snapshot,
    render_due_reminder,
)
from robertito.useful_memory import UsefulMemoryStore


def _expand_path(path: str) -> str:
    return os.path.expanduser((path or "").strip())


def _parse_datetime(value: Any) -> Optional[datetime]:
    if not isinstance(value, str) or not value.strip():
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def _format_datetime(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


class _SafeFormatDict(dict):
    def __missing__(self, key: str) -> str:
        return ""


@dataclass
class PendingProactiveEvent:
    category: str
    not_before_monotonic: float
    created_monotonic: float
    context: dict[str, str]


@dataclass
class HealthSnapshot:
    status: str = "unknown"
    last_update_monotonic: float = 0.0
    seen_once: bool = False


class BehaviorStateNode(Node):
    _SUPPORTED_EXPRESSIONS = {
        "neutral_soft",
        "neutral_soft_a",
        "neutral_soft_b",
        "neutral_soft_c",
        "attentive",
        "relaxed",
        "curious",
        "curious_a",
        "curious_b",
        "curious_c",
        "happy_soft",
        "happy_soft_a",
        "happy_soft_b",
        "happy_soft_c",
        "greeting",
        "pleased",
        "shy_happy",
        "listening",
        "listening_a",
        "listening_b",
        "listening_c",
        "thinking_left",
        "thinking_right",
        "processing",
        "skeptical",
        "confused_soft",
        "sleepy_soft",
        "drowsy",
        "sleeping_breath",
        "blink_soft",
        "glance_left",
        "glance_right",
        "ack",
        "micro_wink_left",
        "micro_wink_right",
        "normal",
        "happy",
        "focus",
        "surprised",
        "wink_left",
        "wink_right",
        "blink",
        "angry",
        "confused",
        "hearts",
        "sad",
        "sleeping",
        "sleeping_alt",
        "sleepy",
    }

    def __init__(self) -> None:
        super().__init__("behavior_state_node")

        self._enabled = bool(self.declare_parameter("enabled", True).value)
        self._expression_topic = str(
            self.declare_parameter("expression_topic", "robertito/eyes_expression").value
        )
        self._tilt_topic = str(self.declare_parameter("tilt_topic", "/head/tilt").value)
        self._cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/cmd_vel").value)
        self._state_topic = str(self.declare_parameter("state_topic", "/behavior/state").value)
        self._error_reason_topic = str(
            self.declare_parameter("error_reason_topic", "/behavior/error_reason").value
        )
        self._tts_topic = str(
            self.declare_parameter("proactive_tts_topic", "/assistant/say").value
        )

        self._presence_active_topic = str(
            self.declare_parameter("presence_active_topic", "/behavior/presence_active").value
        )
        self._greeting_active_topic = str(
            self.declare_parameter("greeting_active_topic", "/behavior/greeting_active").value
        )
        self._listening_active_topic = str(
            self.declare_parameter("listening_active_topic", "/behavior/listening_active").value
        )
        self._thinking_active_topic = str(
            self.declare_parameter("thinking_active_topic", "/behavior/thinking_active").value
        )
        self._speaking_active_topic = str(
            self.declare_parameter("speaking_active_topic", "/behavior/speaking_active").value
        )
        self._wake_health_topic = str(
            self.declare_parameter("wake_health_topic", "/behavior/health/wake_word").value
        )
        self._voice_health_topic = str(
            self.declare_parameter("voice_health_topic", "/behavior/health/voice").value
        )
        self._light_level_topic = str(
            self.declare_parameter("light_level_topic", "arturito/camera/brightness").value
        )
        self._detection_topic = str(
            self.declare_parameter("detection_topic", "arturito/camera/faces").value
        )
        self._listening_timeout_topic = str(
            self.declare_parameter("listening_timeout_topic", "/behavior/listening_timeout").value
        )
        self._daily_greeting_event_topic = str(
            self.declare_parameter(
                "daily_greeting_event_topic", "/behavior/daily_greeting_emitted"
            ).value
        )

        self._presence_visible_hold_sec = max(
            0.5, float(self.declare_parameter("presence_visible_hold_sec", 3.5).value)
        )
        self._state_transition_cooldown_sec = max(
            0.0, float(self.declare_parameter("state_transition_cooldown_sec", 0.25).value)
        )
        self._greeting_release_hold_sec = max(
            0.0, float(self.declare_parameter("greeting_release_hold_sec", 0.8).value)
        )
        self._thinking_release_hold_sec = max(
            0.0, float(self.declare_parameter("thinking_release_hold_sec", 0.4).value)
        )
        self._good_night_hold_sec = max(
            0.2, float(self.declare_parameter("good_night_hold_sec", 2.4).value)
        )
        self._good_night_message_enabled = bool(
            self.declare_parameter("good_night_message_enabled", True).value
        )
        self._good_night_message = str(
            self.declare_parameter("good_night_message", "Buenas noches").value
        ).strip()
        self._darkness_behavior_enabled = bool(
            self.declare_parameter("darkness_behavior_enabled", True).value
        )
        self._darkness_debounce_sec = max(
            0.0, float(self.declare_parameter("darkness_debounce_sec", 1.0).value)
        )
        self._occlusion_recovery_sec = max(
            0.0, float(self.declare_parameter("occlusion_recovery_sec", 2.5).value)
        )
        self._daytime_occlusion_cooldown_sec = max(
            0.0, float(self.declare_parameter("daytime_occlusion_cooldown_sec", 180.0).value)
        )
        self._nighttime_good_night_cooldown_sec = max(
            0.0, float(self.declare_parameter("nighttime_good_night_cooldown_sec", 1800.0).value)
        )
        night_phrases = self.declare_parameter(
            "night_phrase_pool", [self._good_night_message or "Buenas noches"]
        ).value
        if not isinstance(night_phrases, (list, tuple)):
            night_phrases = []
        self._night_phrase_pool = [
            str(item).strip() for item in night_phrases if str(item).strip()
        ]
        day_phrases = self.declare_parameter(
            "daytime_occlusion_phrase_pool",
            [
                "¿Quién está ahí?",
                "Salí de adelante mío.",
                "Ehh, me tapaste.",
                "No veo nada.",
                "Veo oscuridad.",
                "Corréte un poquito.",
                "¿Quién me está tapando?",
                "Che, no veo.",
            ],
        ).value
        if not isinstance(day_phrases, (list, tuple)):
            day_phrases = []
        self._daytime_occlusion_phrase_pool = [
            str(item).strip() for item in day_phrases if str(item).strip()
        ]

        self._neutral_tilt_deg = float(self.declare_parameter("neutral_tilt_deg", 0.0).value)
        self._idle_tilt_deg = float(self.declare_parameter("idle_tilt_deg", 0.0).value)
        self._attentive_tilt_deg = float(
            self.declare_parameter("attentive_tilt_deg", -1.5).value
        )
        self._listening_tilt_deg = float(
            self.declare_parameter("listening_tilt_deg", -2.0).value
        )
        self._thinking_tilt_deg = float(
            self.declare_parameter("thinking_tilt_deg", -1.0).value
        )
        self._speaking_tilt_deg = float(
            self.declare_parameter("speaking_tilt_deg", -0.8).value
        )
        self._greeting_tilt_deg = float(
            self.declare_parameter("greeting_tilt_deg", -2.5).value
        )
        self._sleeping_tilt_deg = float(
            self.declare_parameter("sleeping_tilt_deg", 1.5).value
        )
        self._good_night_tilt_deg = float(
            self.declare_parameter("good_night_tilt_deg", 2.0).value
        )

        raw_priority = self.declare_parameter(
            "priority_order",
            [
                "error",
                "greeting",
                "speaking",
                "thinking",
                "listening",
                "good_night",
                "sleeping",
                "attentive",
                "idle",
            ],
        ).value
        if not isinstance(raw_priority, (list, tuple)):
            raw_priority = [
                "error",
                "greeting",
                "speaking",
                "thinking",
                "listening",
                "good_night",
                "sleeping",
                "attentive",
                "idle",
            ]
        self._priority_order = [str(item).strip().lower() for item in raw_priority if str(item).strip()]
        if not self._priority_order:
            self._priority_order = [
                "error",
                "greeting",
                "speaking",
                "thinking",
                "listening",
                "good_night",
                "sleeping",
                "attentive",
                "idle",
            ]

        self._idle_micro_motion_enabled = bool(
            self.declare_parameter("idle_micro_motion_enabled", True).value
        )
        self._blink_enabled = bool(self.declare_parameter("blink_enabled", True).value)
        self._blink_interval_min_sec = max(
            1.0, float(self.declare_parameter("blink_interval_min_sec", 5.5).value)
        )
        self._blink_interval_max_sec = max(
            self._blink_interval_min_sec,
            float(self.declare_parameter("blink_interval_max_sec", 11.0).value),
        )
        self._blink_duration_sec = max(
            0.05, float(self.declare_parameter("blink_duration_sec", 0.14).value)
        )
        self._idle_min_interval_sec = max(
            4.0, float(self.declare_parameter("idle_min_interval_sec", 18.0).value)
        )
        self._idle_max_interval_sec = max(
            self._idle_min_interval_sec,
            float(self.declare_parameter("idle_max_interval_sec", 42.0).value),
        )
        self._night_idle_interval_scale = max(
            1.0, float(self.declare_parameter("night_idle_interval_scale", 1.8).value)
        )
        self._idle_expression_variation_probability = min(
            1.0,
            max(
                0.0,
                float(self.declare_parameter("idle_expression_variation_probability", 0.35).value),
            ),
        )
        self._idle_tilt_probability = min(
            1.0, max(0.0, float(self.declare_parameter("idle_tilt_probability", 0.22).value))
        )
        self._idle_attention_shift_probability = min(
            1.0,
            max(
                0.0,
                float(self.declare_parameter("idle_attention_shift_probability", 0.10).value),
            ),
        )
        self._idle_tilt_amplitude_min_deg = float(
            self.declare_parameter("idle_tilt_amplitude_min_deg", -2.0).value
        )
        self._idle_tilt_amplitude_max_deg = float(
            self.declare_parameter("idle_tilt_amplitude_max_deg", 2.0).value
        )
        self._idle_tilt_hold_sec = max(
            0.2, float(self.declare_parameter("idle_tilt_hold_sec", 0.9).value)
        )
        self._allow_idle_rotational_shift = bool(
            self.declare_parameter("allow_idle_rotational_shift", False).value
        )
        self._idle_rotation_speed_rad_s = max(
            0.05, abs(float(self.declare_parameter("idle_rotation_speed_rad_s", 0.16).value))
        )
        self._idle_rotation_duration_sec = max(
            0.05, float(self.declare_parameter("idle_rotation_duration_sec", 0.22).value)
        )
        self._idle_rotation_pause_sec = max(
            0.0, float(self.declare_parameter("idle_rotation_pause_sec", 0.10).value)
        )

        raw_idle_pool = self.declare_parameter(
            "idle_expression_pool", ["normal", "focus", "sleepy"]
        ).value
        raw_sleep_pool = self.declare_parameter(
            "sleeping_expression_pool", ["sleepy_soft", "drowsy", "sleeping_breath"]
        ).value
        self._idle_expression_pool = self._sanitize_expression_pool(raw_idle_pool, ["normal"])
        self._sleeping_expression_pool = self._sanitize_expression_pool(
            raw_sleep_pool, ["sleepy_soft"]
        )

        self._idle_expression = str(
            self.declare_parameter("idle_expression", "normal").value
        ).strip().lower()
        self._attentive_expression = str(
            self.declare_parameter("attentive_expression", "focus").value
        ).strip().lower()
        self._listening_expression = str(
            self.declare_parameter("listening_expression", "focus").value
        ).strip().lower()
        self._thinking_expression = str(
            self.declare_parameter("thinking_expression", "confused").value
        ).strip().lower()
        self._speaking_expression = str(
            self.declare_parameter("speaking_expression", "happy").value
        ).strip().lower()
        self._greeting_expression = str(
            self.declare_parameter("greeting_expression", "happy").value
        ).strip().lower()
        self._sleeping_expression = str(
            self.declare_parameter("sleeping_expression", "sleepy_soft").value
        ).strip().lower()
        self._good_night_expression = str(
            self.declare_parameter("good_night_expression", "sleeping_breath").value
        ).strip().lower()

        self._night_start_hour = int(self.declare_parameter("night_start_hour", 22).value)
        self._night_end_hour = int(self.declare_parameter("night_end_hour", 6).value)
        self._use_light_night_override = bool(
            self.declare_parameter("use_light_night_override", True).value
        )
        self._light_dark_threshold = float(
            self.declare_parameter("light_dark_threshold", 5.0).value
        )

        self._error_expression = str(
            self.declare_parameter("error_expression", "confused").value
        ).strip().lower()
        self._health_timeout_sec = max(
            1.0, float(self.declare_parameter("health_timeout_sec", 12.0).value)
        )
        self._health_startup_grace_sec = max(
            0.0, float(self.declare_parameter("health_startup_grace_sec", 20.0).value)
        )

        self._network_check_enabled = bool(
            self.declare_parameter("network_check_enabled", True).value
        )
        self._network_check_host = str(
            self.declare_parameter("network_check_host", "1.1.1.1").value
        )
        self._network_check_port = int(self.declare_parameter("network_check_port", 53).value)
        self._network_check_timeout_sec = max(
            0.1, float(self.declare_parameter("network_check_timeout_sec", 1.5).value)
        )
        self._network_check_interval_sec = max(
            5.0, float(self.declare_parameter("network_check_interval_sec", 30.0).value)
        )

        self._state_tick_sec = max(0.05, float(self.declare_parameter("state_tick_sec", 0.1).value))
        self._verbose_logging = bool(self.declare_parameter("verbose_logging", False).value)

        self._proactive_enabled = bool(self.declare_parameter("proactive_enabled", True).value)
        self._proactive_state_file_path = _expand_path(
            str(
                self.declare_parameter(
                    "proactive_state_file_path", "~/.ros/robertito_proactive_state.json"
                ).value
            )
        )
        self._proactive_global_cooldown_sec = max(
            0.0, float(self.declare_parameter("proactive_global_cooldown_sec", 300.0).value)
        )
        self._proactive_boot_delay_sec = max(
            0.0, float(self.declare_parameter("proactive_boot_delay_sec", 12.0).value)
        )
        self._proactive_idle_return_min_absence_sec = max(
            60.0,
            float(self.declare_parameter("proactive_idle_return_min_absence_sec", 2700.0).value),
        )
        self._proactive_presence_prompt_delay_sec = max(
            0.0, float(self.declare_parameter("proactive_presence_prompt_delay_sec", 4.0).value)
        )
        self._proactive_listening_timeout_delay_sec = max(
            0.0, float(self.declare_parameter("proactive_listening_timeout_delay_sec", 1.2).value)
        )
        self._proactive_recovery_delay_sec = max(
            0.0, float(self.declare_parameter("proactive_recovery_delay_sec", 2.0).value)
        )
        self._proactive_system_ok_min_uptime_sec = max(
            300.0, float(self.declare_parameter("proactive_system_ok_min_uptime_sec", 21600.0).value)
        )
        self._proactive_daily_greeting_block_sec = max(
            0.0, float(self.declare_parameter("proactive_daily_greeting_block_sec", 20.0).value)
        )
        self._proactive_allow_weather_placeholders = bool(
            self.declare_parameter("proactive_allow_weather_placeholders", False).value
        )
        self._family_companion_enabled = bool(
            self.declare_parameter("family_companion_enabled", True).value
        )
        self._quiet_hours_start_hour = int(
            self.declare_parameter("quiet_hours_start_hour", 22).value
        )
        self._quiet_hours_end_hour = int(
            self.declare_parameter("quiet_hours_end_hour", 7).value
        )
        self._proactive_window_sec = max(
            60.0, float(self.declare_parameter("proactive_window_sec", 3600.0).value)
        )
        self._proactive_max_comments_per_window = max(
            1, int(self.declare_parameter("proactive_max_comments_per_window", 3).value)
        )
        self._family_morning_enabled = bool(
            self.declare_parameter("family_morning_enabled", True).value
        )
        self._family_morning_start_hour = int(
            self.declare_parameter("family_morning_start_hour", 6).value
        )
        self._family_morning_end_hour = int(
            self.declare_parameter("family_morning_end_hour", 11).value
        )
        raw_family_morning_greetings = self.declare_parameter(
            "family_morning_greetings",
            ["Buen día.", "Buen día, che.", "Hola, buen día."],
        ).value
        if not isinstance(raw_family_morning_greetings, (list, tuple)):
            raw_family_morning_greetings = ["Buen día."]
        self._family_morning_greetings = [
            str(item).strip() for item in raw_family_morning_greetings if str(item).strip()
        ] or ["Buen día."]
        self._weather_enabled = bool(self.declare_parameter("weather_enabled", False).value)
        self._weather_lat = float(self.declare_parameter("weather_lat", 0.0).value)
        self._weather_lon = float(self.declare_parameter("weather_lon", 0.0).value)
        self._weather_location = str(self.declare_parameter("weather_location", "").value).strip()
        self._weather_unit = str(self.declare_parameter("weather_unit", "celsius").value).strip()
        self._weather_cache_minutes = float(
            self.declare_parameter("weather_cache_minutes", 20.0).value
        )
        self._weather_timeout_sec = float(
            self.declare_parameter("weather_timeout_sec", 4.0).value
        )
        self._weather_timezone = str(
            self.declare_parameter("weather_timezone", "auto").value
        ).strip() or "auto"
        self._family_reminders_enabled = bool(
            self.declare_parameter("family_reminders_enabled", True).value
        )
        self._family_reminder_store_path = _expand_path(
            str(
                self.declare_parameter(
                    "memory_store_path", "~/.ros/robertito_memory_store.json"
                ).value
            )
        )
        self._family_reminder_check_interval_sec = max(
            1.0, float(self.declare_parameter("reminder_check_interval_sec", 10.0).value)
        )
        self._family_reminder_repeat_interval_sec = max(
            0.0, float(self.declare_parameter("due_reminder_repeat_interval_sec", 0.0).value)
        )
        self._family_max_due_announcements = max(
            1, int(self.declare_parameter("max_due_announcements", 1).value)
        )
        self._allow_reminders_in_quiet_hours = bool(
            self.declare_parameter("allow_reminders_in_quiet_hours", False).value
        )
        self._family_user_name = str(self.declare_parameter("user_name", "Che").value).strip()
        raw_due_templates = self.declare_parameter(
            "due_reminder_templates",
            [
                "Acordate de {text}.",
                "{user_name}, acordate de {text}.",
                "Che, no te olvides de {text}.",
            ],
        ).value
        if not isinstance(raw_due_templates, (list, tuple)):
            raw_due_templates = ["Acordate de {text}."]
        self._family_due_reminder_templates = [
            str(item).strip() for item in raw_due_templates if str(item).strip()
        ] or ["Acordate de {text}."]

        self._category_names = [
            "boot_ready",
            "audio_recovered",
            "network_recovered",
            "first_person_today",
            "family_morning_greeting",
            "idle_return",
            "listening_timeout",
            "evening_notice",
            "system_ok",
            "reminder_due",
        ]
        self._category_enabled: dict[str, bool] = {}
        self._category_cooldowns: dict[str, float] = {}
        self._category_daily_max: dict[str, int] = {}
        self._category_phrases: dict[str, list[str]] = {}
        for category in self._category_names:
            self._category_enabled[category] = bool(
                self.declare_parameter(f"proactive_categories.{category}.enabled", True).value
            )
            self._category_cooldowns[category] = max(
                0.0,
                float(
                    self.declare_parameter(
                        f"proactive_categories.{category}.cooldown_sec", 1800.0
                    ).value
                ),
            )
            self._category_daily_max[category] = max(
                0,
                int(
                    self.declare_parameter(
                        f"proactive_categories.{category}.daily_max", 1
                    ).value
                ),
            )
            phrases = self.declare_parameter(
                f"proactive_categories.{category}.phrases", [""]
            ).value
            if not isinstance(phrases, (list, tuple)):
                phrases = []
            self._category_phrases[category] = [
                str(item).strip() for item in phrases if str(item).strip()
            ]

        # MOTOR PUBLISHERS REMOVIDOS — esos topics son responsabilidad de
        # presence_orchestrator_node y eyes_node. behavior_state solo publica /behavior/state.
        self._expr_pub = None
        self._tilt_pub = None
        self._cmd_pub = None
        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        self._error_reason_pub = self.create_publisher(String, self._error_reason_topic, 10)
        self._tts_pub = self.create_publisher(String, self._tts_topic, 10)
        self._daily_greeting_event_pub = self.create_publisher(
            Bool, self._daily_greeting_event_topic, 10
        )

        self.create_subscription(Bool, self._presence_active_topic, self._on_presence_active, 10)
        self.create_subscription(Bool, self._greeting_active_topic, self._on_greeting_active, 10)
        self.create_subscription(Bool, self._listening_active_topic, self._on_listening_active, 10)
        self.create_subscription(Bool, self._thinking_active_topic, self._on_thinking_active, 10)
        self.create_subscription(Bool, self._speaking_active_topic, self._on_speaking_active, 10)
        self.create_subscription(String, self._wake_health_topic, self._on_wake_health, 10)
        self.create_subscription(String, self._voice_health_topic, self._on_voice_health, 10)
        self.create_subscription(Float32, self._light_level_topic, self._on_light_level, 10)
        self.create_subscription(
            Detection2DArray, self._detection_topic, self._on_presence_detections, 10
        )
        self.create_subscription(Bool, self._listening_timeout_topic, self._on_listening_timeout, 10)
        self.create_subscription(Bool, self._daily_greeting_event_topic, self._on_daily_greeting, 10)

        self._boot_monotonic = time.monotonic()
        self._presence_active = False
        self._greeting_active = False
        self._listening_active = False
        self._thinking_active = False
        self._speaking_active = False
        self._light_level: Optional[float] = None
        self._raw_dark_active = False
        self._raw_dark_changed_monotonic = self._boot_monotonic
        self._darkness_episode_active = False
        self._darkness_episode_mode = ""
        self._last_daytime_occlusion_monotonic = -1.0e9
        self._last_nighttime_darkness_monotonic = -1.0e9
        self._night_override_active = False
        self._last_person_seen_monotonic = 0.0
        self._last_presence_end_monotonic = self._boot_monotonic
        self._last_presence_start_monotonic: Optional[float] = None
        self._last_daily_greeting_monotonic = 0.0
        self._last_evening_notice_date: Optional[str] = None
        self._pending_proactive_events: list[PendingProactiveEvent] = []
        self._weather_cache: dict[str, Any] = {}
        self._last_family_reminder_check_monotonic = self._boot_monotonic
        self._memory_store = UsefulMemoryStore(self._family_reminder_store_path)

        self._state = "idle"
        self._last_state_change_monotonic = self._boot_monotonic
        self._last_published_state: Optional[str] = None
        self._last_error_reason = ""
        self._last_expression: Optional[str] = None
        self._last_tilt_command: Optional[float] = None
        self._next_idle_action_at = 0.0
        self._next_blink_at = 0.0
        self._idle_action_lock = threading.Lock()
        self._idle_action_active = False
        self._blink_lock = threading.Lock()
        self._blink_active = False
        self._last_sleep_target = False
        self._good_night_until = 0.0
        self._good_night_announced = False
        self._greeting_release_until = 0.0
        self._thinking_release_until = 0.0

        self._wake_health = HealthSnapshot()
        self._voice_health = HealthSnapshot()
        self._network_health = HealthSnapshot(status="unknown")
        self._last_network_check_monotonic = 0.0
        self._network_available: Optional[bool] = None

        self._boot_ready_announced = False
        self._system_ok_announced_today: Optional[str] = None
        self._load_proactive_state()
        now = time.monotonic()
        self._schedule_next_idle_action(now)
        self._schedule_next_blink(now)

        self._timer = self.create_timer(self._state_tick_sec, self._on_tick)
        self.get_logger().info("Behavior state listo con Living Layer v1.")

    def _sanitize_expression_pool(self, values: Any, fallback: list[str]) -> list[str]:
        if not isinstance(values, (list, tuple)):
            return fallback
        cleaned: list[str] = []
        for value in values:
            item = str(value).strip().lower()
            if item and item in self._SUPPORTED_EXPRESSIONS and item not in cleaned:
                cleaned.append(item)
        return cleaned or fallback

    def _sanitize_expression(self, value: str, fallback: str) -> str:
        item = str(value or "").strip().lower()
        if item in self._SUPPORTED_EXPRESSIONS:
            return item
        return fallback

    def _on_presence_active(self, msg: Bool) -> None:
        now = time.monotonic()
        previous = self._presence_active
        self._presence_active = bool(msg.data)
        if self._presence_active and not previous:
            self._last_presence_start_monotonic = now
            absence_sec = max(0.0, now - self._last_presence_end_monotonic)
            if absence_sec >= self._proactive_idle_return_min_absence_sec:
                self._queue_proactive_event(
                    "idle_return",
                    delay_sec=self._proactive_presence_prompt_delay_sec,
                    context={"time_of_day": self._time_of_day_label()},
                )
            if self._should_queue_family_morning_greeting():
                self._queue_proactive_event(
                    "family_morning_greeting",
                    delay_sec=self._proactive_presence_prompt_delay_sec,
                    context={},
                )
        elif not self._presence_active and previous:
            self._last_presence_end_monotonic = now

    def _on_greeting_active(self, msg: Bool) -> None:
        now = time.monotonic()
        active = bool(msg.data)
        self._greeting_active = active
        if active:
            self._greeting_release_until = 0.0
        else:
            self._greeting_release_until = now + self._greeting_release_hold_sec

    def _on_listening_active(self, msg: Bool) -> None:
        self._listening_active = bool(msg.data)

    def _on_thinking_active(self, msg: Bool) -> None:
        now = time.monotonic()
        active = bool(msg.data)
        self._thinking_active = active
        if active:
            self._thinking_release_until = 0.0
        else:
            self._thinking_release_until = now + self._thinking_release_hold_sec

    def _on_speaking_active(self, msg: Bool) -> None:
        self._speaking_active = bool(msg.data)

    def _on_wake_health(self, msg: String) -> None:
        self._update_health_snapshot(self._wake_health, msg.data)

    def _on_voice_health(self, msg: String) -> None:
        was_seen_once = self._voice_health.seen_once
        previous_ok = self._is_snapshot_ok(self._voice_health)
        self._update_health_snapshot(self._voice_health, msg.data)
        current_ok = self._is_snapshot_ok(self._voice_health)
        if was_seen_once and not previous_ok and current_ok:
            self._queue_proactive_event(
                "audio_recovered",
                delay_sec=self._proactive_recovery_delay_sec,
                context={},
            )

    def _on_light_level(self, msg: Float32) -> None:
        self._light_level = float(msg.data)
        is_dark = self._light_level <= self._light_dark_threshold
        if is_dark != self._raw_dark_active:
            self._raw_dark_active = is_dark
            self._raw_dark_changed_monotonic = time.monotonic()

    def _on_presence_detections(self, msg: Detection2DArray) -> None:
        if msg.detections:
            self._last_person_seen_monotonic = time.monotonic()

    def _on_listening_timeout(self, msg: Bool) -> None:
        if not msg.data:
            return
        self._queue_proactive_event(
            "listening_timeout",
            delay_sec=self._proactive_listening_timeout_delay_sec,
            context={"time_of_day": self._time_of_day_label()},
        )

    def _on_daily_greeting(self, msg: Bool) -> None:
        if msg.data:
            self._last_daily_greeting_monotonic = time.monotonic()
            self._proactive_state["daily_greeting_seen_date"] = self._local_now().date().isoformat()
            self._save_proactive_state()

    def _update_health_snapshot(self, snapshot: HealthSnapshot, status: str) -> None:
        snapshot.status = (status or "").strip() or "unknown"
        snapshot.last_update_monotonic = time.monotonic()
        snapshot.seen_once = True

    def _is_snapshot_ok(self, snapshot: HealthSnapshot) -> bool:
        return snapshot.status.strip().lower() == "ok"

    def _on_tick(self) -> None:
        now = time.monotonic()
        self._update_network_health(now)
        self._update_darkness_behavior(now)
        self._update_sleep_transition(now)
        state, error_reason = self._compute_state(now)
        self._apply_state_transition(state, now)
        self._publish_state_if_needed(error_reason)
        self._publish_expression_for_state()
        self._publish_posture_for_state()
        self._run_idle_life(now)
        self._handle_evening_notice()
        self._handle_boot_ready(now)
        self._handle_system_ok(now)
        self._check_due_family_reminders(now)
        self._process_pending_proactive_events(now)

    def _apply_state_transition(self, state: str, now: float) -> None:
        if state == self._state:
            return
        if (now - self._last_state_change_monotonic) < self._state_transition_cooldown_sec:
            return
        self._state = state
        self._last_state_change_monotonic = now
        if self._verbose_logging:
            self.get_logger().info(f"Estado de comportamiento -> {self._state}")
        if self._state in ("idle", "attentive", "sleeping"):
            self._schedule_next_idle_action(now)
        if self._state in ("good_night", "sleeping"):
            self._schedule_next_blink(now, extend=True)

    def _compute_state(self, now: float) -> tuple[str, str]:
        error_reason = self._compute_error_reason(now)
        flags = {
            "error": error_reason != "",
            "greeting": self._greeting_active or now < self._greeting_release_until,
            "speaking": self._speaking_active,
            "thinking": self._thinking_active or now < self._thinking_release_until,
            "listening": self._listening_active,
            "good_night": now < self._good_night_until,
            "sleeping": self._is_sleeping_target(),
            "attentive": self._person_visible() or self._presence_active,
            "idle": True,
        }
        for candidate in self._priority_order:
            if candidate == "error" and error_reason:
                return "error", error_reason
            if candidate != "error" and flags.get(candidate, False):
                return candidate, error_reason
        return "idle", error_reason

    def _compute_error_reason(self, now: float) -> str:
        if (now - self._boot_monotonic) < self._health_startup_grace_sec:
            return ""
        reasons = []
        reason = self._health_reason("micrófono", self._wake_health, now)
        if reason:
            reasons.append(reason)
        reason = self._health_reason("audio/tts", self._voice_health, now)
        if reason:
            reasons.append(reason)
        if self._network_check_enabled:
            reason = self._health_reason("red", self._network_health, now)
            if reason:
                reasons.append(reason)
        return "; ".join(reasons)

    def _health_reason(self, label: str, snapshot: HealthSnapshot, now: float) -> str:
        if not snapshot.seen_once:
            return ""
        if snapshot.status.strip().lower() != "ok":
            return f"{label}: {snapshot.status}"
        if snapshot.last_update_monotonic <= 0.0:
            return f"{label}: sin latido"
        if (now - snapshot.last_update_monotonic) > self._health_timeout_sec:
            return f"{label}: sin actualización"
        return ""

    def _update_sleep_transition(self, now: float) -> None:
        sleep_target = self._is_sleeping_target()
        if sleep_target and not self._last_sleep_target:
            self._good_night_until = now + self._good_night_hold_sec
        elif not sleep_target:
            self._good_night_until = 0.0
        self._last_sleep_target = sleep_target

    def _is_sleeping_target(self) -> bool:
        if self._night_override_active:
            return True
        return self._is_hour_in_window(self._local_now().hour, self._night_start_hour, self._night_end_hour)

    @staticmethod
    def _is_hour_in_window(hour: int, start_hour: int, end_hour: int) -> bool:
        if start_hour == end_hour:
            return False
        if start_hour < end_hour:
            return start_hour <= hour < end_hour
        return hour >= start_hour or hour < end_hour

    def _is_darkness_night_window(self) -> bool:
        return self._is_hour_in_window(self._local_now().hour, self._night_start_hour, self._night_end_hour)

    def _update_darkness_behavior(self, now: float) -> None:
        if self._light_level is None:
            self._night_override_active = False
            return

        if not self._darkness_behavior_enabled:
            self._darkness_episode_active = False
            self._darkness_episode_mode = ""
            self._night_override_active = self._use_light_night_override and self._raw_dark_active
            return

        if self._darkness_episode_active:
            if self._raw_dark_active:
                self._night_override_active = (
                    self._use_light_night_override
                    and self._darkness_episode_mode == "night"
                    and self._is_darkness_night_window()
                )
                return
            if (now - self._raw_dark_changed_monotonic) < self._occlusion_recovery_sec:
                self._night_override_active = (
                    self._use_light_night_override
                    and self._darkness_episode_mode == "night"
                    and self._is_darkness_night_window()
                )
                return
            self._darkness_episode_active = False
            self._darkness_episode_mode = ""
            self._night_override_active = False
            return

        self._night_override_active = False
        if not self._raw_dark_active:
            return
        if (now - self._raw_dark_changed_monotonic) < self._darkness_debounce_sec:
            return

        self._darkness_episode_active = True
        self._darkness_episode_mode = "night" if self._is_darkness_night_window() else "day"
        self._night_override_active = (
            self._use_light_night_override and self._darkness_episode_mode == "night"
        )
        self._maybe_speak_for_darkness_episode(now, self._darkness_episode_mode)

    def _maybe_speak_for_darkness_episode(self, now: float, mode: str) -> None:
        if self._speaking_active or self._listening_active or self._state == "error":
            return

        if mode == "night":
            if not self._good_night_message_enabled:
                return
            if (now - self._last_nighttime_darkness_monotonic) < self._nighttime_good_night_cooldown_sec:
                return
            phrase = self._choose_darkness_phrase(self._night_phrase_pool)
            if not phrase:
                return
            if not self._can_emit_misc_proactive("darkness_night", allow_in_quiet_hours=True):
                return
            self._tts_pub.publish(String(data=f"[behavior_darkness] {phrase}"))
            self._last_nighttime_darkness_monotonic = now
            self._record_proactive_emit("darkness_night", phrase)
            return

        if (now - self._last_daytime_occlusion_monotonic) < self._daytime_occlusion_cooldown_sec:
            return
        phrase = self._choose_darkness_phrase(self._daytime_occlusion_phrase_pool)
        if not phrase:
            return
        if not self._can_emit_misc_proactive("darkness_day", allow_in_quiet_hours=False):
            return
        self._tts_pub.publish(String(data=f"[behavior_darkness] {phrase}"))
        self._last_daytime_occlusion_monotonic = now
        self._record_proactive_emit("darkness_day", phrase)

    @staticmethod
    def _choose_darkness_phrase(pool: list[str]) -> str:
        if not pool:
            return ""
        return random.choice(pool)

    def _person_visible(self) -> bool:
        return (
            self._last_person_seen_monotonic > 0.0
            and (time.monotonic() - self._last_person_seen_monotonic) <= self._presence_visible_hold_sec
        )

    def _publish_state_if_needed(self, error_reason: str) -> None:
        if self._last_published_state != self._state:
            self._state_pub.publish(String(data=self._state))
            self._last_published_state = self._state
        if error_reason != self._last_error_reason:
            self._error_reason_pub.publish(String(data=error_reason))
            self._last_error_reason = error_reason

    def _expression_for_state(self) -> str:
        if self._state == "error":
            return self._sanitize_expression(self._error_expression, "confused")
        if self._state == "greeting":
            return self._sanitize_expression(self._greeting_expression, "happy")
        if self._state == "speaking":
            return self._sanitize_expression(self._speaking_expression, "happy")
        if self._state == "thinking":
            return self._sanitize_expression(self._thinking_expression, "confused")
        if self._state == "listening":
            return self._sanitize_expression(self._listening_expression, "focus")
        if self._state == "good_night":
            return self._sanitize_expression(self._good_night_expression, "sleeping_breath")
        if self._state == "sleeping":
            return self._sanitize_expression(self._sleeping_expression, "sleepy_soft")
        if self._state == "attentive":
            return self._sanitize_expression(self._attentive_expression, "focus")
        return self._sanitize_expression(self._idle_expression, "normal")

    def _publish_expression_for_state(self) -> None:
        # Las expresiones las publica ahora presence_orchestrator_node.
        # Mantenemos last_expression actualizado para que la lógica interna
        # de variaciones siga decidiendo coherentemente.
        if self._blink_active:
            return
        expression = self._expression_for_state()
        if expression == self._last_expression:
            return
        self._last_expression = expression

    def _base_tilt_for_state(self) -> float:
        if self._state == "greeting":
            return self._greeting_tilt_deg
        if self._state == "speaking":
            return self._speaking_tilt_deg
        if self._state == "thinking":
            return self._thinking_tilt_deg
        if self._state == "listening":
            return self._listening_tilt_deg
        if self._state == "good_night":
            return self._good_night_tilt_deg
        if self._state == "sleeping":
            return self._sleeping_tilt_deg
        if self._state == "attentive":
            return self._attentive_tilt_deg
        if self._state == "idle":
            return self._idle_tilt_deg
        return self._neutral_tilt_deg

    def _publish_posture_for_state(self) -> None:
        if self._idle_action_active:
            return
        target = float(self._base_tilt_for_state())
        if self._last_tilt_command is not None and abs(self._last_tilt_command - target) < 0.05:
            return
        if self._tilt_pub is not None:
            self._tilt_pub.publish(Float32(data=target))
        self._last_tilt_command = target

    def _run_idle_life(self, now: float) -> None:
        if self._blink_enabled and not self._blink_active and now >= self._next_blink_at:
            self._start_blink()
            self._schedule_next_blink(now)

        if not self._idle_micro_motion_enabled:
            return
        if self._state not in ("idle", "attentive", "sleeping"):
            return
        if self._idle_action_active or now < self._next_idle_action_at:
            return

        if self._state == "sleeping":
            self._maybe_sleep_variation(now)
            return

        action = self._choose_idle_action()
        if action == "expression":
            self._apply_idle_expression_variation()
        elif action == "tilt":
            self._start_micro_tilt()
        elif action == "rotation":
            self._start_attention_shift()
        self._schedule_next_idle_action(now)

    def _choose_idle_action(self) -> str:
        options: list[tuple[str, float]] = []
        options.append(("expression", self._idle_expression_variation_probability))
        options.append(("tilt", self._idle_tilt_probability))
        if self._allow_idle_rotational_shift:
            options.append(("rotation", self._idle_attention_shift_probability))
        total = sum(weight for _, weight in options)
        if total <= 0.0:
            return "expression"
        pick = random.uniform(0.0, total)
        running = 0.0
        for name, weight in options:
            running += weight
            if pick <= running:
                return name
        return options[-1][0]

    def _apply_idle_expression_variation(self) -> None:
        # Las variaciones de expresión idle ahora las maneja
        # presence_orchestrator_node. Aquí solo actualizamos last_expression
        # para que el resto de la lógica interna no se confunda.
        expression = self._choose_expression(
            self._idle_expression_pool,
            exclude=self._last_expression,
            fallback=self._sanitize_expression(self._idle_expression, "normal"),
        )
        if expression != self._last_expression:
            self._last_expression = expression

    def _start_blink(self) -> None:
        # El parpadeo lo maneja eyes_node directamente; ya no publicamos
        # expresiones desde acá.
        return

    def _start_micro_tilt(self) -> None:
        if not self._idle_action_lock.acquire(blocking=False):
            return
        self._idle_action_active = True
        base = self._base_tilt_for_state()
        offset = random.uniform(
            self._idle_tilt_amplitude_min_deg, self._idle_tilt_amplitude_max_deg
        )
        target = base + offset

        def _run() -> None:
            try:
                if self._tilt_pub is not None:
                    self._tilt_pub.publish(Float32(data=float(target)))
                self._last_tilt_command = float(target)
                time.sleep(self._idle_tilt_hold_sec)
                restore = float(self._base_tilt_for_state())
                if self._tilt_pub is not None:
                    self._tilt_pub.publish(Float32(data=restore))
                self._last_tilt_command = restore
            finally:
                self._idle_action_active = False
                self._idle_action_lock.release()

        threading.Thread(target=_run, name="living-layer-micro-tilt", daemon=True).start()

    def _start_attention_shift(self) -> None:
        if not self._allow_idle_rotational_shift:
            return
        if not self._idle_action_lock.acquire(blocking=False):
            return
        self._idle_action_active = True

        def _publish_turn(speed: float, duration: float) -> None:
            msg = Twist()
            msg.angular.z = float(speed)
            if self._cmd_pub is not None:
                self._cmd_pub.publish(msg)
            time.sleep(duration)
            if self._cmd_pub is not None:
                self._cmd_pub.publish(Twist())

        def _run() -> None:
            try:
                _publish_turn(self._idle_rotation_speed_rad_s, self._idle_rotation_duration_sec)
                time.sleep(self._idle_rotation_pause_sec)
                _publish_turn(-self._idle_rotation_speed_rad_s, self._idle_rotation_duration_sec)
            finally:
                if self._cmd_pub is not None:
                    self._cmd_pub.publish(Twist())
                self._idle_action_active = False
                self._idle_action_lock.release()

        threading.Thread(target=_run, name="living-layer-attention-shift", daemon=True).start()

    def _maybe_sleep_variation(self, now: float) -> None:
        # Las variaciones de expresión durante sleep ahora las maneja
        # presence_orchestrator_node; mantenemos el scheduling para que la
        # FSM interna siga progresando idénticamente.
        if random.random() < 0.25:
            expression = self._choose_expression(
                self._sleeping_expression_pool,
                exclude=self._last_expression,
                fallback=self._sanitize_expression(self._sleeping_expression, "sleepy_soft"),
            )
            if expression != self._last_expression:
                self._last_expression = expression
        self._schedule_next_idle_action(now, sleeping=True)

    def _schedule_next_idle_action(self, now: float, sleeping: bool = False) -> None:
        interval = random.uniform(self._idle_min_interval_sec, self._idle_max_interval_sec)
        if sleeping or self._state in ("good_night", "sleeping"):
            interval *= self._night_idle_interval_scale
        self._next_idle_action_at = now + interval

    def _schedule_next_blink(self, now: float, extend: bool = False) -> None:
        interval = random.uniform(self._blink_interval_min_sec, self._blink_interval_max_sec)
        if extend or self._state in ("good_night", "sleeping"):
            interval *= self._night_idle_interval_scale
        self._next_blink_at = now + interval

    def _choose_expression(
        self, pool: list[str], *, exclude: Optional[str], fallback: str
    ) -> str:
        cleaned = [item for item in pool if item in self._SUPPORTED_EXPRESSIONS]
        if not cleaned:
            return fallback
        choices = [item for item in cleaned if item != exclude]
        if not choices:
            choices = cleaned
        return random.choice(choices)

    def _update_network_health(self, now: float) -> None:
        if not self._network_check_enabled:
            return
        if (now - self._last_network_check_monotonic) < self._network_check_interval_sec:
            return
        self._last_network_check_monotonic = now
        was_seen_once = self._network_health.seen_once
        previous_ok = self._is_snapshot_ok(self._network_health)
        ok, detail = self._check_network()
        self._network_health.status = "ok" if ok else f"error: {detail}"
        self._network_health.last_update_monotonic = now
        self._network_health.seen_once = True
        self._network_available = ok
        if was_seen_once and not previous_ok and ok:
            self._queue_proactive_event(
                "network_recovered",
                delay_sec=self._proactive_recovery_delay_sec,
                context={},
            )

    def _check_network(self) -> tuple[bool, str]:
        try:
            with socket.create_connection(
                (self._network_check_host, self._network_check_port),
                timeout=self._network_check_timeout_sec,
            ):
                return True, "ok"
        except OSError as exc:
            return False, str(exc)

    def _handle_boot_ready(self, now: float) -> None:
        if self._boot_ready_announced or not self._proactive_enabled:
            return
        if (now - self._boot_monotonic) < self._proactive_boot_delay_sec:
            return
        if not self._is_snapshot_ok(self._voice_health):
            return
        self._queue_proactive_event("boot_ready", delay_sec=0.0, context={})

    def _handle_evening_notice(self) -> None:
        if not self._proactive_enabled or self._state not in ("good_night", "sleeping"):
            return
        local_date = self._local_now().date().isoformat()
        if self._last_evening_notice_date == local_date:
            return
        self._queue_proactive_event(
            "evening_notice",
            delay_sec=self._proactive_presence_prompt_delay_sec,
            context={"time_of_day": self._time_of_day_label()},
        )

    def _handle_system_ok(self, now: float) -> None:
        if not self._proactive_enabled:
            return
        if (now - self._boot_monotonic) < self._proactive_system_ok_min_uptime_sec:
            return
        if self._state not in ("idle", "attentive", "sleeping"):
            return
        if self._compute_error_reason(now):
            return
        local_date = self._local_now().date().isoformat()
        if self._system_ok_announced_today == local_date:
            return
        self._queue_proactive_event(
            "system_ok",
            delay_sec=self._proactive_presence_prompt_delay_sec,
            context={"time_of_day": self._time_of_day_label()},
        )

    def _queue_proactive_event(
        self, category: str, *, delay_sec: float, context: dict[str, str]
    ) -> None:
        if not self._proactive_enabled:
            return
        if category not in self._category_names:
            return
        if not self._category_enabled.get(category, False):
            return
        if not self._category_phrases.get(category):
            return
        if any(event.category == category for event in self._pending_proactive_events):
            return
        now = time.monotonic()
        self._pending_proactive_events.append(
            PendingProactiveEvent(
                category=category,
                not_before_monotonic=now + max(0.0, delay_sec),
                created_monotonic=now,
                context=context,
            )
        )

    def _process_pending_proactive_events(self, now: float) -> None:
        if not self._pending_proactive_events:
            return
        ready = [
            event
            for event in self._pending_proactive_events
            if now >= event.not_before_monotonic
        ]
        self._pending_proactive_events = [
            event
            for event in self._pending_proactive_events
            if now < event.not_before_monotonic
        ]
        for event in sorted(ready, key=lambda item: item.not_before_monotonic):
            if self._emit_proactive_phrase(event.category, event.context, now):
                continue
            if (now - event.created_monotonic) < 30.0:
                self._pending_proactive_events.append(
                    PendingProactiveEvent(
                        category=event.category,
                        not_before_monotonic=now + 2.0,
                        created_monotonic=event.created_monotonic,
                        context=event.context,
                    )
                )

    def _emit_proactive_phrase(
        self, category: str, context: dict[str, str], now: float
    ) -> bool:
        if not self._can_emit_proactive(category, now):
            return False
        phrase = self._choose_phrase(category, context)
        if not phrase:
            return False
        self._tts_pub.publish(String(data=f"[behavior_proactive] {phrase}"))
        self._record_proactive_emit(category, phrase)
        if category == "boot_ready":
            self._boot_ready_announced = True
        elif category == "family_morning_greeting":
            self._last_daily_greeting_monotonic = now
            self._proactive_state["daily_greeting_seen_date"] = self._local_now().date().isoformat()
            self._daily_greeting_event_pub.publish(Bool(data=True))
        elif category == "evening_notice":
            self._last_evening_notice_date = self._local_now().date().isoformat()
        elif category == "system_ok":
            self._system_ok_announced_today = self._local_now().date().isoformat()
        if self._verbose_logging:
            self.get_logger().info(f"Proactivo {category}: {phrase}")
        return True

    def _can_emit_proactive(self, category: str, now: float) -> bool:
        if not self._proactive_enabled:
            return False
        if not self._category_enabled.get(category, False):
            return False
        if self._speaking_active:
            return False
        if self._listening_active and category != "listening_timeout":
            return False
        if category != "boot_ready" and self._state == "error":
            return False
        if self._is_in_quiet_hours():
            allowed_during_quiet = {"evening_notice"}
            if self._allow_reminders_in_quiet_hours:
                allowed_during_quiet.add("reminder_due")
            if category not in allowed_during_quiet:
                return False
        if (
            self._last_daily_greeting_monotonic > 0.0
            and (now - self._last_daily_greeting_monotonic) < self._proactive_daily_greeting_block_sec
        ):
            return False
        if category == "family_morning_greeting" and self._last_daily_greeting_monotonic > 0.0:
            return False
        last_global = _parse_datetime(self._proactive_state.get("last_global_at"))
        now_dt = datetime.now(timezone.utc)
        if last_global and (now_dt - last_global).total_seconds() < self._proactive_global_cooldown_sec:
            return False
        recent_events = self._recent_proactive_events(now_dt)
        if len(recent_events) >= self._proactive_max_comments_per_window:
            return False
        last_by_category = _parse_datetime(
            self._proactive_state.get("last_by_category", {}).get(category)
        )
        cooldown = self._category_cooldowns.get(category, 0.0)
        if last_by_category and (now_dt - last_by_category).total_seconds() < cooldown:
            return False
        daily_max = self._category_daily_max.get(category, 0)
        if daily_max > 0 and category != "boot_ready":
            local_date = self._local_now().date().isoformat()
            counts_date = self._proactive_state.get("daily_counts_date")
            counts = self._proactive_state.get("daily_counts", {})
            if counts_date == local_date and int(counts.get(category, 0)) >= daily_max:
                return False
        return True

    def _choose_phrase(self, category: str, context: dict[str, str]) -> str:
        if category == "family_morning_greeting":
            snapshot = self._get_weather_snapshot()
            return build_family_morning_greeting(
                greeting_pool=self._family_morning_greetings,
                snapshot=snapshot,
            ).strip()
        if category == "reminder_due":
            text = str(context.get("text") or "").strip()
            if not text:
                return ""
            return render_due_reminder(
                text=text,
                user_name=self._family_user_name or "Che",
                templates=self._family_due_reminder_templates,
            )
        pool = list(self._category_phrases.get(category, []))
        if not pool:
            return ""
        last_phrase = self._proactive_state.get("last_phrase_by_category", {}).get(category)
        last_global = self._proactive_state.get("last_global_phrase", "")
        candidates = [
            phrase for phrase in pool if phrase != last_phrase and phrase != last_global
        ]
        if not candidates:
            candidates = [phrase for phrase in pool if phrase != last_phrase]
        if not candidates:
            candidates = pool
        template = random.choice(candidates)
        render_context = {
            "time_of_day": self._time_of_day_label(),
            "temp_c": "",
            "weather_desc": "",
        }
        render_context.update(context)
        if not self._proactive_allow_weather_placeholders:
            render_context["temp_c"] = ""
            render_context["weather_desc"] = ""
        return template.format_map(_SafeFormatDict(render_context)).strip()

    def _record_proactive_emit(self, category: str, phrase: str) -> None:
        now = datetime.now(timezone.utc)
        local_date = self._local_now().date().isoformat()
        if self._proactive_state.get("daily_counts_date") != local_date:
            self._proactive_state["daily_counts_date"] = local_date
            self._proactive_state["daily_counts"] = {}
        daily_counts = self._proactive_state.setdefault("daily_counts", {})
        daily_counts[category] = int(daily_counts.get(category, 0)) + 1
        last_by_category = self._proactive_state.setdefault("last_by_category", {})
        last_phrase_by_category = self._proactive_state.setdefault("last_phrase_by_category", {})
        last_by_category[category] = _format_datetime(now)
        last_phrase_by_category[category] = phrase
        self._proactive_state["last_global_at"] = _format_datetime(now)
        self._proactive_state["last_global_phrase"] = phrase
        recent_events = self._proactive_state.setdefault("recent_emits", [])
        recent_events.append(_format_datetime(now))
        self._proactive_state["recent_emits"] = [
            timestamp
            for timestamp in recent_events
            if (
                (parsed := _parse_datetime(timestamp)) is not None
                and (now - parsed).total_seconds() <= self._proactive_window_sec
            )
        ]
        self._save_proactive_state()

    def _load_proactive_state(self) -> None:
        self._proactive_state: dict[str, Any] = {
            "last_global_at": None,
            "last_global_phrase": "",
            "last_by_category": {},
            "last_phrase_by_category": {},
            "daily_counts_date": "",
            "daily_counts": {},
            "daily_greeting_seen_date": "",
            "recent_emits": [],
        }
        path = self._proactive_state_file_path
        if not path or not os.path.exists(path):
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            if isinstance(payload, dict):
                self._proactive_state.update(payload)
        except Exception as exc:
            self.get_logger().warning(f"No se pudo cargar estado proactivo: {exc}")

    def _save_proactive_state(self) -> None:
        path = self._proactive_state_file_path
        if not path:
            return
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp_path = f"{path}.tmp"
        try:
            with open(tmp_path, "w", encoding="utf-8") as handle:
                json.dump(self._proactive_state, handle, ensure_ascii=False, indent=2)
            os.replace(tmp_path, path)
        except Exception as exc:
            self.get_logger().warning(f"No se pudo guardar estado proactivo: {exc}")

    def _local_now(self) -> datetime:
        return datetime.now().astimezone()

    def _time_of_day_label(self) -> str:
        hour = self._local_now().hour
        if 6 <= hour < 12:
            return "mañana"
        if 12 <= hour < 20:
            return "tarde"
        return "noche"

    def _should_queue_family_morning_greeting(self) -> bool:
        if not self._family_companion_enabled or not self._family_morning_enabled:
            return False
        now = self._local_now()
        if not self._is_hour_in_window(
            now.hour, self._family_morning_start_hour, self._family_morning_end_hour
        ):
            return False
        return self._proactive_state.get("daily_greeting_seen_date") != now.date().isoformat()

    def _is_in_quiet_hours(self) -> bool:
        return self._is_hour_in_window(
            self._local_now().hour,
            self._quiet_hours_start_hour,
            self._quiet_hours_end_hour,
        )

    def _recent_proactive_events(self, now_dt: datetime) -> list[str]:
        recent = []
        for timestamp in self._proactive_state.get("recent_emits", []):
            parsed = _parse_datetime(timestamp)
            if parsed is None:
                continue
            if (now_dt - parsed).total_seconds() <= self._proactive_window_sec:
                recent.append(timestamp)
        self._proactive_state["recent_emits"] = recent
        return recent

    def _can_emit_misc_proactive(self, category: str, *, allow_in_quiet_hours: bool) -> bool:
        now_dt = datetime.now(timezone.utc)
        if self._is_in_quiet_hours() and not allow_in_quiet_hours:
            return False
        recent_events = self._recent_proactive_events(now_dt)
        if len(recent_events) >= self._proactive_max_comments_per_window:
            return False
        last_by_category = _parse_datetime(
            self._proactive_state.get("last_by_category", {}).get(category)
        )
        if category == "darkness_day":
            cooldown = self._daytime_occlusion_cooldown_sec
        else:
            cooldown = self._nighttime_good_night_cooldown_sec
        return not (
            last_by_category and (now_dt - last_by_category).total_seconds() < cooldown
        )

    def _get_weather_snapshot(self) -> Optional[dict[str, Any]]:
        if not self._weather_enabled:
            return None
        fetched_at = self._weather_cache.get("fetched_at")
        if isinstance(fetched_at, (int, float)):
            if (time.time() - float(fetched_at)) <= (self._weather_cache_minutes * 60.0):
                return self._weather_cache
        snapshot = fetch_weather_snapshot(
            latitude=self._weather_lat,
            longitude=self._weather_lon,
            temperature_unit=self._weather_unit,
            timeout_sec=self._weather_timeout_sec,
            timezone_name=self._weather_timezone,
        )
        if snapshot is None:
            return None
        snapshot["fetched_at"] = time.time()
        self._weather_cache = snapshot
        return self._weather_cache

    def _check_due_family_reminders(self, now: float) -> None:
        if not self._family_companion_enabled or not self._family_reminders_enabled:
            return
        if (now - self._last_family_reminder_check_monotonic) < self._family_reminder_check_interval_sec:
            return
        self._last_family_reminder_check_monotonic = now
        self._memory_store.load()
        due_items = self._memory_store.due_reminders(
            now=self._local_now(),
            repeat_interval_sec=self._family_reminder_repeat_interval_sec,
            max_announcements=self._family_max_due_announcements,
        )
        for reminder in due_items:
            text = str(reminder.get("text") or "").strip()
            reminder_id = str(reminder.get("id") or "").strip()
            if not text or not reminder_id:
                continue
            if not self._emit_proactive_phrase("reminder_due", {"text": text}, now):
                continue
            self._memory_store.load()
            self._memory_store.mark_reminder_announced(reminder_id, when=self._local_now())

    def destroy_node(self) -> bool:
        if self._cmd_pub is not None:
            self._cmd_pub.publish(Twist())
        if self._tilt_pub is not None:
            self._tilt_pub.publish(Float32(data=float(self._neutral_tilt_deg)))
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BehaviorStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
