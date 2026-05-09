"""Detección de triggers de interrupción del orchestrator.

Cada trigger tiene una prioridad numérica fija. El orchestrator consulta los
triggers en orden descendente de prioridad y dispara el handler del primero
que esté activo.
"""
from __future__ import annotations

import datetime as _dt
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class Trigger(Enum):
    """Triggers que pueden interrumpir el ciclo principal."""

    CLIFF = "cliff"
    BUMP = "bump"
    STARTLED = "startled"
    SHAKEN = "shaken"
    WAKE = "wake"
    BEDTIME = "bedtime"
    DAILY_GREETING = "daily_greeting"
    SCHOOL_FAREWELL = "school_farewell"


PRIORITIES: dict[Trigger, int] = {
    Trigger.CLIFF: 100,
    Trigger.BUMP: 90,
    Trigger.STARTLED: 85,
    Trigger.SHAKEN: 80,
    Trigger.WAKE: 70,
    Trigger.BEDTIME: 50,
    Trigger.DAILY_GREETING: 50,
    Trigger.SCHOOL_FAREWELL: 55,
}


@dataclass
class TriggerConfig:
    bump_cooldown_s: float = 2.0
    startled_distance_min_cm: float = 5.0
    startled_distance_max_cm: float = 25.0
    startled_cooldown_s: float = 5.0
    shaken_imu_threshold_g: float = 1.5
    shaken_imu_release_g: float = 0.8
    shaken_min_duration_s: float = 0.5
    shaken_release_duration_s: float = 1.0
    shaken_cooldown_s: float = 3.0
    cliff_cooldown_s: float = 3.0
    bedtime_enabled: bool = True
    bedtime_start_hour: int = 23
    bedtime_brightness_threshold: float = 40.0
    bedtime_brightness_consistent_s: float = 30.0
    bedtime_cooldown_s: float = 600.0
    daily_greeting_enabled: bool = True
    daily_greeting_start_hour: int = 6
    daily_greeting_end_hour: int = 11
    daily_greeting_cooldown_s: float = 600.0
    school_farewell_enabled: bool = True
    school_farewell_hour: int = 7
    school_farewell_minute: int = 20
    school_farewell_window_min: int = 15
    # ISO weekday numbers (1=Mon..7=Sun). Empty list = todos los días.
    school_farewell_weekdays: tuple = (1, 2, 3, 4, 5)
    school_farewell_cooldown_s: float = 600.0


class TriggerDetector:
    """Mantiene estado interno y evalúa cuáles triggers están activos.

    Updaters (update_*) reciben mensajes desde callbacks ROS y son seguros
    para llamar concurrentemente con los chequeos del FSM tick (lock interno).
    """

    def __init__(self, config: TriggerConfig, persistence: Optional["object"] = None) -> None:
        self._cfg = config
        self._persistence = persistence
        self._lock = threading.Lock()

        # Sensor state
        self._cliff_active: bool = False
        self._bumper_l_at: Optional[float] = None
        self._bumper_r_at: Optional[float] = None
        self._ultrasonic_cm: Optional[float] = None
        self._imu_magnitude_g: float = 1.0
        self._wake_received_at: Optional[float] = None
        self._brightness_value: Optional[float] = None
        self._dark_since: Optional[float] = None

        # Shaken latch with hysteresis
        self._shaken_above_since: Optional[float] = None
        self._shaken_below_since: Optional[float] = None
        self._shaken_latched: bool = False

        # Edge-triggered latches: cada uno arma cuando la condición se libera
        # y dispara una sola vez al volver a entrar en la zona activa.
        # Evita el loop "objeto cerca → fire → cooldown vence → objeto sigue
        # cerca → fire de nuevo".
        self._startled_armed: bool = True
        self._bump_l_armed: bool = True
        self._bump_r_armed: bool = True
        self._cliff_armed: bool = True

        # Cooldowns
        self._last_handled_at: dict[Trigger, float] = {}

    # ─────────── update_* (called from ROS callbacks) ───────────

    def update_cliff(self, active: bool) -> None:
        with self._lock:
            self._cliff_active = bool(active)
            if not active:
                self._cliff_armed = True  # liberado: re-armado para próximo evento

    def update_bumper_l(self, active: bool, now: float) -> None:
        with self._lock:
            if active:
                self._bumper_l_at = float(now)
            else:
                self._bump_l_armed = True

    def update_bumper_r(self, active: bool, now: float) -> None:
        with self._lock:
            if active:
                self._bumper_r_at = float(now)
            else:
                self._bump_r_armed = True

    def update_ultrasonic(self, distance_cm: float) -> None:
        with self._lock:
            self._ultrasonic_cm = float(distance_cm)
            # Re-arm cuando el objeto se aleja del rango startled.
            if distance_cm > self._cfg.startled_distance_max_cm:
                self._startled_armed = True

    def update_imu_magnitude(self, g: float, now: float) -> None:
        with self._lock:
            self._imu_magnitude_g = float(g)
            cfg = self._cfg
            if g >= cfg.shaken_imu_threshold_g:
                if self._shaken_above_since is None:
                    self._shaken_above_since = float(now)
                self._shaken_below_since = None
                if (
                    not self._shaken_latched
                    and (now - self._shaken_above_since) >= cfg.shaken_min_duration_s
                ):
                    self._shaken_latched = True
            elif g <= cfg.shaken_imu_release_g:
                if self._shaken_below_since is None:
                    self._shaken_below_since = float(now)
                self._shaken_above_since = None
                if (
                    self._shaken_latched
                    and (now - self._shaken_below_since) >= cfg.shaken_release_duration_s
                ):
                    self._shaken_latched = False

    def update_wake(self, now: float) -> None:
        with self._lock:
            self._wake_received_at = float(now)

    def update_brightness(self, value: float, now: float) -> None:
        with self._lock:
            self._brightness_value = float(value)
            if value <= self._cfg.bedtime_brightness_threshold:
                if self._dark_since is None:
                    self._dark_since = float(now)
            else:
                self._dark_since = None

    # ─────────── activations / queries ───────────

    def cliff_active(self, now: float) -> bool:
        with self._lock:
            if not self._cliff_active:
                return False
            if not self._cliff_armed:
                return False  # ya disparó; espera que se libere para re-armar
            return self._cooldown_ok(Trigger.CLIFF, now, self._cfg.cliff_cooldown_s)

    def bump_active(self, now: float) -> tuple[bool, str]:
        """Returns (active, side) — side is 'left', 'right', or '' for both/none."""
        with self._lock:
            window = 0.4
            l_recent = (
                self._bumper_l_at is not None
                and (now - self._bumper_l_at) < window
                and self._bump_l_armed
            )
            r_recent = (
                self._bumper_r_at is not None
                and (now - self._bumper_r_at) < window
                and self._bump_r_armed
            )
            if not (l_recent or r_recent):
                return (False, "")
            if not self._cooldown_ok(Trigger.BUMP, now, self._cfg.bump_cooldown_s):
                return (False, "")
            if l_recent and r_recent:
                return (True, "")
            return (True, "left" if l_recent else "right")

    def startled_active(self, now: float) -> bool:
        with self._lock:
            d = self._ultrasonic_cm
            if d is None:
                return False
            cfg = self._cfg
            if not (cfg.startled_distance_min_cm <= d <= cfg.startled_distance_max_cm):
                return False
            if not self._startled_armed:
                return False  # objeto sigue cerca; espera que se aleje a re-armar
            return self._cooldown_ok(Trigger.STARTLED, now, cfg.startled_cooldown_s)

    def shaken_active(self, now: float) -> bool:
        with self._lock:
            if not self._shaken_latched:
                return False
            return self._cooldown_ok(Trigger.SHAKEN, now, self._cfg.shaken_cooldown_s)

    def wake_active(self, now: float) -> bool:
        """Devuelve True una sola vez por wake event."""
        with self._lock:
            if self._wake_received_at is None:
                return False
            handled_at = self._last_handled_at.get(Trigger.WAKE, 0.0)
            return self._wake_received_at > handled_at

    def bedtime_active(self, now: float) -> bool:
        cfg = self._cfg
        if not cfg.bedtime_enabled:
            return False
        # Defense-in-depth: cooldown runtime aunque la persistencia diga "no
        # anunciado hoy" (por si el archivo se borró o falla la escritura).
        if not self._cooldown_ok(Trigger.BEDTIME, now, cfg.bedtime_cooldown_s):
            return False
        with self._lock:
            if self._dark_since is None:
                return False
            if (now - self._dark_since) < cfg.bedtime_brightness_consistent_s:
                return False
        local_hour = _dt.datetime.now().hour
        if local_hour < cfg.bedtime_start_hour and local_hour > 4:
            return False
        if self._persistence is not None and self._persistence.bedtime_announced_today():
            return False
        return True

    def school_farewell_due(self, now: float) -> bool:
        cfg = self._cfg
        if not cfg.school_farewell_enabled:
            return False
        if not self._cooldown_ok(Trigger.SCHOOL_FAREWELL, now, cfg.school_farewell_cooldown_s):
            return False
        local = _dt.datetime.now()
        # ISO weekday: 1=Mon..7=Sun. Si la lista está vacía, no filtrar.
        weekdays = cfg.school_farewell_weekdays or ()
        if weekdays and local.isoweekday() not in weekdays:
            return False
        # Ventana: [hour:minute, hour:minute + window_min)
        target = local.replace(
            hour=cfg.school_farewell_hour,
            minute=cfg.school_farewell_minute,
            second=0,
            microsecond=0,
        )
        delta_min = (local - target).total_seconds() / 60.0
        if not (0 <= delta_min < float(cfg.school_farewell_window_min)):
            return False
        if self._persistence is not None and self._persistence.school_farewell_done_today():
            return False
        return True

    def daily_greeting_due(self, now: float) -> bool:
        cfg = self._cfg
        if not cfg.daily_greeting_enabled:
            return False
        if not self._cooldown_ok(Trigger.DAILY_GREETING, now, cfg.daily_greeting_cooldown_s):
            return False
        local_hour = _dt.datetime.now().hour
        if not (cfg.daily_greeting_start_hour <= local_hour < cfg.daily_greeting_end_hour):
            return False
        if self._persistence is not None and self._persistence.daily_greeting_done_today():
            return False
        return True

    # ─────────── housekeeping ───────────

    def mark_handled(self, trigger: Trigger, now: float) -> None:
        with self._lock:
            self._last_handled_at[trigger] = float(now)
            # Consumir latches: tras manejar un trigger edge-triggered, el
            # disparo no se repite hasta que se libere (re-arm explícito).
            if trigger == Trigger.STARTLED:
                self._startled_armed = False
            elif trigger == Trigger.BUMP:
                self._bump_l_armed = False
                self._bump_r_armed = False
            elif trigger == Trigger.CLIFF:
                self._cliff_armed = False

    def _cooldown_ok(self, trigger: Trigger, now: float, cooldown_s: float) -> bool:
        last = self._last_handled_at.get(trigger)
        if last is None:
            return True  # nunca disparado: no hay cooldown que respetar
        return (now - last) >= cooldown_s

    # Diagnostic snapshot (for /orchestrator/get_state)
    def snapshot(self) -> dict:
        with self._lock:
            return {
                "cliff_active": self._cliff_active,
                "ultrasonic_cm": self._ultrasonic_cm,
                "imu_g": self._imu_magnitude_g,
                "shaken_latched": self._shaken_latched,
                "brightness": self._brightness_value,
                "dark_since": self._dark_since,
                "wake_received_at": self._wake_received_at,
            }
