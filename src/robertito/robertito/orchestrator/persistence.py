"""Persistencia en disco del orchestrator (JSON con escritura atómica)."""
from __future__ import annotations

import datetime as _dt
import json
import logging
import os
import threading
from typing import Optional


_DEFAULT_STATE: dict = {
    "last_face_seen_at": 0.0,
    "last_daily_greeting_date": "",
    "last_bedtime_announced_at": 0.0,
    "last_bedtime_announced_date": "",
    "last_greeting_efusivo_at": 0.0,
    "last_greeting_casual_at": 0.0,
    "last_school_farewell_date": "",
}


class PersistenceStore:
    def __init__(self, state_file_path: str, logger: Optional[logging.Logger] = None) -> None:
        self._path = state_file_path
        self._logger = logger or logging.getLogger(__name__)
        self._lock = threading.Lock()
        self._data: dict = dict(_DEFAULT_STATE)
        self._load()

    def _load(self) -> None:
        if not os.path.exists(self._path):
            return
        try:
            with open(self._path, "r", encoding="utf-8") as fh:
                raw = json.load(fh)
            if isinstance(raw, dict):
                self._data.update(raw)
        except Exception:
            self._logger.warning(f"could not parse {self._path}; using defaults")

    def _persist(self) -> None:
        try:
            os.makedirs(os.path.dirname(self._path), exist_ok=True)
            tmp = self._path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(self._data, fh, indent=2)
            os.replace(tmp, self._path)
        except Exception:
            self._logger.exception("failed to persist orchestrator state")

    @staticmethod
    def _today_str() -> str:
        return _dt.date.today().isoformat()

    # ─────────── daily greeting ───────────

    def daily_greeting_done_today(self) -> bool:
        with self._lock:
            return self._data.get("last_daily_greeting_date") == self._today_str()

    def mark_daily_greeting_done(self) -> None:
        with self._lock:
            self._data["last_daily_greeting_date"] = self._today_str()
            self._persist()

    # ─────────── bedtime ───────────

    def bedtime_announced_today(self) -> bool:
        # Comparamos por fecha de calendario, NO por timestamp. El orchestrator
        # pasa time.monotonic() (segundos desde boot, no Unix epoch); si lo
        # interpretáramos como timestamp daría 1970-01-01 y bedtime se
        # dispararía en loop cada tick.
        with self._lock:
            return self._data.get("last_bedtime_announced_date") == self._today_str()

    def mark_bedtime_announced(self, ts: float) -> None:
        with self._lock:
            self._data["last_bedtime_announced_at"] = float(ts)
            self._data["last_bedtime_announced_date"] = self._today_str()
            self._persist()

    # ─────────── school farewell ───────────

    def school_farewell_done_today(self) -> bool:
        with self._lock:
            return self._data.get("last_school_farewell_date") == self._today_str()

    def mark_school_farewell_done(self) -> None:
        with self._lock:
            self._data["last_school_farewell_date"] = self._today_str()
            self._persist()

    # ─────────── greetings ───────────

    def greeting_efusivo_in_cooldown(self, cooldown_s: float, now: float) -> bool:
        with self._lock:
            ts = float(self._data.get("last_greeting_efusivo_at") or 0.0)
        return (now - ts) < cooldown_s

    def mark_greeting_efusivo(self, now: float) -> None:
        with self._lock:
            self._data["last_greeting_efusivo_at"] = float(now)
            self._persist()

    def greeting_casual_in_cooldown(self, cooldown_s: float, now: float) -> bool:
        with self._lock:
            ts = float(self._data.get("last_greeting_casual_at") or 0.0)
        return (now - ts) < cooldown_s

    def mark_greeting_casual(self, now: float) -> None:
        with self._lock:
            self._data["last_greeting_casual_at"] = float(now)
            self._persist()

    # ─────────── face seen ───────────

    def get_last_face_seen_at(self) -> float:
        with self._lock:
            return float(self._data.get("last_face_seen_at") or 0.0)

    def set_last_face_seen_at(self, ts: float) -> None:
        with self._lock:
            self._data["last_face_seen_at"] = float(ts)
            self._persist()
