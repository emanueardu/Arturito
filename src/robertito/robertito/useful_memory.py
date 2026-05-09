import json
import os
import random
import re
import time
import unicodedata
from datetime import datetime, timedelta
from typing import Any, Optional
from zoneinfo import ZoneInfo


_REMINDER_PREFIX_RE = re.compile(
    r"^(?:robertito\s+)?(?:record[aá]me|record[aá]|acordate(?:\s+que)?|acordate\s+de|no\s+te\s+olvides\s+de)\s+",
    re.IGNORECASE,
)
_MEMORY_PREFIX_RE = re.compile(
    r"^(?:robertito\s+)?(?:record[aá](?:\s+que)?|acordate(?:\s+que)?|guardate(?:\s+que)?|no\s+te\s+olvides\s+que)\s+",
    re.IGNORECASE,
)
_RELATIVE_RE = re.compile(
    r"\ben\s+(?P<amount>\d+|una|un)\s+(?P<unit>minuto|minutos|hora|horas)\b",
    re.IGNORECASE,
)
_DAY_TOKEN = (
    r"hoy|mañana|manana|"
    r"el\s+lunes|el\s+martes|el\s+miercoles|el\s+miércoles|"
    r"el\s+jueves|el\s+viernes|el\s+sabado|el\s+sábado|el\s+domingo|"
    r"lunes|martes|miercoles|miércoles|jueves|viernes|sabado|sábado|domingo"
)
_ABSOLUTE_TIME_RE = re.compile(
    rf"\b(?:(?P<day>{_DAY_TOKEN})\s+)?a\s+las?\s+(?P<hour>\d{{1,2}})(?:(?::|\.)(?P<minute>\d{{2}}))?\s*(?:hs?|horas?)?\b",
    re.IGNORECASE,
)
_BARE_HOUR_RE = re.compile(
    rf"\b(?:(?P<day>{_DAY_TOKEN})\s+)?(?P<hour>\d{{1,2}})(?:(?::|\.)(?P<minute>\d{{2}}))?\s*(?:hs?|horas)\b",
    re.IGNORECASE,
)
_WEEKDAY_OFFSET = {
    "lunes": 0, "martes": 1, "miercoles": 2, "jueves": 3,
    "viernes": 4, "sabado": 5, "domingo": 6,
}
_TOMORROW_HOUR_RE = re.compile(
    r"\bma(?:ñ|n)ana(?:\s+a\s+las?\s+(?P<hour>\d{1,2})(?::(?P<minute>\d{2}))?)\b",
    re.IGNORECASE,
)
_TOMORROW_PART_RE = re.compile(
    r"\bma(?:ñ|n)ana\s+a\s+la\s+(?P<part>manana|tarde|noche)\b",
    re.IGNORECASE,
)
_AMBIGUOUS_TIME_RE = re.compile(
    r"\b(ma(?:ñ|n)ana|hoy|pasado|tarde|noche|a\s+las?|en\s+\d+)\b",
    re.IGNORECASE,
)


def normalize_spanish(text: str) -> str:
    normalized = unicodedata.normalize("NFD", text)
    stripped = "".join(ch for ch in normalized if not unicodedata.combining(ch))
    filtered = "".join(
        (ch.lower() if ch.isalnum() or ch.isspace() else " ") for ch in stripped
    )
    return " ".join(filtered.split())


def _clean_memory_text(text: str) -> str:
    cleaned = re.sub(r"^(que|de)\s+", "", text.strip(), flags=re.IGNORECASE)
    return cleaned.strip(" .,:;")


def _hour_for_part(part: str, *, morning: int, afternoon: int, night: int) -> int:
    if part == "manana":
        return morning
    if part == "tarde":
        return afternoon
    return night


def _parse_amount(value: str) -> int:
    if value in {"un", "una"}:
        return 1
    return int(value)


def _build_absolute_due(
    *,
    day_value: Optional[str],
    hour: int,
    minute: int,
    now: datetime,
    tz: ZoneInfo,
) -> datetime:
    base = now.astimezone(tz)
    day_normalized = normalize_spanish(day_value or "").replace("el ", "").strip()
    if day_normalized == "manana":
        base = base + timedelta(days=1)
    elif day_normalized in _WEEKDAY_OFFSET:
        # Día de la semana: avanzar al próximo (si hoy ya pasó la hora, también).
        target_wd = _WEEKDAY_OFFSET[day_normalized]
        days_ahead = (target_wd - base.weekday()) % 7
        if days_ahead == 0:
            # Hoy es ese día — solo si la hora ya pasó, vamos a la próxima semana.
            tentative = base.replace(hour=hour, minute=minute, second=0, microsecond=0)
            if tentative <= now.astimezone(tz):
                days_ahead = 7
        base = base + timedelta(days=days_ahead)
    due = base.replace(hour=hour, minute=minute, second=0, microsecond=0)
    if day_normalized in {"", "hoy"} and due <= now.astimezone(tz):
        due = due + timedelta(days=1)
    return due


def classify_memory_category(normalized_text: str) -> str:
    preference_markers = (
        "prefiero",
        "me gusta",
        "no me repitas",
        "saludes corto",
        "saludame",
        "no me",
    )
    fact_markers = (
        "es el principal",
        "es la principal",
        "bluetooth es",
        "mi ",
    )
    if any(marker in normalized_text for marker in preference_markers):
        return "preference"
    if any(marker in normalized_text for marker in fact_markers):
        return "fact"
    if "contexto" in normalized_text or "reunion" in normalized_text:
        return "context"
    return "note"


def parse_memory_statement(original_text: str, normalized_text: str) -> Optional[dict[str, str]]:
    stripped_original = original_text.strip()
    if not _MEMORY_PREFIX_RE.match(stripped_original):
        return None
    text = _MEMORY_PREFIX_RE.sub("", stripped_original, count=1)
    cleaned = _clean_memory_text(text)
    if not cleaned:
        return None
    category = classify_memory_category(normalized_text)
    return {"text": cleaned, "category": category}


def parse_reminder_request(
    original_text: str,
    normalized_text: str,
    *,
    now: datetime,
    tz: ZoneInfo,
    allow_unscheduled_notes: bool,
    default_morning_hour: int,
    default_afternoon_hour: int,
    default_night_hour: int,
) -> Optional[dict[str, Any]]:
    stripped = original_text.strip()
    if not _REMINDER_PREFIX_RE.match(stripped):
        return None

    body = _REMINDER_PREFIX_RE.sub("", stripped, count=1)
    body_normalized = normalize_spanish(body)

    relative = _RELATIVE_RE.search(body_normalized)
    if relative:
        amount = _parse_amount(relative.group("amount").lower())
        unit = relative.group("unit").lower()
        delta = timedelta(hours=amount) if "hora" in unit else timedelta(minutes=amount)
        due = now.astimezone(tz) + delta
        reminder_text = _clean_memory_text(
            re.sub(_RELATIVE_RE, "", body, count=1).strip()
        )
        if not reminder_text:
            return {"kind": "clarify", "question": "¿Qué querés que te recuerde?"}
        return {
            "kind": "scheduled",
            "text": reminder_text,
            "due_at": due.astimezone(tz).isoformat(),
            "spoken_when": f"en {amount} {'hora' if 'hora' in unit and amount == 1 else unit}",
        }

    absolute_time = _ABSOLUTE_TIME_RE.search(body) or _BARE_HOUR_RE.search(body)
    if absolute_time:
        hour = int(absolute_time.group("hour"))
        minute = int(absolute_time.group("minute") or 0)
        if 0 <= hour <= 23 and 0 <= minute <= 59:
            due = _build_absolute_due(
                day_value=absolute_time.group("day"),
                hour=hour,
                minute=minute,
                now=now,
                tz=tz,
            )
            reminder_text = _clean_memory_text(
                body[: absolute_time.start()] + " " + body[absolute_time.end() :]
            )
            if not reminder_text:
                question = "¿Qué querés que te recuerde?"
                if normalize_spanish(absolute_time.group("day") or "") == "manana":
                    question = "¿Qué querés que te recuerde mañana?"
                return {"kind": "clarify", "question": question}
            spoken_when = (
                f"mañana a las {due.hour:02d}:{due.minute:02d}"
                if normalize_spanish(absolute_time.group("day") or "") == "manana"
                else f"a las {due.hour:02d}:{due.minute:02d}"
            )
            return {
                "kind": "scheduled",
                "text": reminder_text,
                "due_at": due.astimezone(tz).isoformat(),
                "spoken_when": spoken_when,
            }

    tomorrow_hour = _TOMORROW_HOUR_RE.search(body_normalized)
    if tomorrow_hour:
        hour = int(tomorrow_hour.group("hour"))
        minute = int(tomorrow_hour.group("minute") or 0)
        base = now.astimezone(tz) + timedelta(days=1)
        due = base.replace(hour=hour, minute=minute, second=0, microsecond=0)
        reminder_text = _clean_memory_text(
            re.sub(_TOMORROW_HOUR_RE, "", body, count=1).strip()
        )
        if not reminder_text:
            return {"kind": "clarify", "question": "¿Qué querés que te recuerde mañana?"}
        return {
            "kind": "scheduled",
            "text": reminder_text,
            "due_at": due.astimezone(tz).isoformat(),
            "spoken_when": f"mañana a las {due.hour:02d}:{due.minute:02d}",
        }

    tomorrow_part = _TOMORROW_PART_RE.search(body_normalized)
    if tomorrow_part:
        part = tomorrow_part.group("part").lower()
        hour = _hour_for_part(
            part,
            morning=default_morning_hour,
            afternoon=default_afternoon_hour,
            night=default_night_hour,
        )
        base = now.astimezone(tz) + timedelta(days=1)
        due = base.replace(hour=hour, minute=0, second=0, microsecond=0)
        reminder_text = _clean_memory_text(
            re.sub(_TOMORROW_PART_RE, "", body, count=1).strip()
        )
        if not reminder_text:
            return {"kind": "clarify", "question": "¿Qué querés que te recuerde mañana?"}
        return {
            "kind": "scheduled",
            "text": reminder_text,
            "due_at": due.astimezone(tz).isoformat(),
            "spoken_when": f"mañana a la {part}",
        }

    if "manana" in body_normalized and _AMBIGUOUS_TIME_RE.search(body_normalized):
        return {
            "kind": "clarify",
            "question": "¿Para qué hora de mañana querés que te lo recuerde?",
        }

    cleaned = _clean_memory_text(body)
    if not cleaned:
        return {"kind": "clarify", "question": "¿Qué querés que te recuerde?"}
    if not allow_unscheduled_notes:
        return {
            "kind": "clarify",
            "question": "¿Para cuándo querés que te lo recuerde?",
        }
    return {
        "kind": "unscheduled",
        "text": cleaned,
        "spoken_when": "sin horario",
    }


class UsefulMemoryStore:
    def __init__(self, path: str) -> None:
        self._path = os.path.expanduser(path)
        self._state = self._default_state()
        self.load()

    @staticmethod
    def _default_state() -> dict[str, Any]:
        return {
            "version": 1,
            "reminders": [],
            "memories": [],
            "daily_context": {
                "last_greeting_date": "",
                "last_reminder_summary_date": "",
            },
        }

    def load(self) -> None:
        if not self._path or not os.path.exists(self._path):
            self._state = self._default_state()
            return
        try:
            with open(self._path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            if isinstance(payload, list):
                self._state = self._default_state()
                self._state["reminders"] = payload
                self._migrate()
                self.save()
                return
            if isinstance(payload, dict):
                self._state = self._default_state()
                self._state.update(payload)
                self._migrate()
                return
        except Exception:
            self._state = self._default_state()

    def save(self) -> None:
        directory = os.path.dirname(self._path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp_path = f"{self._path}.tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(self._state, handle, ensure_ascii=False, indent=2)
        os.replace(tmp_path, self._path)

    def _migrate(self) -> None:
        reminders = self._state.get("reminders", [])
        if not isinstance(reminders, list):
            reminders = []
        migrated: list[dict[str, Any]] = []
        for reminder in reminders:
            if not isinstance(reminder, dict):
                continue
            migrated.append(
                {
                    "id": reminder.get("id") or self._new_id("reminder"),
                    "text": str(reminder.get("text", "")).strip(),
                    "due_at": reminder.get("due_at"),
                    "status": reminder.get(
                        "status",
                        "announced" if reminder.get("spoken_at") else "pending",
                    ),
                    "created_at": reminder.get("created_at") or datetime.now().isoformat(),
                    "announced_at": reminder.get("announced_at") or reminder.get("spoken_at"),
                    "last_announced_at": reminder.get("last_announced_at") or reminder.get("spoken_at"),
                    "announce_count": int(reminder.get("announce_count", 1 if reminder.get("spoken_at") else 0)),
                    "kind": reminder.get("kind") or ("scheduled" if reminder.get("due_at") else "unscheduled"),
                }
            )
        self._state["reminders"] = migrated
        memories = self._state.get("memories", [])
        if not isinstance(memories, list):
            self._state["memories"] = []
        daily_context = self._state.get("daily_context", {})
        if not isinstance(daily_context, dict):
            self._state["daily_context"] = self._default_state()["daily_context"]

    @staticmethod
    def _new_id(prefix: str) -> str:
        return f"{prefix}-{int(time.time() * 1000)}-{random.randint(1000, 9999)}"

    def add_reminder(self, *, text: str, due_at: Optional[str], kind: str) -> dict[str, Any]:
        reminder = {
            "id": self._new_id("reminder"),
            "text": text.strip(),
            "due_at": due_at,
            "status": "pending",
            "created_at": datetime.now().isoformat(),
            "announced_at": None,
            "last_announced_at": None,
            "announce_count": 0,
            "kind": kind,
        }
        self._state.setdefault("reminders", []).append(reminder)
        self.save()
        return reminder

    def add_memory(self, *, text: str, category: str) -> dict[str, Any]:
        memory = {
            "id": self._new_id("memory"),
            "text": text.strip(),
            "category": category,
            "created_at": datetime.now().isoformat(),
            "last_recalled_at": None,
        }
        self._state.setdefault("memories", []).append(memory)
        self.save()
        return memory

    def set_daily_context(self, key: str, value: str) -> None:
        self._state.setdefault("daily_context", {})[key] = value
        self.save()

    def reminders(self) -> list[dict[str, Any]]:
        return list(self._state.get("reminders", []))

    def memories(self) -> list[dict[str, Any]]:
        return list(self._state.get("memories", []))

    def due_reminders(
        self,
        *,
        now: datetime,
        repeat_interval_sec: float,
        max_announcements: int,
    ) -> list[dict[str, Any]]:
        due_items: list[dict[str, Any]] = []
        changed = False
        for reminder in self._state.get("reminders", []):
            due_at_raw = reminder.get("due_at")
            if not due_at_raw:
                continue
            try:
                due_at = datetime.fromisoformat(str(due_at_raw))
            except ValueError:
                continue
            if due_at > now:
                continue
            announce_count = int(reminder.get("announce_count", 0))
            last_announced_at = reminder.get("last_announced_at")
            if announce_count >= max_announcements:
                continue
            if last_announced_at and repeat_interval_sec > 0.0:
                try:
                    last_dt = datetime.fromisoformat(str(last_announced_at))
                except ValueError:
                    last_dt = None
                if last_dt is not None and (now - last_dt).total_seconds() < repeat_interval_sec:
                    continue
            due_items.append(reminder)
        if changed:
            self.save()
        return due_items

    def mark_reminder_announced(self, reminder_id: str, *, when: datetime) -> None:
        for reminder in self._state.get("reminders", []):
            if reminder.get("id") != reminder_id:
                continue
            reminder["status"] = "announced"
            reminder["announced_at"] = reminder.get("announced_at") or when.isoformat()
            reminder["last_announced_at"] = when.isoformat()
            reminder["announce_count"] = int(reminder.get("announce_count", 0)) + 1
            break
        self.save()

    def today_pending(self, *, now: datetime) -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        today = now.date()
        for reminder in self._state.get("reminders", []):
            due_at_raw = reminder.get("due_at")
            if not due_at_raw:
                continue
            try:
                due_at = datetime.fromisoformat(str(due_at_raw))
            except ValueError:
                continue
            if due_at.date() == today and reminder.get("status") in {"pending", "announced"}:
                items.append(reminder)
        return sorted(items, key=lambda item: item.get("due_at") or "")

    def upcoming_pending(self, *, now: datetime, limit: int) -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        for reminder in self._state.get("reminders", []):
            due_at_raw = reminder.get("due_at")
            if not due_at_raw:
                continue
            try:
                due_at = datetime.fromisoformat(str(due_at_raw))
            except ValueError:
                continue
            if reminder.get("status") not in {"pending", "announced"}:
                continue
            if due_at >= now:
                items.append(reminder)
        items.sort(key=lambda item: item.get("due_at") or "")
        return items[:limit]

    def unscheduled_reminders(self, *, limit: int) -> list[dict[str, Any]]:
        items = [
            reminder
            for reminder in self._state.get("reminders", [])
            if not reminder.get("due_at") and reminder.get("status") in {"pending", "announced"}
        ]
        return items[:limit]

    def recent_memories(self, *, limit: int) -> list[dict[str, Any]]:
        items = list(self._state.get("memories", []))
        items.sort(key=lambda item: item.get("created_at") or "", reverse=True)
        return items[:limit]
