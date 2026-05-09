from datetime import datetime
from pathlib import Path
import sys
from zoneinfo import ZoneInfo

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "robertito"))

from robertito.useful_memory import (
    UsefulMemoryStore,
    normalize_spanish,
    parse_memory_statement,
    parse_reminder_request,
)


def test_parse_relative_reminder():
    tz = ZoneInfo("America/Argentina/Buenos_Aires")
    now = datetime(2026, 4, 19, 10, 0, tzinfo=tz)
    original = "recordame en 20 minutos revisar la bomba"
    parsed = parse_reminder_request(
        original,
        normalize_spanish(original),
        now=now,
        tz=tz,
        allow_unscheduled_notes=True,
        default_morning_hour=9,
        default_afternoon_hour=15,
        default_night_hour=20,
    )
    assert parsed is not None
    assert parsed["kind"] == "scheduled"
    assert parsed["text"] == "revisar la bomba"
    assert parsed["due_at"].startswith("2026-04-19T10:20:00")


def test_parse_tomorrow_reminder_with_hour():
    tz = ZoneInfo("America/Argentina/Buenos_Aires")
    now = datetime(2026, 4, 19, 10, 0, tzinfo=tz)
    original = "recordame llamar a Pedro mañana a las 9"
    parsed = parse_reminder_request(
        original,
        normalize_spanish(original),
        now=now,
        tz=tz,
        allow_unscheduled_notes=True,
        default_morning_hour=9,
        default_afternoon_hour=15,
        default_night_hour=20,
    )
    assert parsed is not None
    assert parsed["kind"] == "scheduled"
    assert parsed["text"] == "llamar a Pedro"
    assert parsed["due_at"].startswith("2026-04-20T09:00:00")


def test_parse_tomorrow_part_of_day():
    tz = ZoneInfo("America/Argentina/Buenos_Aires")
    now = datetime(2026, 4, 19, 10, 0, tzinfo=tz)
    original = "acordate que tengo reunión mañana a la tarde"
    parsed = parse_reminder_request(
        original,
        normalize_spanish(original),
        now=now,
        tz=tz,
        allow_unscheduled_notes=True,
        default_morning_hour=9,
        default_afternoon_hour=15,
        default_night_hour=20,
    )
    assert parsed is not None
    assert parsed["kind"] == "scheduled"
    assert parsed["text"] == "tengo reunión"
    assert parsed["due_at"].startswith("2026-04-20T15:00:00")


def test_parse_unscheduled_reminder():
    tz = ZoneInfo("America/Argentina/Buenos_Aires")
    now = datetime(2026, 4, 19, 10, 0, tzinfo=tz)
    original = "recordame comprar filamento"
    parsed = parse_reminder_request(
        original,
        normalize_spanish(original),
        now=now,
        tz=tz,
        allow_unscheduled_notes=True,
        default_morning_hour=9,
        default_afternoon_hour=15,
        default_night_hour=20,
    )
    assert parsed is not None
    assert parsed["kind"] == "unscheduled"
    assert parsed["text"] == "comprar filamento"


def test_parse_memory_statement():
    original = "recordá que prefiero que me saludes corto"
    parsed = parse_memory_statement(original, normalize_spanish(original))
    assert parsed is not None
    assert parsed["category"] == "preference"
    assert parsed["text"] == "prefiero que me saludes corto"


def test_memory_store_due_reminders(tmp_path):
    store = UsefulMemoryStore(str(tmp_path / "memory.json"))
    reminder = store.add_reminder(
        text="llamar a Pedro",
        due_at="2026-04-20T09:00:00-03:00",
        kind="scheduled",
    )
    due = store.due_reminders(
        now=datetime.fromisoformat("2026-04-20T09:10:00-03:00"),
        repeat_interval_sec=0.0,
        max_announcements=1,
    )
    assert [item["id"] for item in due] == [reminder["id"]]
    store.mark_reminder_announced(reminder["id"], when=datetime.fromisoformat("2026-04-20T09:10:00-03:00"))
    due_again = store.due_reminders(
        now=datetime.fromisoformat("2026-04-20T09:15:00-03:00"),
        repeat_interval_sec=0.0,
        max_announcements=1,
    )
    assert due_again == []
