from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "robertito"))

from robertito.family_companion import (  # noqa: E402
    build_family_morning_greeting,
    build_family_weather_comment,
    render_due_reminder,
    render_reminder_ack,
)


def test_build_family_weather_comment_for_cold_day():
    snapshot = {
        "temperature_rounded": 9,
        "description": "despejado",
    }
    assert build_family_weather_comment(snapshot) == "Hoy está fresco, mejor salir con abrigo."


def test_build_family_morning_greeting_combines_weather():
    snapshot = {
        "temperature_rounded": 30,
        "description": "despejado",
    }
    message = build_family_morning_greeting(
        greeting_pool=["Buen día."],
        snapshot=snapshot,
    )
    assert message == "Buen día. Hoy hace calor, mejor ropa liviana."


def test_render_due_reminder_uses_text():
    rendered = render_due_reminder(
        text="apagar la estufa",
        user_name="Emanuel",
        templates=["Che, no te olvides de {text}."],
    )
    assert rendered == "Che, no te olvides de apagar la estufa."


def test_render_scheduled_reminder_ack():
    rendered = render_reminder_ack(
        kind="scheduled",
        spoken_when="mañana a la tarde",
        scheduled_templates=["Dale, te lo recuerdo {spoken_when}."],
        unscheduled_templates=["Dale, me lo guardo."],
    )
    assert rendered == "Dale, te lo recuerdo mañana a la tarde."
