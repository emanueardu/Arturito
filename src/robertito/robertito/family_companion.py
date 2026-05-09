import random
from datetime import datetime
from typing import Any, Optional

import requests


def weather_description(code: Optional[int]) -> Optional[str]:
    mapping = {
        0: "despejado",
        1: "mayormente despejado",
        2: "parcialmente nublado",
        3: "nublado",
        45: "con niebla",
        48: "con niebla",
        51: "con llovizna leve",
        53: "con llovizna",
        55: "con llovizna intensa",
        61: "con lluvia leve",
        63: "con lluvia",
        65: "con lluvia fuerte",
        80: "con chaparrones leves",
        81: "con chaparrones",
        82: "con chaparrones fuertes",
        95: "con tormenta",
    }
    return mapping.get(code)


def fetch_weather_snapshot(
    *,
    latitude: float,
    longitude: float,
    temperature_unit: str,
    timeout_sec: float,
    timezone_name: str,
) -> Optional[dict[str, Any]]:
    if latitude == 0.0 and longitude == 0.0:
        return None
    params = {
        "latitude": latitude,
        "longitude": longitude,
        "current": "temperature_2m,weather_code",
        "temperature_unit": temperature_unit,
        "timezone": timezone_name or "auto",
    }
    try:
        response = requests.get(
            "https://api.open-meteo.com/v1/forecast",
            params=params,
            timeout=timeout_sec,
        )
        response.raise_for_status()
        payload = response.json()
    except requests.RequestException:
        return None
    current = payload.get("current", {})
    temperature = current.get("temperature_2m")
    if temperature is None:
        return None
    code = current.get("weather_code")
    description = None
    if code is not None:
        try:
            description = weather_description(int(code))
        except (TypeError, ValueError):
            description = None
    return {
        "temperature": float(temperature),
        "temperature_rounded": int(round(float(temperature))),
        "units": payload.get("current_units", {}).get("temperature_2m", "°C"),
        "description": description,
        "weather_code": code,
        "fetched_at": datetime.utcnow().isoformat() + "Z",
    }


def build_family_weather_comment(snapshot: Optional[dict[str, Any]]) -> str:
    if not snapshot:
        return ""
    rounded_temp = int(snapshot.get("temperature_rounded", 0))
    description = str(snapshot.get("description") or "").strip().lower()
    rainy = any(token in description for token in ("lluvia", "chaparr", "tormenta"))

    if rainy:
        return f"Hoy viene {description or 'inestable'}, mejor salir preparado."
    if rounded_temp <= 12:
        return "Hoy está fresco, mejor salir con abrigo."
    if rounded_temp <= 18:
        return "Hoy está medio fresco."
    if rounded_temp >= 29:
        return "Hoy hace calor, mejor ropa liviana."
    if rounded_temp >= 24:
        return "Hoy va a estar bastante calentito."
    if description in {"despejado", "mayormente despejado"}:
        return "Parece que va a estar lindo hoy."
    if description:
        return f"Hoy está {description}."
    return ""


def build_family_morning_greeting(
    *,
    greeting_pool: list[str],
    snapshot: Optional[dict[str, Any]],
) -> str:
    greeting = random.choice(greeting_pool) if greeting_pool else "Buen día."
    weather_comment = build_family_weather_comment(snapshot)
    if not weather_comment:
        return greeting.strip()
    greeting = greeting.strip()
    if greeting.endswith("."):
        return f"{greeting} {weather_comment}"
    return f"{greeting}. {weather_comment}"


def build_family_weather_report(snapshot: Optional[dict[str, Any]], *, location: str) -> str:
    if not snapshot:
        return "No pude consultar el clima ahora."
    rounded_temp = int(snapshot.get("temperature_rounded", 0))
    units = str(snapshot.get("units") or "°C")
    description = str(snapshot.get("description") or "").strip()
    place = (location or "tu zona").strip()
    base = f"En {place}"
    if description:
        return f"{base} está {description} y hacen {rounded_temp}{units}."
    return f"{base} hacen {rounded_temp}{units}."


def render_due_reminder(
    *,
    text: str,
    user_name: str,
    templates: list[str],
) -> str:
    cleaned = text.strip().rstrip(".")
    pool = templates or ["Acordate de {text}."]
    rendered = random.choice(pool).format(
        user_name=(user_name.strip() or "Che"),
        text=cleaned,
    )
    return rendered.strip()


def render_reminder_ack(
    *,
    kind: str,
    spoken_when: str,
    scheduled_templates: list[str],
    unscheduled_templates: list[str],
) -> str:
    if kind == "scheduled":
        template_pool = scheduled_templates or ["Dale, te lo recuerdo {spoken_when}."]
        return random.choice(template_pool).format(spoken_when=spoken_when).strip()
    template_pool = unscheduled_templates or ["Dale, me lo guardo."]
    return random.choice(template_pool).strip()
