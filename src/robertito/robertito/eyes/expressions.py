"""Catálogo de las 13 expresiones canónicas + alias legacy + motion profiles."""
from __future__ import annotations

from robertito.eyes.state import EyeState, EyesPair

# Estilo base de bordes redondeados (consistente entre todas las shapes con cuerpo)
_BASE_RADIUS = 14


EXPRESSIONS: dict[str, EyesPair] = {
    "calma": EyesPair(
        left=EyeState(),
        right=EyeState(),
    ),
    "atento": EyesPair(
        left=EyeState(
            width=80, height=56, corner_radius=16,
            pupil_size=1.2, pupil_y=-0.15,
            glint_size=4, glint_offset=(-5, -5),
        ),
        right=EyeState(
            width=80, height=56, corner_radius=16,
            pupil_size=1.2, pupil_y=-0.15,
            glint_size=4, glint_offset=(-5, -5),
        ),
    ),
    "pensativo": EyesPair(
        left=EyeState(
            pupil_size=0.9, pupil_x=0.4, pupil_y=-0.15,
            eyelid_top_pct=0.30, eyelid_bottom_pct=0.18,
            glint_size=3, glint_offset=(-3, -3),
        ),
        right=EyeState(
            pupil_size=0.9, pupil_x=0.4, pupil_y=-0.15,
            eyelid_top_pct=0.30, eyelid_bottom_pct=0.18,
            glint_size=3, glint_offset=(-3, -3),
        ),
    ),
    "feliz": EyesPair(
        left=EyeState(
            shape="arc_up", width=80, height=48,
            arc_thickness=10, pupil_visible=False, blink_disabled=True,
        ),
        right=EyeState(
            shape="arc_up", width=80, height=48,
            arc_thickness=10, pupil_visible=False, blink_disabled=True,
        ),
    ),
    "triste": EyesPair(
        left=EyeState(
            shape="inclined_rect", width=70, height=44,
            corner_radius=14, inclination_deg=15, inclination_mode="outer_drop",
            pupil_size=0.9, pupil_y=0.45,
            breath_freq_hz=0.25,
            extras=("teardrop", "sag_dynamic"),
        ),
        right=EyeState(
            shape="inclined_rect", width=70, height=44,
            corner_radius=14, inclination_deg=15, inclination_mode="outer_drop",
            pupil_size=0.9, pupil_y=0.45,
            breath_freq_hz=0.25,
            extras=("teardrop", "sag_dynamic"),
        ),
    ),
    "sorprendido": EyesPair(
        left=EyeState(
            width=60, height=56, corner_radius=16,
            pupil_size=0.45, glint_size=2, glint_offset=(-2, -2),
            breath_freq_hz=0.5,
            extras=("jitter_tension",),
        ),
        right=EyeState(
            width=60, height=56, corner_radius=16,
            pupil_size=0.45, glint_size=2, glint_offset=(-2, -2),
            breath_freq_hz=0.5,
            extras=("jitter_tension",),
        ),
    ),
    "dormido": EyesPair(
        left=EyeState(
            shape="line", width=64, height=5, corner_radius=2,
            pupil_visible=False, blink_disabled=True,
            breath_freq_hz=0.25,
            extras=("z_floating_slow",),
        ),
        right=EyeState(
            shape="line", width=64, height=5, corner_radius=2,
            pupil_visible=False, blink_disabled=True,
            breath_freq_hz=0.25,
            extras=("z_floating_slow",),
        ),
    ),
    "corazones": EyesPair(
        left=EyeState(
            shape="heart", width=56, height=50,
            pupil_visible=False, blink_disabled=True,
            extras=("heart_pulse", "sparkles"),
        ),
        right=EyeState(
            shape="heart", width=56, height=50,
            pupil_visible=False, blink_disabled=True,
            extras=("heart_pulse", "sparkles"),
        ),
    ),
    "risa_fuerte": EyesPair(
        left=EyeState(
            shape="xx", width=44, height=44, arc_thickness=5,
            pupil_visible=False, blink_disabled=True,
            extras=("shake", "bounce_xx"),
        ),
        right=EyeState(
            shape="xx", width=44, height=44, arc_thickness=5,
            pupil_visible=False, blink_disabled=True,
            extras=("shake", "bounce_xx"),
        ),
    ),
    "picaron": EyesPair(
        left=EyeState(
            shape="inclined_rect", width=72, height=30,
            corner_radius=12, inclination_deg=8, inclination_mode="inner_drop",
            pupil_size=0.7, pupil_x=0.4,
            glint_size=2, glint_offset=(-2, -2),
        ),
        right=EyeState(
            shape="inclined_rect", width=72, height=30,
            corner_radius=12, inclination_deg=8, inclination_mode="inner_drop",
            pupil_size=0.7, pupil_x=-0.4,
            glint_size=2, glint_offset=(-2, -2),
        ),
        asymmetric=True,
    ),
    "saludo": EyesPair(
        left=EyeState(
            shape="heart", width=40, height=36,
            pupil_visible=False,
            extras=("heart_pulse_slow",),
        ),
        right=EyeState(
            shape="arc_up", width=70, height=32,
            arc_thickness=10, pupil_visible=False,
            extras=("wink_dynamic",),
        ),
        asymmetric=True,
    ),
    "dormitando": EyesPair(
        left=EyeState(
            shape="arc_down", width=76, height=26,
            arc_thickness=8, pupil_visible=False, blink_disabled=True,
            breath_freq_hz=0.25,
            extras=("eyelash_dots", "z_floating_fast"),
        ),
        right=EyeState(
            shape="arc_down", width=76, height=26,
            arc_thickness=8, pupil_visible=False, blink_disabled=True,
            breath_freq_hz=0.25,
            extras=("eyelash_dots", "z_floating_fast"),
        ),
    ),
    "enojado": EyesPair(
        left=EyeState(
            shape="inclined_rect", width=76, height=32,
            corner_radius=10, inclination_deg=16, inclination_mode="inner_drop",
            pupil_size=0.7, pupil_x=0.4, pupil_y=0.2,
            glint_visible=False,
            show_brow=True, brow_style="angry",
            breath_freq_hz=0.7,
            extras=("jitter_tension", "burst_shake", "anger_spark"),
        ),
        right=EyeState(
            shape="inclined_rect", width=76, height=32,
            corner_radius=10, inclination_deg=16, inclination_mode="inner_drop",
            pupil_size=0.7, pupil_x=-0.4, pupil_y=0.2,
            glint_visible=False,
            show_brow=True, brow_style="angry",
            breath_freq_hz=0.7,
            extras=("jitter_tension", "burst_shake", "anger_spark"),
        ),
        asymmetric=True,
    ),
}


# Mapping de strings legacy a expresiones nuevas.
# Tokens "_glance_left" / "_glance_right" / "_blink" son señales internas.
ALIAS_MAP: dict[str, str] = {
    # Default
    "normal": "calma", "neutral_soft": "calma", "neutral_soft_a": "calma",
    "neutral_soft_b": "calma", "neutral_soft_c": "calma", "relaxed": "calma",
    "ack": "calma",
    # Atento
    "attentive": "atento", "focus": "atento",
    "curious": "atento", "curious_a": "atento", "curious_c": "atento",
    # Sorprendido
    "curious_b": "sorprendido", "surprised": "sorprendido",
    # Feliz
    "happy": "feliz", "happy_soft": "feliz", "happy_soft_a": "feliz",
    "happy_soft_b": "feliz", "happy_soft_c": "feliz", "greeting": "feliz",
    "pleased": "feliz",
    # Saludo (con guiño)
    "shy_happy": "saludo", "wink_left": "saludo", "wink_right": "saludo",
    "micro_wink_left": "saludo", "micro_wink_right": "saludo",
    # Pensativo
    "listening": "pensativo", "listening_a": "pensativo",
    "listening_b": "pensativo", "listening_c": "pensativo",
    "processing": "pensativo", "thinking_left": "pensativo",
    "thinking_right": "pensativo", "confused": "pensativo",
    "confused_soft": "pensativo",
    # Enojado
    "skeptical": "enojado", "angry": "enojado",
    # Corazones
    "hearts": "corazones",
    # Dormido
    "sleeping_breath": "dormido", "sleeping": "dormido",
    # Dormitando
    "sleepy_soft": "dormitando", "sleepy": "dormitando",
    "sleeping_alt": "dormitando", "drowsy": "dormitando", "sad": "dormitando",
    # Tokens internos (gaze override / blink trigger)
    "glance_left": "_glance_left",
    "glance_right": "_glance_right",
    "blink": "_blink", "blink_soft": "_blink",
}


# Movimientos corporales sugeridos por expresión.
# Hooks publicados a `robertito/motion_request`. Por ahora ningún nodo los
# consume — placeholder para un futuro motion_executor_node.
MOTION_PROFILES: dict[str, dict] = {
    "calma": {
        "head_tilt_deg": 0.0,
        "sway": {"angular_z_amp": 0.05, "freq_hz": 0.1},
    },
    "atento": {"head_tilt_deg": 8.0, "lock_target": "face"},
    "pensativo": {"head_tilt_deg": 3.0},
    "feliz": {"bounce": {"angular_z_amp": 0.4, "freq_hz": 1.5, "cycles": 2}},
    "triste": {"head_tilt_deg": -15.0, "freeze": True},
    "sorprendido": {
        "head_tilt_deg": 10.0,
        "retreat": {"linear_x": -0.05, "duration_s": 0.3},
    },
    "dormido": {"head_tilt_deg": -20.0, "freeze": True},
    "corazones": {
        "head_tilt_deg": 2.0,
        "sway": {"angular_z_amp": 0.15, "freq_hz": 0.4},
    },
    "risa_fuerte": {
        "shake": {"angular_z_amp": 0.6, "freq_hz": 4.0, "duration_s": 1.5},
    },
    "picaron": {
        "head_tilt_deg": 2.0,
        "advance": {"linear_x": 0.03, "duration_s": 0.4},
    },
    "saludo": {
        "head_tilt_deg": 5.0,
        "sway": {"angular_z_amp": 0.1, "freq_hz": 0.6},
    },
    "dormitando": {
        "head_tilt_loop": {"down_deg": -10, "up_deg": -2, "period_s": 6.0},
    },
    "enojado": {
        "head_tilt_deg": 5.0,
        "shake": {"angular_z_amp": 0.2, "freq_hz": 5.0, "duration_s": 0.6},
    },
}


def resolve_expression(name: str) -> str:
    """Resuelve un string (canónico o legacy) al nombre de expresión canónico.

    Devuelve el nombre canónico (clave de EXPRESSIONS) o un token interno
    "_glance_left" / "_glance_right" / "_blink". Si el string es desconocido
    devuelve "calma".
    """
    key = (name or "").strip().lower()
    if key in EXPRESSIONS:
        return key
    return ALIAS_MAP.get(key, "calma")
