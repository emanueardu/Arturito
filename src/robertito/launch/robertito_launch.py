import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("robertito")
    default_uart_params = os.path.join(pkg_share, "config", "arturito_uart.yaml")
    default_face_params = os.path.join(pkg_share, "config", "arturito_camera.yaml")
    default_behavior_params = os.path.join(pkg_share, "config", "behavior_state.yaml")
    default_useful_memory_params = os.path.join(pkg_share, "config", "useful_memory.yaml")
    default_family_companion_params = os.path.join(pkg_share, "config", "family_companion.yaml")
    default_living_phrases_path = os.path.join(pkg_share, "config", "living_phrases.yaml")
    default_eyes_params = os.path.join(pkg_share, "config", "arturito_eyes.yaml")
    default_orchestrator_params = os.path.join(pkg_share, "config", "presence_orchestrator.yaml")

    startup_message_arg = DeclareLaunchArgument(
        "startup_message",
        default_value="Hola, soy Robertito",
        description="Mensaje inicial que dirá el nodo de TTS al arrancar.",
    )
    wake_word_arg = DeclareLaunchArgument(
        "wake_word",
        default_value="robertito",
        description="Palabra clave que activa al asistente.",
    )
    mic_index_arg = DeclareLaunchArgument(
        "microphone_device_index",
        default_value="",
        description="Índice del micrófono a usar para el wake word.",
    )
    uart_start_enabled_arg = DeclareLaunchArgument(
        "uart_start_enabled",
        default_value="true",
        description="Si es true, el puente UART acepta comandos apenas arranca.",
    )
    uart_params_arg = DeclareLaunchArgument(
        "uart_params",
        default_value=default_uart_params,
        description="Archivo de parámetros YAML para el puente UART.",
    )
    face_params_arg = DeclareLaunchArgument(
        "face_params",
        default_value=default_face_params,
        description="Archivo de parámetros YAML para el detector de rostros.",
    )
    behavior_params_arg = DeclareLaunchArgument(
        "behavior_params",
        default_value=default_behavior_params,
        description="Archivo de parámetros YAML para el motor de estados de comportamiento.",
    )
    orchestrator_params_arg = DeclareLaunchArgument(
        "orchestrator_params",
        default_value=default_orchestrator_params,
        description="Archivo de parámetros YAML para presence_orchestrator_node.",
    )
    useful_memory_params_arg = DeclareLaunchArgument(
        "useful_memory_params",
        default_value=default_useful_memory_params,
        description="Archivo de parámetros YAML para memoria útil y recordatorios.",
    )
    family_companion_params_arg = DeclareLaunchArgument(
        "family_companion_params",
        default_value=default_family_companion_params,
        description="Archivo de parámetros YAML para la capa Family Companion.",
    )
    face_detector_active_arg = DeclareLaunchArgument(
        "face_detector_start_active",
        default_value="true",
        description="Si es true, el detector de rostros publica imágenes desde el arranque.",
    )
    voice_mode_arg = DeclareLaunchArgument(
        "voice_mode",
        default_value="jarvis",
        description="Modo de voz: normal o jarvis.",
    )
    startup_greeting_arg = DeclareLaunchArgument(
        "startup_greeting",
        default_value="",
        description="Saludo inicial del node manager. Vacío por defecto (usa startup_message de voice_synth).",
    )

    wake_topic = "/wake_word/detected"
    assistant_say_topic = "/assistant/say"
    uart_params = LaunchConfiguration("uart_params")
    face_params = LaunchConfiguration("face_params")
    behavior_params = LaunchConfiguration("behavior_params")
    orchestrator_params = LaunchConfiguration("orchestrator_params")
    useful_memory_params = LaunchConfiguration("useful_memory_params")
    family_companion_params = LaunchConfiguration("family_companion_params")
    startup_message = LaunchConfiguration("startup_message")
    wake_word = LaunchConfiguration("wake_word")
    mic_index = LaunchConfiguration("microphone_device_index")
    uart_start_enabled = LaunchConfiguration("uart_start_enabled")
    face_detector_start_active = LaunchConfiguration("face_detector_start_active")
    voice_mode = LaunchConfiguration("voice_mode")
    startup_greeting = LaunchConfiguration("startup_greeting")

    eyes_node = Node(
        package="robertito",
        executable="eyes_node",
        name="robertito_eyes",
        output="screen",
        parameters=[
            default_eyes_params,  # arturito_eyes.yaml — rotate, gaze, frame_rate, etc.
            {"enable_auto_blink": False},
            {"expression_topic": "robertito/eyes_expression"},
            # Gaze: las pupilas siguen al rostro detectado por face_detector_node.
            {"person_detection_topic": "arturito/camera/faces"},
            {"person_expression": "normal"},
            {"person_timeout_sec": 1.0},
            {"sleep_expression": "sleeping_breath"},
            {"sleep_animation_expression_a": "sleeping_breath"},
            {"sleep_animation_expression_b": "drowsy"},
            {"light_level_topic": ""},
            {"dark_threshold": 5.0},
            {"sleep_animation_enabled": False},
            {"sleep_animation_period_sec": 0.7},
            {"sleep_tts_topic": assistant_say_topic},
            {"sleep_tts_message": ""},
        ],
    )

    voice_synth = Node(
        package="robertito",
        executable="voice_synth_node",
        name="voice_synth_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "robertito_voice.yaml"),
            {"input_topic": assistant_say_topic},
            {"voice_mode": voice_mode},
            {"startup_message": startup_message},
            {"startup_delay": 0.2},
            {"speaking_active_topic": "/behavior/speaking_active"},
            {"health_topic": "/behavior/health/voice"},
        ],
    )

    wake_listener = Node(
        package="robertito",
        executable="wake_word_listener_node",
        name="wake_word_listener",
        output="screen",
        # Defensa en profundidad: si por cualquier motivo el listener se mata
        # (USB glitch, watchdog interno, mic suspended), launch lo respawnea.
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {"wake_word": wake_word},
            {"language": "es-ES"},
            {"cooldown_sec": 3.0},
            {"microphone_device_index": mic_index},
            {"energy_threshold": 180.0},
            {"dynamic_energy": True},
            {"phrase_time_limit": 12.0},
            {"recalibrate_interval_sec": 90.0},
            {"microphone_reopen_interval_sec": 900.0},
            {"recognizer_backend": "google"},
            {"command_topic": "/tracker_control"},
            {"stop_phrase": "dejar de seguir"},
            {"start_phrase": "seguir"},
            {"command_window_sec": 15.0},
            {"direct_command_routing_enabled": True},
            {"command_requires_wake": False},
            {"conversation_end_phrases": "gracias,chau,listo"},
            {"wake_topic": wake_topic},
            {"tts_topic": assistant_say_topic},
            {"tts_suppress_enabled": True},
            {"recognized_text_topic": "/assistant/listen_text"},
            {"listening_active_topic": "/behavior/listening_active"},
            {"health_topic": "/behavior/health/wake_word"},
        ],
    )

    api_chat = Node(
        package="robertito",
        executable="api_chat_node",
        name="api_chat_node",
        output="screen",
        parameters=[
            useful_memory_params,
            family_companion_params,
            {"tts_topic": assistant_say_topic},
            {"wake_topic": wake_topic},
            {"user_text_topic": "/assistant/listen_text"},
            {"listening_timeout_topic": "/behavior/listening_timeout"},
            {"start_active": False},
            {"enable_microphone": False},
            {"voice_mode": voice_mode},
            {"weather_enabled": os.getenv("WEATHER_ENABLED", "true").lower() == "true"},
            {"weather_lat": float(os.getenv("WEATHER_LAT", "-34.8270"))},
            {"weather_lon": float(os.getenv("WEATHER_LON", "-58.3930"))},
            {"weather_location": os.getenv("WEATHER_LOCATION", "Burzaco, Buenos Aires, Argentina")},
            {"weather_unit": os.getenv("WEATHER_UNIT", "celsius")},
            {"follow_mode_topic": "/assistant/mode/follow_person"},
            {"clean_mode_topic": "/assistant/mode/cleaning_quick"},
            {"command_topic": "/tracker_control"},
            # Las expresiones ahora pasan por el orchestrator (no publishing directo).
            {"orchestrator_request_topic": "/robertito/orchestrator_request"},
            {"tilt_topic": "/head/tilt"},
            {"thinking_active_topic": "/behavior/thinking_active"},
            # Triggers de ground_mode y confirmaciones (editables vía YAML).
            {"phrases_yaml_path": default_living_phrases_path},
        ],
    )

    face_detector = Node(
        package="robertito",
        executable="face_detector_node",
        name="face_detector_node",
        output="screen",
        parameters=[
            face_params,
            {"wake_topic": wake_topic},
            {"start_active": ParameterValue(face_detector_start_active, value_type=bool)},
            {"publish_brightness": True},
            {"brightness_topic": "arturito/camera/brightness"},
        ],
    )

    # PR5: tracker activado, control por orchestrator vía /person_tracker/active.
    # El tracker NO mueve adelante/atrás (linear.x=0). Solo tilt + giro acotado ±45°.
    person_tracker = Node(
        package="robertito",
        executable="person_tracker_node",
        name="person_tracker_node",
        condition=IfCondition("true"),
        output="screen",
        parameters=[
            {"start_active": False},
            {"flip_horizontal": True},
            # Twist directo a /cmd_vel para que uart_bridge lo consuma.
            {"cmd_vel_topic": "/cmd_vel"},
            {"tilt_topic": "/head/tilt"},
            {"neutral_tilt_deg": 0.0},
            {"frame_width": 640.0},
            {"frame_height": 480.0},
            {"lost_timeout_sec": 1.5},
            {"control_period_sec": 0.05},
        ],
    )

    behavior_params_list = [behavior_params, family_companion_params]
    if "WEATHER_ENABLED" in os.environ:
        behavior_params_list.append(
            {"weather_enabled": os.getenv("WEATHER_ENABLED", "false").lower() == "true"}
        )
    if "WEATHER_LAT" in os.environ:
        behavior_params_list.append({"weather_lat": float(os.getenv("WEATHER_LAT", "0.0"))})
    if "WEATHER_LON" in os.environ:
        behavior_params_list.append({"weather_lon": float(os.getenv("WEATHER_LON", "0.0"))})
    if "WEATHER_LOCATION" in os.environ:
        behavior_params_list.append({"weather_location": os.getenv("WEATHER_LOCATION", "")})
    if "WEATHER_UNIT" in os.environ:
        behavior_params_list.append({"weather_unit": os.getenv("WEATHER_UNIT", "celsius")})

    presence_orchestrator = Node(
        package="robertito",
        executable="presence_orchestrator_node",
        name="presence_orchestrator",
        output="screen",
        parameters=[orchestrator_params, family_companion_params],
    )

    behavior_state = Node(
        package="robertito",
        executable="behavior_state_node",
        name="behavior_state_node",
        output="screen",
        parameters=behavior_params_list,
    )

    # Modo limpieza rápida: subscribe a /assistant/mode/cleaning_quick
    # y maneja aspiradora/escobillas + navegación evitando obstáculos.
    clean_quick = Node(
        package="robertito",
        executable="clean_quick_node",
        name="clean_quick",
        output="screen",
        parameters=[
            {"mode_topic": "/assistant/mode/cleaning_quick"},
            {"cmd_vel_topic": "arturito/cmd_vel_clean"},
            {"max_speed_mps": 0.18},
            {"speed_pwm": 200.0},
            {"turn_speed_radps": 0.9},
            # CRÍTICO: arrancar en INACTIVO. El default del nodo es True
            # y eso causaba que limpieza arranque sola al boot.
            {"mode_enabled": False},
            # Tilt durante limpieza: 60° hacia arriba (default era 45°).
            # Lo re-publica periódicamente para mantenerlo estable aunque
            # otra cosa lo cambie.
            {"tilt_active_deg": 60.0},
        ],
    )

    uart_bridge = Node(
        package="robertito",
        executable="uart_node",
        name="arturito_uart_bridge",
        output="screen",
        parameters=[
            uart_params,
            {"wake_topic": wake_topic},
            {"movement_topic": "/movement_cmds"},
            {"start_enabled": ParameterValue(uart_start_enabled, value_type=bool)},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(startup_message_arg)
    ld.add_action(wake_word_arg)
    ld.add_action(mic_index_arg)
    ld.add_action(uart_start_enabled_arg)
    ld.add_action(uart_params_arg)
    ld.add_action(face_params_arg)
    ld.add_action(behavior_params_arg)
    ld.add_action(orchestrator_params_arg)
    ld.add_action(useful_memory_params_arg)
    ld.add_action(family_companion_params_arg)
    ld.add_action(face_detector_active_arg)
    ld.add_action(voice_mode_arg)
    ld.add_action(startup_greeting_arg)
    ld.add_action(eyes_node)
    ld.add_action(voice_synth)
    ld.add_action(wake_listener)
    ld.add_action(api_chat)
    ld.add_action(face_detector)
    ld.add_action(person_tracker)
    ld.add_action(presence_orchestrator)
    ld.add_action(behavior_state)
    ld.add_action(clean_quick)
    ld.add_action(uart_bridge)
    return ld
