import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("robertito")
    default_uart_params = os.path.join(pkg_share, "config", "arturito_uart.yaml")
    default_wifi_params = os.path.join(pkg_share, "config", "wifi_localization.yaml")
    default_docking_params = os.path.join(pkg_share, "config", "docking_calibration.yaml")

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
    wifi_params_arg = DeclareLaunchArgument(
        "wifi_params",
        default_value=default_wifi_params,
        description="Archivo de parámetros YAML para localización WiFi.",
    )
    docking_params_arg = DeclareLaunchArgument(
        "docking_params",
        default_value=default_docking_params,
        description="Archivo de parámetros YAML para la calibración de docking.",
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
    wifi_params = LaunchConfiguration("wifi_params")
    docking_params = LaunchConfiguration("docking_params")
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
            {"enable_auto_blink": True},
            {"expression_topic": "robertito/eyes_expression"},
            {"person_detection_topic": "arturito/camera/faces"},
            {"person_expression": "happy"},
            {"person_timeout_sec": 1.0},
            {"sleep_expression": "sleeping"},
            {"light_level_topic": "arturito/camera/brightness"},
            {"dark_threshold": 5.0},
            {"sleep_animation_enabled": True},
            {"sleep_animation_period_sec": 0.7},
            {"sleep_tts_topic": assistant_say_topic},
            {"sleep_tts_message": "Buenas noches"},
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
        ],
    )

    wake_listener = Node(
        package="robertito",
        executable="wake_word_listener_node",
        name="wake_word_listener",
        output="screen",
        parameters=[
            {"wake_word": wake_word},
            {"language": "es-ES"},
            {"cooldown_sec": 3.0},
            {"microphone_device_index": mic_index},
            {"energy_threshold": 180.0},
            {"dynamic_energy": True},
            {"recognizer_backend": "google"},
            {"command_topic": "/tracker_control"},
            {"stop_phrase": "dejar de seguir"},
            {"start_phrase": "seguir"},
            {"wake_topic": wake_topic},
            {"tts_topic": assistant_say_topic},
            {"tts_suppress_enabled": False},
        ],
    )

    api_chat = Node(
        package="robertito",
        executable="api_chat_node",
        name="api_chat_node",
        output="screen",
        parameters=[
            {"tts_topic": assistant_say_topic},
            {"user_text_topic": "/user_text_input"},
            {"wake_topic": wake_topic},
            {"start_active": False},
            {"enable_microphone": False},
            {"voice_mode": voice_mode},
            {"weather_enabled": os.getenv("WEATHER_ENABLED", "false").lower() == "true"},
            {"weather_lat": float(os.getenv("WEATHER_LAT", "0.0"))},
            {"weather_lon": float(os.getenv("WEATHER_LON", "0.0"))},
            {"weather_location": os.getenv("WEATHER_LOCATION", "")},
            {"weather_unit": os.getenv("WEATHER_UNIT", "celsius")},
            {"follow_mode_topic": "/assistant/mode/follow_person"},
            {"clean_mode_topic": "/assistant/mode/cleaning_quick"},
        ],
    )

    face_detector = Node(
        package="robertito",
        executable="face_detector_node",
        name="face_detector_node",
        output="screen",
        parameters=[
            {"wake_topic": wake_topic},
            {"start_active": ParameterValue(face_detector_start_active, value_type=bool)},
            {"publish_brightness": True},
            {"brightness_topic": "arturito/camera/brightness"},
        ],
    )

    person_tracker = Node(
        package="robertito",
        executable="person_tracker_node",
        name="person_tracker_node",
        output="screen",
        parameters=[
            {"wake_topic": wake_topic},
            {"start_active": False},
            {"activate_on_wake": False},
            {"expression_topic": "robertito/eyes_expression"},
            {"flip_horizontal": True},
            {"command_topic": "/tracker_control"},
            {"movement_topic": "/movement_cmds"},
            {"search_tilt_deg": 30.0},
            {"search_expression": "focus"},
            {"search_angular_speed": 0.35},
            {"search_speed_command": "v255"},
            {"stop_speed_command": "S"},
            {"search_tilt_command_prefix": "t"},
            {"neutral_tilt_deg": 0.0},
            {"target_bbox_area": 22000.0},
            {"target_bbox_tolerance": 6000.0},
            {"approach_linear_speed": 0.12},
            {"target_distance_m": 0.25},
            {"max_linear_speed": 0.25},
            {"max_angular_speed": 1.8},
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

    docking_localization = Node(
        package="robertito",
        executable="docking_calibration_node",
        name="docking_calibration_node",
        output="screen",
        parameters=[docking_params],
    )

    wifi_localization = Node(
        package="robertito",
        executable="wifi_localization_node",
        name="wifi_localization_node",
        output="screen",
        parameters=[wifi_params],
    )

    node_manager = Node(
        package="robertito",
        executable="node_manager",
        name="robertito_node_manager",
        output="screen",
        parameters=[
            {"wake_topic": wake_topic},
            {"eyes_expression_topic": "robertito/eyes_expression"},
            {"tts_topic": assistant_say_topic},
            {"startup_greeting": startup_greeting},
            {"wake_greeting": "Hola, ¿en qué puedo ayudarte?"},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(startup_message_arg)
    ld.add_action(wake_word_arg)
    ld.add_action(mic_index_arg)
    ld.add_action(uart_start_enabled_arg)
    ld.add_action(uart_params_arg)
    ld.add_action(wifi_params_arg)
    ld.add_action(docking_params_arg)
    ld.add_action(face_detector_active_arg)
    ld.add_action(voice_mode_arg)
    ld.add_action(startup_greeting_arg)
    ld.add_action(eyes_node)
    ld.add_action(voice_synth)
    ld.add_action(wake_listener)
    ld.add_action(api_chat)
    ld.add_action(face_detector)
    ld.add_action(person_tracker)
    ld.add_action(uart_bridge)
    ld.add_action(wifi_localization)
    ld.add_action(docking_localization)
    ld.add_action(node_manager)
    return ld
