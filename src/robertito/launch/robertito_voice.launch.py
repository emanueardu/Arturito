from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("robertito")
    useful_memory_params = f"{pkg_share}/config/useful_memory.yaml"
    family_companion_params = f"{pkg_share}/config/family_companion.yaml"
    sensitivity_arg = DeclareLaunchArgument("sensitivity", default_value="0.6")
    vad_silence_arg = DeclareLaunchArgument("vad_silence_ms", default_value="800")
    vad_aggressiveness_arg = DeclareLaunchArgument("vad_aggressiveness", default_value="2")
    # Entrada (STT/mic): sounddevice por default. Salida (TTS) usa ALSA
    # directo al parlante USB; ver tts_audio_backend / tts_audio_device.
    audio_backend_arg = DeclareLaunchArgument("audio_backend", default_value="sounddevice")
    audio_device_arg = DeclareLaunchArgument("audio_device", default_value="default")
    tts_audio_backend_arg = DeclareLaunchArgument(
        "tts_audio_backend", default_value="alsa"
    )
    tts_audio_device_arg = DeclareLaunchArgument(
        "tts_audio_device", default_value="plughw:CARD=Device,DEV=0"
    )
    voice_mode_arg = DeclareLaunchArgument("voice_mode", default_value="normal")
    stt_mode_arg = DeclareLaunchArgument("stt_mode", default_value="azure")
    wake_word_arg = DeclareLaunchArgument("wake_word", default_value="robertito")
    startup_message_arg = DeclareLaunchArgument("startup_message", default_value="Buenos días")
    startup_delay_arg = DeclareLaunchArgument("startup_delay", default_value="0.6")
    tts_rate_arg = DeclareLaunchArgument("tts_rate", default_value="0.90")
    tts_pitch_arg = DeclareLaunchArgument("tts_pitch", default_value="-2st")

    sensitivity = LaunchConfiguration("sensitivity")
    vad_silence_ms = LaunchConfiguration("vad_silence_ms")
    vad_aggressiveness = LaunchConfiguration("vad_aggressiveness")
    audio_backend = LaunchConfiguration("audio_backend")
    audio_device = LaunchConfiguration("audio_device")
    tts_audio_backend = LaunchConfiguration("tts_audio_backend")
    tts_audio_device = LaunchConfiguration("tts_audio_device")
    voice_mode = LaunchConfiguration("voice_mode")
    stt_mode = LaunchConfiguration("stt_mode")
    wake_word = LaunchConfiguration("wake_word")
    startup_message = LaunchConfiguration("startup_message")
    startup_delay = LaunchConfiguration("startup_delay")
    tts_rate = LaunchConfiguration("tts_rate")
    tts_pitch = LaunchConfiguration("tts_pitch")

    wake_listener = Node(
        package="robertito",
        executable="wake_word_listener_node",
        name="wake_word_listener",
        output="screen",
        parameters=[
            {"wake_word": wake_word},
            {"sensitivity": sensitivity},
            {"sample_rate": 16000},
            {"chunk_size": 1024},
            {"wake_topic": "/wake_word/detected"},
            {"tts_topic": "/assistant/say"},
            {"tts_suppress_enabled": True},
            {"recognized_text_topic": "/assistant/listen_text"},
            {"direct_command_routing_enabled": False},
            {"listening_active_topic": "/behavior/listening_active"},
            {"listening_timeout_topic": "/behavior/listening_timeout"},
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
            {"wake_topic": "/wake_word/detected"},
            {"user_text_topic": "/assistant/listen_text"},
            {"listening_timeout_topic": "/behavior/listening_timeout"},
            {"stt_mode": stt_mode},
            {"voice_mode": voice_mode},
            {"audio_backend": audio_backend},
            {"audio_device": audio_device},
            {"vad_silence_ms": vad_silence_ms},
            {"vad_aggressiveness": vad_aggressiveness},
            {"tts_topic": "/assistant/say"},
            {"startup_message": startup_message},
            {"startup_delay": startup_delay},
            {"external_tts": True},
            {"enable_microphone": False},
            {"command_topic": "/tracker_control"},
            # Las expresiones se piden vía orchestrator request bus.
            {"orchestrator_request_topic": "/robertito/orchestrator_request"},
            {"tilt_topic": "/head/tilt"},
        ],
    )

    voice_synth = Node(
        package="robertito",
        executable="voice_synth_node",
        name="voice_synth_node",
        output="screen",
        parameters=[
            {"input_topic": "/assistant/say"},
            {"voice_mode": voice_mode},
            {"tts_provider": "azure"},
            {"supports_ssml": True},
            {"use_ssml": True},
            {"rate": tts_rate},
            {"pitch": tts_pitch},
            {"voice_name": "es-AR-TomasNeural"},
            {"tts_rate": tts_rate},
            {"tts_pitch": tts_pitch},
            {"tts_output_format": "Riff24Khz16BitMonoPcm"},
            {"enable_ssml": True},
            {"audio_backend": tts_audio_backend},
            {"audio_device": tts_audio_device},
            {"startup_message": ""},
        ],
    )

    return LaunchDescription(
        [
            sensitivity_arg,
            vad_silence_arg,
            vad_aggressiveness_arg,
            audio_backend_arg,
            audio_device_arg,
            tts_audio_backend_arg,
            tts_audio_device_arg,
            voice_mode_arg,
            stt_mode_arg,
            wake_word_arg,
            startup_message_arg,
            startup_delay_arg,
            tts_rate_arg,
            tts_pitch_arg,
            wake_listener,
            api_chat,
            voice_synth,
        ]
    )
