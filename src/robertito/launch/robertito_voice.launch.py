from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sensitivity_arg = DeclareLaunchArgument("sensitivity", default_value="0.6")
    vad_silence_arg = DeclareLaunchArgument("vad_silence_ms", default_value="800")
    vad_aggressiveness_arg = DeclareLaunchArgument("vad_aggressiveness", default_value="2")
    audio_backend_arg = DeclareLaunchArgument("audio_backend", default_value="sounddevice")
    audio_device_arg = DeclareLaunchArgument("audio_device", default_value="default")
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
        ],
    )

    api_chat = Node(
        package="robertito",
        executable="api_chat_node",
        name="api_chat_node",
        output="screen",
        parameters=[
            {"wake_topic": "/wake_word/detected"},
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
            {"audio_backend": audio_backend},
            {"audio_device": audio_device},
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
