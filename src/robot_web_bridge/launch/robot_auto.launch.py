from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    startup_message = DeclareLaunchArgument(
        'startup_message', default_value='Hola, soy Robertito'
    )
    voice_mode = DeclareLaunchArgument('voice_mode', default_value='jarvis')
    wake_word = DeclareLaunchArgument('wake_word', default_value='robertito')
    mic_index = DeclareLaunchArgument('microphone_device_index', default_value='0')
    uart_params = DeclareLaunchArgument(
        'uart_params',
        default_value=PathJoinSubstitution(
            [FindPackageShare('robertito'), 'config', 'arturito_uart.yaml']
        ),
    )
    uart_start_enabled = DeclareLaunchArgument('uart_start_enabled', default_value='true')
    enable_control = DeclareLaunchArgument('enable_control_bridge', default_value='true')
    tilt_input_topic = DeclareLaunchArgument('tilt_input_topic', default_value='/robot_web/tilt_deg')
    tilt_min = DeclareLaunchArgument('tilt_min_deg', default_value='-20.0')
    tilt_max = DeclareLaunchArgument('tilt_max_deg', default_value='45.0')
    vacuum_input_topic = DeclareLaunchArgument(
        'vacuum_input_topic', default_value='/robot_web/vacuum_enable'
    )
    brush_input_topic = DeclareLaunchArgument(
        'brush_input_topic', default_value='/robot_web/brush_enable'
    )

    rosbridge_port = DeclareLaunchArgument('rosbridge_port', default_value='9090')
    video_port = DeclareLaunchArgument('video_port', default_value='8080')
    video_topic = DeclareLaunchArgument('video_topic', default_value='')
    audio_port = DeclareLaunchArgument('audio_port', default_value='8081')
    audio_topic = DeclareLaunchArgument('audio_topic', default_value='')
    audio_device = DeclareLaunchArgument('audio_device', default_value='default')
    enable_audio = DeclareLaunchArgument('enable_audio_server', default_value='true')
    enable_alsa = DeclareLaunchArgument('enable_alsa_fallback', default_value='false')

    robertito_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('robertito'), 'launch', 'robertito_launch.py']
            )
        ),
        launch_arguments={
            'startup_message': LaunchConfiguration('startup_message'),
            'voice_mode': LaunchConfiguration('voice_mode'),
            'wake_word': LaunchConfiguration('wake_word'),
            'microphone_device_index': LaunchConfiguration('microphone_device_index'),
            'uart_params': LaunchConfiguration('uart_params'),
            'uart_start_enabled': LaunchConfiguration('uart_start_enabled'),
        }.items(),
    )

    web_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('robot_web_bridge'), 'launch', 'web_bridge.launch.py']
            )
        ),
        launch_arguments={
            'rosbridge_port': LaunchConfiguration('rosbridge_port'),
            'video_port': LaunchConfiguration('video_port'),
            'video_topic': LaunchConfiguration('video_topic'),
            'audio_port': LaunchConfiguration('audio_port'),
            'audio_topic': LaunchConfiguration('audio_topic'),
            'audio_device': LaunchConfiguration('audio_device'),
            'enable_audio_server': LaunchConfiguration('enable_audio_server'),
            'enable_alsa_fallback': LaunchConfiguration('enable_alsa_fallback'),
            'enable_control_bridge': LaunchConfiguration('enable_control_bridge'),
            'tilt_input_topic': LaunchConfiguration('tilt_input_topic'),
            'tilt_min_deg': LaunchConfiguration('tilt_min_deg'),
            'tilt_max_deg': LaunchConfiguration('tilt_max_deg'),
            'vacuum_input_topic': LaunchConfiguration('vacuum_input_topic'),
            'brush_input_topic': LaunchConfiguration('brush_input_topic'),
        }.items(),
    )

    return LaunchDescription(
        [
            startup_message,
            voice_mode,
            wake_word,
            mic_index,
            uart_params,
            uart_start_enabled,
            enable_control,
            tilt_input_topic,
            tilt_min,
            tilt_max,
            vacuum_input_topic,
            brush_input_topic,
            rosbridge_port,
            video_port,
            video_topic,
            audio_port,
            audio_topic,
            audio_device,
            enable_audio,
            enable_alsa,
            robertito_launch,
            web_bridge_launch,
        ]
    )
