from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rosbridge_port = DeclareLaunchArgument('rosbridge_port', default_value='9090')
    video_port = DeclareLaunchArgument('video_port', default_value='8080')
    video_topic = DeclareLaunchArgument('video_topic', default_value='')
    video_host = DeclareLaunchArgument('video_host', default_value='0.0.0.0')
    audio_port = DeclareLaunchArgument('audio_port', default_value='8081')
    audio_host = DeclareLaunchArgument('audio_host', default_value='0.0.0.0')
    audio_device = DeclareLaunchArgument('audio_device', default_value='default')
    audio_topic = DeclareLaunchArgument('audio_topic', default_value='')
    enable_audio = DeclareLaunchArgument('enable_audio_server', default_value='true')
    enable_alsa = DeclareLaunchArgument('enable_alsa_fallback', default_value='false')
    single_process = DeclareLaunchArgument('modo_unico_proceso', default_value='false')
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

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': LaunchConfiguration('rosbridge_port')}],
    )

    video_node = Node(
        package='robot_web_bridge',
        executable='video_server',
        name='robot_web_video_bridge',
        output='screen',
        parameters=[{
            'video_port': LaunchConfiguration('video_port'),
            'video_topic': LaunchConfiguration('video_topic'),
            'video_host': LaunchConfiguration('video_host'),
        }],
        condition=UnlessCondition(LaunchConfiguration('modo_unico_proceso')),
    )

    audio_node = Node(
        package='robot_web_bridge',
        executable='audio_server',
        name='robot_web_audio_bridge',
        output='screen',
        parameters=[{
            'audio_port': LaunchConfiguration('audio_port'),
            'audio_topic': LaunchConfiguration('audio_topic'),
            'audio_device': LaunchConfiguration('audio_device'),
            'audio_host': LaunchConfiguration('audio_host'),
            'enable_audio_server': LaunchConfiguration('enable_audio_server'),
            'enable_alsa_fallback': LaunchConfiguration('enable_alsa_fallback'),
        }],
        condition=UnlessCondition(LaunchConfiguration('modo_unico_proceso')),
    )

    combined_node = Node(
        package='robot_web_bridge',
        executable='web_bridge',
        name='robot_web_bridge',
        output='screen',
        parameters=[{
            'video_port': LaunchConfiguration('video_port'),
            'video_topic': LaunchConfiguration('video_topic'),
            'video_host': LaunchConfiguration('video_host'),
            'audio_port': LaunchConfiguration('audio_port'),
            'audio_topic': LaunchConfiguration('audio_topic'),
            'audio_device': LaunchConfiguration('audio_device'),
            'audio_host': LaunchConfiguration('audio_host'),
            'enable_audio_server': LaunchConfiguration('enable_audio_server'),
            'enable_alsa_fallback': LaunchConfiguration('enable_alsa_fallback'),
        }],
        condition=IfCondition(LaunchConfiguration('modo_unico_proceso')),
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
    )

    control_node = Node(
        package='robot_web_bridge',
        executable='control_bridge',
        name='robot_web_control_bridge',
        output='screen',
        parameters=[{
            'tilt_input_topic': LaunchConfiguration('tilt_input_topic'),
            'tilt_min_deg': LaunchConfiguration('tilt_min_deg'),
            'tilt_max_deg': LaunchConfiguration('tilt_max_deg'),
            'vacuum_input_topic': LaunchConfiguration('vacuum_input_topic'),
            'brush_input_topic': LaunchConfiguration('brush_input_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_control_bridge')),
    )

    return LaunchDescription([
        rosbridge_port,
        video_port,
        video_topic,
        video_host,
        audio_port,
        audio_host,
        audio_device,
        audio_topic,
        enable_audio,
        enable_alsa,
        single_process,
        enable_control,
        tilt_input_topic,
        tilt_min,
        tilt_max,
        vacuum_input_topic,
        brush_input_topic,
        rosapi_node,
        rosbridge_node,
        video_node,
        audio_node,
        control_node,
        combined_node,
    ])
