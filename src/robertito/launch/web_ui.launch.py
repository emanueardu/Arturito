"""Launch opcional para el stack web de Robertito.

NO se levanta en boot. Usar cuando se necesita el frontend web:

    ros2 launch robertito web_ui.launch.py

Wrappea web_bridge.launch.py de robot_web_bridge para ofrecer una entrada
estable desde el paquete robertito (sin duplicar lógica).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    enable_rosbridge_arg = DeclareLaunchArgument(
        "enable_rosbridge",
        default_value="true",
        description=(
            "Si true, levanta rosapi + rosbridge_websocket. "
            "NOTA: web_bridge.launch.py upstream no expone toggle individual; "
            "este flag se documenta para uso futuro."
        ),
    )
    enable_video_arg = DeclareLaunchArgument(
        "enable_video_bridge",
        default_value="true",
        description=(
            "Si true, levanta robot_web_video_bridge. "
            "NOTA: web_bridge.launch.py upstream no expone toggle individual; "
            "este flag se documenta para uso futuro."
        ),
    )
    enable_audio_arg = DeclareLaunchArgument(
        "enable_audio_bridge",
        default_value="true",
        description="Si true, levanta robot_web_audio_bridge.",
    )
    enable_control_arg = DeclareLaunchArgument(
        "enable_control_bridge",
        default_value="true",
        description="Si true, levanta robot_web_control_bridge.",
    )

    rosbridge_port_arg = DeclareLaunchArgument("rosbridge_port", default_value="9090")
    video_port_arg = DeclareLaunchArgument("video_port", default_value="8080")
    video_topic_arg = DeclareLaunchArgument("video_topic", default_value="")
    audio_port_arg = DeclareLaunchArgument("audio_port", default_value="8081")
    audio_topic_arg = DeclareLaunchArgument("audio_topic", default_value="")
    audio_device_arg = DeclareLaunchArgument("audio_device", default_value="default")
    enable_alsa_arg = DeclareLaunchArgument("enable_alsa_fallback", default_value="false")

    web_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_web_bridge"), "launch", "web_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "rosbridge_port": LaunchConfiguration("rosbridge_port"),
            "video_port": LaunchConfiguration("video_port"),
            "video_topic": LaunchConfiguration("video_topic"),
            "audio_port": LaunchConfiguration("audio_port"),
            "audio_topic": LaunchConfiguration("audio_topic"),
            "audio_device": LaunchConfiguration("audio_device"),
            "enable_audio_server": LaunchConfiguration("enable_audio_bridge"),
            "enable_alsa_fallback": LaunchConfiguration("enable_alsa_fallback"),
            "enable_control_bridge": LaunchConfiguration("enable_control_bridge"),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(enable_rosbridge_arg)
    ld.add_action(enable_video_arg)
    ld.add_action(enable_audio_arg)
    ld.add_action(enable_control_arg)
    ld.add_action(rosbridge_port_arg)
    ld.add_action(video_port_arg)
    ld.add_action(video_topic_arg)
    ld.add_action(audio_port_arg)
    ld.add_action(audio_topic_arg)
    ld.add_action(audio_device_arg)
    ld.add_action(enable_alsa_arg)
    ld.add_action(web_bridge_launch)
    return ld
