import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('robertito')
    default_uart_params = os.path.join(pkg_share, 'config', 'arturito_uart.yaml')
    default_eyes_params = os.path.join(pkg_share, 'config', 'arturito_eyes.yaml')
    default_face_params = os.path.join(pkg_share, 'config', 'arturito_camera.yaml')
    default_docking_params = os.path.join(pkg_share, 'config', 'docking_calibration.yaml')
    default_wifi_params = os.path.join(pkg_share, 'config', 'wifi_localization.yaml')

    uart_params = LaunchConfiguration('uart_params')
    eyes_params = LaunchConfiguration('eyes_params')
    use_eyes = LaunchConfiguration('use_eyes')
    use_tts = LaunchConfiguration('use_tts')
    use_wake_word = LaunchConfiguration('use_wake_word')
    face_params = LaunchConfiguration('face_params')
    wifi_params = LaunchConfiguration('wifi_params')
    use_face_detector = LaunchConfiguration('use_face_detector')
    use_face_greeter = LaunchConfiguration('use_face_greeter')
    tts_command = LaunchConfiguration('tts_command')
    docking_params = LaunchConfiguration('docking_params')

    declare_uart = DeclareLaunchArgument(
        'uart_params',
        default_value=default_uart_params,
        description='Path to YAML file with Arturito UART parameters.',
    )
    declare_eyes = DeclareLaunchArgument(
        'eyes_params',
        default_value=default_eyes_params,
        description='Path to YAML file with Arturito eyes parameters.',
    )
    declare_use_eyes = DeclareLaunchArgument(
        'use_eyes',
        default_value='true',
        description='Launch OLED eyes node (true/false).',
    )
    declare_use_tts = DeclareLaunchArgument(
        'use_tts',
        default_value='true',
        description='Launch text-to-speech node (true/false).',
    )
    declare_use_wake_word = DeclareLaunchArgument(
        'use_wake_word',
        default_value='true',
        description='Launch wake word assistant node (true/false).',
    )
    declare_face_params = DeclareLaunchArgument(
        'face_params',
        default_value=default_face_params,
        description='Path to YAML file with Arturito face detector parameters.',
    )
    declare_use_face_detector = DeclareLaunchArgument(
        'use_face_detector',
        default_value='true',
        description='Launch USB camera face detector (true/false).',
    )
    declare_use_face_greeter = DeclareLaunchArgument(
        'use_face_greeter',
        default_value='true',
        description='Launch face greeter node (true/false).',
    )
    declare_tts_command = DeclareLaunchArgument(
        'tts_command',
        default_value='spd-say',
        description='Command used by the TTS node for speech synthesis.',
    )
    declare_docking = DeclareLaunchArgument(
        'docking_params',
        default_value=default_docking_params,
        description='Archivo de parámetros YAML para la calibración de docking.',
    )
    declare_docking = DeclareLaunchArgument(
        'docking_params',
        default_value=default_docking_params,
        description='Archivo de parámetros YAML para la calibración de docking.',
    )
    declare_wifi = DeclareLaunchArgument(
        'wifi_params',
        default_value=default_wifi_params,
        description='Path to YAML file with WiFi localization parameters.',
    )

    uart_node = Node(
        package='robertito',
        executable='uart_node',
        name='arturito_uart_bridge',
        output='screen',
        parameters=[uart_params],
    )

    eyes_node = Node(
        package='robertito',
        executable='eyes_node',
        name='arturito_eyes',
        output='screen',
        parameters=[eyes_params],
        condition=IfCondition(use_eyes),
    )

    voice_node = Node(
        package='robertito',
        executable='voice_synth_node',
        name='arturito_voice',
        output='screen',
        parameters=[
            {
                'input_topic': 'arturito/say',
                'voice_mode': 'normal',
                'tts_provider': 'command',
                'tts_command': tts_command,
                'startup_message': 'buen dia a todos!!!',
                'startup_delay': 0.5,
            }
        ],
        condition=IfCondition(use_tts),
    )

    face_node = Node(
        package='robertito',
        executable='face_detector_node',
        name='arturito_face_detector',
        output='screen',
        parameters=[face_params],
        condition=IfCondition(use_face_detector),
    )

    face_greeter_node = Node(
        package='robertito',
        executable='face_greeter_node',
        name='arturito_face_greeter',
        output='screen',
        condition=IfCondition(use_face_greeter),
    )

    wake_word_node = Node(
        package='robertito',
        executable='wake_word_node',
        name='arturito_wake_word',
        output='screen',
        condition=IfCondition(use_wake_word),
    )

    docking_node = Node(
        package='robertito',
        executable='docking_calibration_node',
        name='docking_calibration_node',
        output='screen',
        parameters=[docking_params],
    )

    wifi_node = Node(
        package='robertito',
        executable='wifi_localization_node',
        name='wifi_localization_node',
        output='screen',
        parameters=[wifi_params],
    )

    return LaunchDescription(
        [
            declare_uart,
            declare_eyes,
            declare_use_eyes,
            declare_use_tts,
            declare_use_wake_word,
            declare_face_params,
            declare_use_face_detector,
            declare_use_face_greeter,
            declare_tts_command,
            declare_docking,
            declare_wifi,
            uart_node,
            eyes_node,
            voice_node,
            face_node,
            face_greeter_node,
            wake_word_node,
            docking_node,
            wifi_node,
        ]
    )
