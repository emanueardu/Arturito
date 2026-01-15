import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('robertito')
    robot_bringup_share = get_package_share_directory('robot_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(robot_bringup_share, 'maps', 'apartamento_map.yaml')
    nav2_params_default = os.path.join(pkg_share, 'config', 'robertito_nav2_params.yaml')
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    open_loop_params = os.path.join(pkg_share, 'config', 'open_loop_odom.yaml')
    map_odom_params = os.path.join(pkg_share, 'config', 'map_odom.yaml')
    obstacles_params = os.path.join(pkg_share, 'config', 'obstacles.yaml')
    wifi_params = os.path.join(pkg_share, 'config', 'wifi_localization.yaml')
    docking_params = os.path.join(pkg_share, 'config', 'docking_calibration.yaml')
    uart_params = os.path.join(pkg_share, 'config', 'arturito_uart.yaml')

    map_arg = DeclareLaunchArgument('map', default_value=default_map, description='Mapa YAML para Nav2')
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params', default_value=nav2_params_default, description='Archivo de parámetros de Nav2'
    )
    uart_enabled_arg = DeclareLaunchArgument(
        'uart_start_enabled',
        default_value='true',
        description='Permite al bridge UART aceptar comandos al iniciar',
    )

    os.environ.setdefault('RMW_CYCLONEDDS_PARTICIPANT_LIMIT', '256')
    dds_participant_limit = SetEnvironmentVariable(
        'RMW_CYCLONEDDS_PARTICIPANT_LIMIT',
        '256',
    )

    twist_mux = Node(
        package='robertito',
        executable='twist_mux_node',
        name='robertito_twist_mux',
        output='screen',
        parameters=[twist_mux_params],
    )

    cmd_vel_bridge = Node(
        package='robertito',
        executable='cmd_vel_bridge_node',
        name='robertito_cmd_vel_bridge',
        output='screen',
        parameters=[{'input_topic': 'cmd_vel_nav', 'output_topic': '/cmd_vel/nav2'}],
    )

    open_loop_odom = Node(
        package='robertito',
        executable='open_loop_odom_node',
        name='robertito_open_loop_odom',
        output='screen',
        parameters=[open_loop_params],
    )

    map_odom = Node(
        package='robertito',
        executable='map_odom_broadcaster',
        name='robertito_map_odom_broadcaster',
        output='screen',
        parameters=[map_odom_params],
    )

    obstacles = Node(
        package='robertito',
        executable='obstacles_node',
        name='robertito_obstacles',
        output='screen',
        parameters=[obstacles_params],
    )

    wifi_localization = Node(
        package='robertito',
        executable='wifi_localization_node',
        name='wifi_localization_node',
        output='screen',
        parameters=[wifi_params],
    )

    docking_localization = Node(
        package='robertito',
        executable='docking_calibration_node',
        name='docking_calibration_node',
        output='screen',
        parameters=[docking_params],
    )

    uart_bridge = Node(
        package='robertito',
        executable='uart_node',
        name='arturito_uart_bridge',
        output='screen',
        parameters=[uart_params, {'start_enabled': LaunchConfiguration('uart_start_enabled')}],
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_camera',
        arguments=['0.12', '0', '0.25', '0', '0', '0', 'base_link', 'camera_link'],
    )

    sonar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_sonar',
        arguments=['0.18', '0', '0.08', '0', '0', '0', 'base_link', 'ultrasonic_link'],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('nav2_params'),
            'use_sim_time': 'False',
            'slam': 'False',
            'use_localization': 'False',
            'use_namespace': 'False',
            'autostart': 'True',
            'use_composition': 'False',
            'log_level': 'info',
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(nav2_params_arg)
    ld.add_action(uart_enabled_arg)
    ld.add_action(dds_participant_limit)
    ld.add_action(wifi_localization)
    ld.add_action(docking_localization)
    ld.add_action(uart_bridge)
    ld.add_action(twist_mux)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(open_loop_odom)
    ld.add_action(map_odom)
    ld.add_action(obstacles)
    ld.add_action(imu_tf)
    ld.add_action(camera_tf)
    ld.add_action(sonar_tf)
    ld.add_action(nav2_launch)

    return ld
