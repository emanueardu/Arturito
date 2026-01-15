from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    enable_esp32 = DeclareLaunchArgument('enable_esp32', default_value='true')
    enable_cleaning = DeclareLaunchArgument('enable_cleaning_controller', default_value='true')
    enable_tags = DeclareLaunchArgument('enable_tag_detector', default_value='true')
    enable_docking = DeclareLaunchArgument('enable_go_to_base', default_value='false')

    port_arg     = DeclareLaunchArgument('port',   default_value='/dev/ttyAMA0')
    baud_arg     = DeclareLaunchArgument('baud',   default_value='115200')
    poll_arg     = DeclareLaunchArgument('poll_hz',default_value='15.0')

    width_arg    = DeclareLaunchArgument('width',  default_value='640')
    height_arg   = DeclareLaunchArgument('height', default_value='480')
    encoding_arg = DeclareLaunchArgument('encoding', default_value='rgb8')
    cam_name_arg = DeclareLaunchArgument('camera_name', default_value='facecam')
    cleaning_pwm_arg = DeclareLaunchArgument('cleaning_pwm', default_value='200')
    travel_pwm_arg = DeclareLaunchArgument('travel_pwm', default_value='255')
    mode_topic_arg = DeclareLaunchArgument('cleaning_mode_topic', default_value='routine_state')
    mode_bool_topic_arg = DeclareLaunchArgument('cleaning_mode_bool_topic', default_value='cleaning_active')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw')
    camera_info_topic_arg = DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info')
    base_waypoint_arg = DeclareLaunchArgument('base_waypoint_id', default_value='base')
    base_tag_arg = DeclareLaunchArgument('base_tag_id', default_value='0')

    esp32 = Node(
        package='esp32_serial_bridge',
        executable='bridge',
        name='esp32_serial_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'poll_hz': LaunchConfiguration('poll_hz'),
            'max_pwm': 255.0,
            'max_vx': 0.5,
            'max_wz': 1.5,
            'base_width': 0.28,
            'bump_recovery_enabled': True,
            'bump_recovery_pwm': 180,
            'bump_recovery_duration': 0.45,
            'obstacle_avoid_enabled': True,
            'obstacle_distance_threshold': 0.35,
            'obstacle_reverse_pwm': 120,
            'obstacle_reverse_duration': 0.3,
            'obstacle_turn_pwm': 160,
            'obstacle_turn_duration': 0.45,
            'distance_request_period': 1.0
        }],
        condition=IfCondition(LaunchConfiguration('enable_esp32'))
    )

    cleaning_controller = Node(
        package='robot_bringup',
        executable='cleaning_controller',
        name='cleaning_controller',
        output='screen',
        parameters=[{
            'cleaning_pwm': LaunchConfiguration('cleaning_pwm'),
            'travel_pwm': LaunchConfiguration('travel_pwm'),
            'mode_topic': LaunchConfiguration('cleaning_mode_topic'),
            'mode_bool_topic': LaunchConfiguration('cleaning_mode_bool_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_cleaning_controller'))
    )

    tag_params = [{
        'camera_topic': LaunchConfiguration('camera_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'camera_frame': 'camera_link',
        'aruco_dictionary': 'DICT_4X4_50',
        'tag_size_m': 0.048,
        'known_tags_file': PathJoinSubstitution([FindPackageShare('robot_bringup'), 'maps', 'apartamento_tags.yaml'])
    }]

    tag_detector = Node(
        package='robot_bringup',
        executable='tag_detector_node',
        name='tag_detector',
        output='screen',
        parameters=tag_params,
        condition=IfCondition(LaunchConfiguration('enable_tag_detector'))
    )

    go_to_base = Node(
        package='robot_bringup',
        executable='go_to_base_node',
        name='go_to_base_node',
        output='screen',
        parameters=[{
            'base_id': LaunchConfiguration('base_waypoint_id'),
            'target_tag_id': LaunchConfiguration('base_tag_id'),
            'waypoint_file': PathJoinSubstitution([FindPackageShare('robot_bringup'), 'maps', 'apartamento_waypoints.yaml']),
        }],
        condition=IfCondition(LaunchConfiguration('enable_go_to_base'))
    )

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='facecam',
        output='screen',
        parameters=[{
            'camera_name': LaunchConfiguration('camera_name'),
            'output_encoding': LaunchConfiguration('encoding'),
            'camera_frame_id': 'camera_link'
        }],
        remappings=[('image_raw', '/camera/image_raw')]
    )

    return LaunchDescription([
        enable_esp32,
        enable_cleaning,
        enable_tags,
        enable_docking,
        port_arg, baud_arg, poll_arg,
        width_arg, height_arg, encoding_arg, cam_name_arg,
        cleaning_pwm_arg, travel_pwm_arg, mode_topic_arg, mode_bool_topic_arg,
        camera_topic_arg, camera_info_topic_arg,
        base_waypoint_arg, base_tag_arg,
        esp32, cleaning_controller, tag_detector, go_to_base, camera
    ])
