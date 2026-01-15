from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('robot_bringup')
    default_map = PathJoinSubstitution([package_share, 'maps', 'apartamento_map.yaml'])
    nav2_params = PathJoinSubstitution([package_share, 'config', 'nav2_params.yaml'])

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map YAML file to load.'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Full path to the Nav2 parameter file to use.'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        nav2_bringup
    ])
