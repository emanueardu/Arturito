import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('robertito')
    default_params = os.path.join(pkg_share, 'config', 'arturito_camera.yaml')

    params_file = LaunchConfiguration('camera_params')

    declare_params = DeclareLaunchArgument(
        'camera_params',
        default_value=default_params,
        description='Path to YAML file with Arturito face detector parameters.',
    )

    detector_node = Node(
        package='robertito',
        executable='arturito_face_detector',
        name='arturito_face_detector',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([declare_params, detector_node])
