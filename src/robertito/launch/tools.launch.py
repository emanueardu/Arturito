"""Launch opcional para herramientas de calibración.

Ningún nodo de este launch arranca con el servicio principal. Usar manualmente
durante sesiones de calibración:

    ros2 launch robertito tools.launch.py enable_docking_calibration:=true
    ros2 launch robertito tools.launch.py enable_wifi_localization:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("robertito")
    default_wifi_params = os.path.join(pkg_share, "config", "wifi_localization.yaml")
    default_docking_params = os.path.join(pkg_share, "config", "docking_calibration.yaml")

    enable_docking_arg = DeclareLaunchArgument(
        "enable_docking_calibration",
        default_value="false",
        description="Si true, levanta docking_calibration_node.",
    )
    enable_wifi_arg = DeclareLaunchArgument(
        "enable_wifi_localization",
        default_value="false",
        description="Si true, levanta wifi_localization_node.",
    )
    docking_params_arg = DeclareLaunchArgument(
        "docking_params",
        default_value=default_docking_params,
        description="Archivo de parámetros YAML para la calibración de docking.",
    )
    wifi_params_arg = DeclareLaunchArgument(
        "wifi_params",
        default_value=default_wifi_params,
        description="Archivo de parámetros YAML para localización WiFi.",
    )

    docking_params = LaunchConfiguration("docking_params")
    wifi_params = LaunchConfiguration("wifi_params")

    docking_localization = Node(
        package="robertito",
        executable="docking_calibration_node",
        name="docking_calibration_node",
        output="screen",
        parameters=[docking_params],
        condition=IfCondition(LaunchConfiguration("enable_docking_calibration")),
    )

    wifi_localization = Node(
        package="robertito",
        executable="wifi_localization_node",
        name="wifi_localization_node",
        output="screen",
        parameters=[wifi_params],
        condition=IfCondition(LaunchConfiguration("enable_wifi_localization")),
    )

    ld = LaunchDescription()
    ld.add_action(enable_docking_arg)
    ld.add_action(enable_wifi_arg)
    ld.add_action(docking_params_arg)
    ld.add_action(wifi_params_arg)
    ld.add_action(docking_localization)
    ld.add_action(wifi_localization)
    return ld
