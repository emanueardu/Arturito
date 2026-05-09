from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_topic = LaunchConfiguration("image_topic", default="/camera/image_raw")
    db_path = LaunchConfiguration("db_path", default="~/.robertito/face_db/face_db.json")
    threshold = LaunchConfiguration("threshold", default="0.6")
    greet_cooldown = LaunchConfiguration("greet_cooldown_sec", default="3600")
    process_fps = LaunchConfiguration("process_fps", default="5.0")
    detection_model = LaunchConfiguration("detection_model", default="hog")

    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value=image_topic, description="Topic de cámara"),
        DeclareLaunchArgument("db_path", default_value=db_path, description="Ruta donde se guarda la DB"),
        DeclareLaunchArgument("threshold", default_value=threshold, description="Distancia máxima para coincidencias"),
        DeclareLaunchArgument(
            "greet_cooldown_sec",
            default_value=greet_cooldown,
            description="Segundos mínimos entre saludos iguales",
        ),
        DeclareLaunchArgument(
            "process_fps",
            default_value=process_fps,
            description="Máxima velocidad de procesamiento",
        ),
        DeclareLaunchArgument(
            "detection_model",
            default_value=detection_model,
            description="Modelo de detección (hog o cnn)",
        ),
        Node(
            package="robertito_face_recognition",
            executable="face_recognition_node",
            name="face_recognition_node",
            parameters=[
                {
                    "image_topic": image_topic,
                    "db_path": db_path,
                    "threshold": threshold,
                    "greet_cooldown_sec": greet_cooldown,
                    "process_fps": process_fps,
                    "detection_model": detection_model,
                }
            ],
            output="screen",
        ),
    ])
