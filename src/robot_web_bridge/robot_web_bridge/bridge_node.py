from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .audio_stream import AudioStreamingNode
from .video_stream import VideoStreamNode


def main(args=None) -> None:
    """Inicia video y audio en un solo proceso para simplificar despliegues."""
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    video_node = VideoStreamNode()
    audio_node = AudioStreamingNode()
    executor.add_node(video_node)
    executor.add_node(audio_node)
    try:
        executor.spin()
    finally:
        video_node.destroy_node()
        audio_node.destroy_node()
        rclpy.shutdown()
