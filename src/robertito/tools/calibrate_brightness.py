#!/usr/bin/env python3
"""Mide brightness reportada por la cámara durante 30s.

Uso:
    1. Asegurate de que face_detector_node está corriendo (publish_brightness=true).
    2. Poné la luz como cuando querés que el robot se duerma.
    3. cd ~/ros2_ws && python3 src/robertito/tools/calibrate_brightness.py
    4. Ajustá ``bedtime_brightness_threshold`` en presence_orchestrator.yaml.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class BrightnessCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("brightness_calibrator")
        self.values: list[float] = []
        self.create_subscription(
            Float32, "arturito/camera/brightness", self._on_value, 10
        )
        self.start = time.monotonic()

    def _on_value(self, msg: Float32) -> None:
        try:
            self.values.append(float(msg.data))
        except (TypeError, ValueError):
            pass

    def report(self) -> None:
        if not self.values:
            print("No se recibieron lecturas de brightness. ¿face_detector activo?")
            return
        vmin = min(self.values)
        vavg = sum(self.values) / len(self.values)
        vmax = max(self.values)
        suggested = vavg * 0.6
        print(f"Lecturas: {len(self.values)}")
        print(f"  min={vmin:.1f}  avg={vavg:.1f}  max={vmax:.1f}")
        print(f"Sugerencia bedtime_brightness_threshold: {suggested:.1f}")
        print("  (60% del promedio actual)")


def main() -> None:
    rclpy.init()
    node = BrightnessCalibrator()
    print("Midiendo brightness 30s. Ctrl+C para terminar antes...")
    try:
        while time.monotonic() - node.start < 30:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
