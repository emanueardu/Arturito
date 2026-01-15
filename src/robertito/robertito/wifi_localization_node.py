import json
import os
import re
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PING_TIME_RE = re.compile(r'time[=<]([0-9.]+)\s*ms')


def _expand_path(path: str) -> str:
    return os.path.expanduser(path.strip()) if path else path


def _safe_float(value: Any) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class WifiLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__('wifi_localization_node')

        self._targets = self._load_targets()
        self._ping_count = int(self.declare_parameter('ping_count', 4).value)
        self._ping_timeout = float(self.declare_parameter('ping_timeout_sec', 1.0).value)
        self._measurement_period = float(
            self.declare_parameter('measurement_period_sec', 3.0).value
        )
        self._fail_latency_ms = float(
            self.declare_parameter('fail_latency_ms', 999.0).value
        )
        self._store_path = _expand_path(
            str(
                self.declare_parameter(
                    'anchor_store_path', '~/.ros/robertito_wifi_anchors.json'
                ).value
            )
        )
        self._anchors_topic = str(
            self.declare_parameter('anchors_topic', '/robot_web/wifi_anchors').value
        )
        self._pose_topic = str(
            self.declare_parameter('pose_topic', '/robot_web/wifi_pose').value
        )
        self._status_topic = str(
            self.declare_parameter('status_topic', '/robot_web/wifi_status').value
        )
        self._calibrate_topic = str(
            self.declare_parameter('calibrate_topic', '/robot_web/wifi_calibrate').value
        )

        self._anchors_lock = threading.Lock()
        self._anchors: List[Dict[str, Any]] = []
        self._last_latency: Optional[List[float]] = None
        self._calibrating = False

        self._pub_anchors = self.create_publisher(String, self._anchors_topic, 10)
        self._pub_pose = self.create_publisher(String, self._pose_topic, 10)
        self._pub_status = self.create_publisher(String, self._status_topic, 10)
        self.create_subscription(String, self._calibrate_topic, self._on_calibrate, 10)

        self._load_anchors()
        self._publish_anchors()

        if self._targets:
            self._timer = self.create_timer(self._measurement_period, self._on_measure)
        else:
            self._publish_status('error', 'No hay objetivos de ping configurados.')

        self.get_logger().info(
            f'WifiLocalization listo. Targets={self._targets}, anchors={len(self._anchors)}'
        )

    def _load_targets(self) -> List[str]:
        raw = self.declare_parameter('targets', ['192.168.0.1']).value
        if isinstance(raw, str):
            targets = [item.strip() for item in raw.split(',') if item.strip()]
        else:
            targets = [str(item).strip() for item in raw if str(item).strip()]
        return targets

    def _load_anchors(self) -> None:
        if not self._store_path:
            return
        try:
            if not os.path.exists(self._store_path):
                return
            with open(self._store_path, 'r', encoding='utf-8') as handle:
                payload = json.load(handle)
            anchors = payload.get('anchors', [])
            if isinstance(anchors, list):
                self._anchors = anchors
        except Exception as exc:
            self.get_logger().warn(f'No se pudieron cargar anchors: {exc}')

    def _save_anchors(self) -> None:
        if not self._store_path:
            return
        try:
            store_dir = os.path.dirname(self._store_path)
            if store_dir:
                os.makedirs(store_dir, exist_ok=True)
            payload = {
                'anchors': self._anchors,
                'targets': self._targets,
                'updated_at': time.time(),
            }
            with open(self._store_path, 'w', encoding='utf-8') as handle:
                json.dump(payload, handle, indent=2)
        except Exception as exc:
            self.get_logger().warn(f'No se pudieron guardar anchors: {exc}')

    def _publish_status(self, state: str, message: str, extra: Optional[Dict[str, Any]] = None) -> None:
        payload = {'state': state, 'message': message, 'timestamp': time.time()}
        if extra:
            payload.update(extra)
        self._pub_status.publish(String(data=json.dumps(payload)))

    def _publish_anchors(self) -> None:
        with self._anchors_lock:
            payload = {
                'anchors': self._anchors,
                'targets': self._targets,
                'updated_at': time.time(),
            }
        self._pub_anchors.publish(String(data=json.dumps(payload)))

    def _on_calibrate(self, msg: String) -> None:
        if self._calibrating:
            self._publish_status('busy', 'Calibracion en curso, espera un momento.')
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self._publish_status('error', 'Payload de calibracion invalido.')
            return
        x = _safe_float(data.get('x'))
        y = _safe_float(data.get('y'))
        label = str(data.get('label', '')).strip() or None
        if x is None or y is None:
            self._publish_status('error', 'Coordenadas invalidas para calibrar.')
            return
        if not self._targets:
            self._publish_status('error', 'No hay targets configurados.')
            return
        self._calibrating = True
        self._publish_status('calibrating', 'Midiendo latencia WiFi...', {'x': x, 'y': y})
        threading.Thread(target=self._calibrate_job, args=(x, y, label), daemon=True).start()

    def _calibrate_job(self, x: float, y: float, label: Optional[str]) -> None:
        latencies = self._measure_targets()
        anchor_id = f'anchor_{int(time.time())}'
        anchor = {
            'id': anchor_id,
            'x': x,
            'y': y,
            'label': label,
            'latencies_ms': latencies,
            'created_at': time.time(),
        }
        with self._anchors_lock:
            self._anchors.append(anchor)
        self._save_anchors()
        self._publish_anchors()
        self._publish_status('saved', 'Punto de calibracion guardado.', {'anchor': anchor})
        self._calibrating = False

    def _on_measure(self) -> None:
        latencies = self._measure_targets()
        self._last_latency = latencies
        self._publish_status('ok', 'Medicion actualizada.', {'latencies_ms': latencies})
        self._publish_estimate(latencies)

    def _measure_targets(self) -> List[float]:
        latencies: List[float] = []
        for target in self._targets:
            samples: List[float] = []
            for _ in range(max(1, self._ping_count)):
                sample = self._ping_target(target)
                if sample is not None:
                    samples.append(sample)
            if samples:
                latencies.append(sum(samples) / len(samples))
            else:
                latencies.append(self._fail_latency_ms)
        return latencies

    def _ping_target(self, target: str) -> Optional[float]:
        timeout_sec = max(1, int(round(self._ping_timeout)))
        try:
            proc = subprocess.run(
                ['ping', '-c', '1', '-W', str(timeout_sec), target],
                capture_output=True,
                text=True,
                check=False,
            )
        except Exception as exc:
            self.get_logger().warn(f'Ping fallo para {target}: {exc}')
            return None
        output = (proc.stdout or '') + '\n' + (proc.stderr or '')
        match = PING_TIME_RE.search(output)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                return None
        return None

    def _publish_estimate(self, latencies: List[float]) -> None:
        with self._anchors_lock:
            anchors = list(self._anchors)
        if not anchors:
            return
        best_anchor = None
        best_distance = None
        for anchor in anchors:
            anchor_lat = anchor.get('latencies_ms')
            if not isinstance(anchor_lat, list) or len(anchor_lat) != len(latencies):
                continue
            distance = 0.0
            for current, baseline in zip(latencies, anchor_lat):
                distance += (float(current) - float(baseline)) ** 2
            distance = distance ** 0.5
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_anchor = anchor
        if not best_anchor or best_distance is None:
            return
        confidence = 1.0 / (1.0 + best_distance)
        payload = {
            'x': best_anchor.get('x'),
            'y': best_anchor.get('y'),
            'anchor_id': best_anchor.get('id'),
            'confidence': confidence,
            'distance': best_distance,
            'latencies_ms': latencies,
            'timestamp': time.time(),
        }
        self._pub_pose.publish(String(data=json.dumps(payload)))


def main() -> None:
    rclpy.init()
    node = WifiLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
