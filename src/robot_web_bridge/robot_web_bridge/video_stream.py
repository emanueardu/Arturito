from __future__ import annotations

import os
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Optional, Tuple
from urllib.parse import parse_qs, urlparse
import json

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage, Image

try:  # OpenCV es opcional pero recomendado para convertir a JPEG.
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - entorno sin OpenCV
    cv2 = None  # type: ignore


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, default))
    except (TypeError, ValueError):
        return default


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    return value if value else default


class _FrameBuffer:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame: Optional[bytes] = None
        self._seq = 0

    def push(self, data: bytes) -> None:
        with self._condition:
            self._frame = data
            self._seq += 1
            self._condition.notify_all()

    def wait_for_frame(self, timeout: float) -> Optional[Tuple[int, bytes]]:
        end_time = time.time() + timeout
        with self._condition:
            while self._frame is None and timeout > 0:
                self._condition.wait(timeout=timeout)
                timeout = end_time - time.time()
            if self._frame is None:
                return None
            return self._seq, self._frame


class _VideoHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, node: 'VideoStreamNode', host: str, port: int) -> None:
        handler = self._build_handler(node)
        super().__init__((host, port), handler)
        self._node = node

    def _build_handler(self, node: 'VideoStreamNode'):
        class _Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802 - firma requerida
                node.handle_http_request(self)

            def log_message(self, fmt: str, *args) -> None:  # noqa: D401
                node.get_logger().debug(f'HTTP video: {fmt % args}')

        return _Handler


class VideoStreamNode(Node):
    """Convierte topics de imagen en MJPEG accesible via HTTP."""

    _SUPPORTED_TYPES = {
        'sensor_msgs/msg/Image': Image,
        'sensor_msgs/msg/CompressedImage': CompressedImage,
    }

    _TOPIC_CANDIDATES = [
        '/camera/image_raw/compressed',
        '/camera/image_raw',
        '/image_raw',
        '/image',
        '/camera/color/image_raw',
        '/camera/fisheye1/image_raw',
        '/arturito/camera/faces/image',
        'arturito/camera/faces/image',
    ]

    def __init__(self) -> None:
        super().__init__('robot_web_video_bridge')
        declared_topic = self.declare_parameter(
            'video_topic', _env_str('VIDEO_TOPIC', '')
        ).value
        self._declared_topic = str(declared_topic).strip()
        self._jpeg_quality = int(self.declare_parameter('video_jpeg_quality', 80).value)
        self._port = int(self.declare_parameter('video_port', _env_int('VIDEO_PORT', 8080)).value)
        self._host = str(self.declare_parameter('video_host', _env_str('VIDEO_HOST', '0.0.0.0')).value)
        self._frame_buffer = _FrameBuffer()
        self._bridge = CvBridge()
        self._active_topic: Optional[str] = None
        self._active_type: Optional[str] = None
        self._subscription = None
        self._catalog_lock = threading.Lock()
        self._topic_catalog: Dict[str, str] = {}
        self._placeholder_enabled = bool(
            self.declare_parameter('enable_placeholder_video', True).value
        )
        self._placeholder_topic = str(
            self.declare_parameter('placeholder_topic', '/robot_web/placeholder/image').value
        )
        self._placeholder_fps = float(
            self.declare_parameter('placeholder_fps', 4.0).value
        )
        self._placeholder_pub = (
            self.create_publisher(Image, self._placeholder_topic, QoSProfile(depth=2))
            if self._placeholder_enabled
            else None
        )
        self._placeholder_timer = None
        self._placeholder_running = False
        self._placeholder_phase = 0.0
        self._server = _VideoHTTPServer(self, self._host, self._port)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever, name='video_http', daemon=True
        )
        self._server_thread.start()
        self.get_logger().info(
            f'Servidor de video listo en http://{self._host}:{self._port}/mjpeg'
        )
        self.add_on_set_parameters_callback(self._on_parameter_update)
        self.create_timer(3.0, self._refresh_catalog)
        self._refresh_catalog(initial=True)

    def _on_parameter_update(self, params):
        for param in params:
            if param.name == 'video_topic' and param.value:
                if self._set_video_topic(str(param.value)):
                    return SetParametersResult(success=True)
                return SetParametersResult(success=False, reason='No se pudo cambiar video_topic.')
        return SetParametersResult(success=True)

    def _refresh_catalog(self, initial: bool = False) -> None:
        topics = self.get_topic_names_and_types()
        updated = {}
        for name, types in topics:
            for type_name in types:
                if type_name in self._SUPPORTED_TYPES:
                    updated[name] = type_name
        use_placeholder = False
        if not updated and self._placeholder_enabled and self._placeholder_pub is not None:
            updated[self._placeholder_topic] = 'sensor_msgs/msg/Image'
            use_placeholder = True
            self._start_placeholder_stream()
        else:
            self._stop_placeholder_stream()
        with self._catalog_lock:
            self._topic_catalog = updated
        if self._active_topic == self._placeholder_topic and not use_placeholder:
            # Forza re-selección al aparecer una cámara real.
            self._active_topic = None
        if self._active_topic is None:
            candidate = self._declared_topic or self._pick_auto_topic(updated)
            if candidate:
                self._set_video_topic(candidate, silent=not initial)
            elif initial:
                self.get_logger().warn(
                    'No se detectaron tópicos de imagen disponibles. '
                    'Configura video_topic o espera a que el nodo de visión publique.'
                )

    def _pick_auto_topic(self, catalog: Dict[str, str]) -> Optional[str]:
        if not catalog:
            return None
        for candidate in self._TOPIC_CANDIDATES:
            if candidate and candidate in catalog:
                return candidate
        return sorted(catalog.keys())[0]

    def _set_video_topic(self, topic_name: str, silent: bool = False) -> bool:
        topic = topic_name.strip()
        if not topic:
            return False
        with self._catalog_lock:
            msg_type = self._topic_catalog.get(topic)
        if msg_type is None:
            if not silent:
                self.get_logger().warn(
                    f'El tópico {topic} no se encuentra o no es sensor_msgs/Image.'
                )
            return False
        if msg_type not in self._SUPPORTED_TYPES:
            self.get_logger().error(
                f'El tópico {topic} expone {msg_type}, formato no soportado.'
            )
            return False
        if topic == self._active_topic:
            return True
        if self._subscription:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        msg_cls = self._SUPPORTED_TYPES[msg_type]
        qos = QoSProfile(depth=2)
        if msg_cls is Image:
            self._subscription = self.create_subscription(
                Image, topic, self._on_image, qos
            )
        else:
            self._subscription = self.create_subscription(
                CompressedImage, topic, self._on_compressed, qos
            )
        self._active_topic = topic
        self._active_type = msg_type
        if not silent:
            self.get_logger().info(
                f'Servidor de video escuchando {topic} ({msg_type}).'
            )
        return True

    def _on_image(self, msg: Image) -> None:
        if cv2 is None:
            self.get_logger().error_once(
                'OpenCV no está disponible; instala python3-opencv para video MJPEG.'
            )
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, encoded = cv2.imencode(
                '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)]
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f'Error convirtiendo imagen: {exc}')
            return
        if not success or encoded is None:
            self.get_logger().warn(
                'OpenCV no pudo codificar la imagen entrante en JPEG.'
            )
            return
        self._frame_buffer.push(encoded.tobytes())

    def _on_compressed(self, msg: CompressedImage) -> None:
        data = bytes(msg.data)
        fmt = msg.format.lower()
        if 'jpeg' in fmt or 'jpg' in fmt:
            self._frame_buffer.push(data)
            return
        if cv2 is None:
            self.get_logger().warn(
                f'Recepción de {fmt} sin OpenCV; no se puede recodificar a JPEG.'
            )
            return
        np_arr = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        success, encoded = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)]
        )
        if success and encoded is not None:
            self._frame_buffer.push(encoded.tobytes())

    def handle_http_request(self, handler: BaseHTTPRequestHandler) -> None:
        parsed = urlparse(handler.path)
        if parsed.path in ('/', '/status'):
            self._respond_status(handler)
            return
        if parsed.path != '/mjpeg':
            handler.send_error(HTTPStatus.NOT_FOUND, 'Ruta no disponible.')
            return
        query = parse_qs(parsed.query)
        requested_topic = query.get('topic', [None])[0]
        if requested_topic:
            if not self._set_video_topic(requested_topic):
                handler.send_error(
                    HTTPStatus.NOT_FOUND,
                    f'Tópico {requested_topic} inválido o sin soporte.',
                )
                return
        if self._active_topic is None:
            handler.send_error(
                HTTPStatus.SERVICE_UNAVAILABLE,
                'No hay video disponible; revisa el nodo de cámara.',
            )
            return
        self._stream_mjpeg(handler)

    def _stream_mjpeg(self, handler: BaseHTTPRequestHandler) -> None:
        handler.send_response(HTTPStatus.OK)
        boundary = 'frame'
        handler.send_header(
            'Content-Type', f'multipart/x-mixed-replace; boundary={boundary}'
        )
        handler.send_header('Cache-Control', 'no-store')
        handler.send_header('Access-Control-Allow-Origin', '*')
        handler.end_headers()
        last_seq = -1
        try:
            while True:
                frame_info = self._frame_buffer.wait_for_frame(timeout=5.0)
                if frame_info is None:
                    handler.wfile.write(
                        f'--{boundary}\r\nContent-Type: text/plain\r\n\r\n'.encode()
                    )
                    handler.wfile.write(b'sin-video\r\n')
                    continue
                seq, frame = frame_info
                if seq == last_seq:
                    continue
                last_seq = seq
                header = (
                    f'--{boundary}\r\n'
                    'Content-Type: image/jpeg\r\n'
                    f'Content-Length: {len(frame)}\r\n\r\n'
                )
                handler.wfile.write(header.encode())
                handler.wfile.write(frame)
                handler.wfile.write(b'\r\n')
        except (ConnectionResetError, BrokenPipeError):
            self.get_logger().debug('Cliente de video desconectado.')

    def _respond_status(self, handler: BaseHTTPRequestHandler) -> None:
        body = {
            'topic_activo': self._active_topic,
            'tipo': self._active_type,
            'puerto': self._port,
            'topics_detectados': list(self._topic_catalog.keys()),
        }
        payload = (json.dumps(body, ensure_ascii=False) + '\n').encode('utf-8')
        handler.send_response(HTTPStatus.OK)
        handler.send_header('Content-Type', 'application/json')
        handler.send_header('Content-Length', str(len(payload)))
        handler.end_headers()
        handler.wfile.write(payload)

    def destroy_node(self) -> bool:
        self.get_logger().info('Apagando servidor de video.')
        self._server.shutdown()
        self._server.server_close()
        self._stop_placeholder_stream()
        return super().destroy_node()

    def _start_placeholder_stream(self) -> None:
        if self._placeholder_running or self._placeholder_pub is None:
            return
        if self._placeholder_timer is None:
            period = max(1.0 / max(self._placeholder_fps, 1.0), 0.2)
            self._placeholder_timer = self.create_timer(period, self._publish_placeholder_frame)
        self._placeholder_running = True
        self.get_logger().warn(
            f'Publicando video de prueba en {self._placeholder_topic} hasta que exista una cámara real.'
        )

    def _stop_placeholder_stream(self) -> None:
        self._placeholder_running = False
        if self._placeholder_timer is not None:
            self._placeholder_timer.cancel()
            self._placeholder_timer = None

    def _publish_placeholder_frame(self) -> None:
        if not self._placeholder_running or self._placeholder_pub is None:
            return
        height, width = 360, 640
        gradient = np.linspace(0, 255, width, dtype=np.uint8)
        band = np.roll(gradient, int(self._placeholder_phase) % width)
        frame = np.tile(band, (height, 1))
        image = np.stack(
            [
                frame,
                np.flipud(frame),
                np.roll(frame, 50, axis=1),
            ],
            axis=2,
        )
        msg = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'robot_web_placeholder'
        self._placeholder_pub.publish(msg)
        self._placeholder_phase = (self._placeholder_phase + 3.5) % width


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = VideoStreamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
