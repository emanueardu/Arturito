from __future__ import annotations

import os
import queue
import struct
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict
from urllib.parse import urlparse
import json

from audio_common_msgs.msg import AudioData
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, default))
    except (TypeError, ValueError):
        return default


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    return value if value else default


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


class _AudioHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, node: 'AudioStreamingNode', host: str, port: int) -> None:
        handler = self._build_handler(node)
        super().__init__((host, port), handler)
        self._node = node

    def _build_handler(self, node: 'AudioStreamingNode'):
        class _Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                node.handle_http_request(self)

            def log_message(self, fmt: str, *args) -> None:
                node.get_logger().debug(f'HTTP audio: {fmt % args}')

        return _Handler


class _AudioChunkDispatcher:
    def __init__(self, max_queue: int = 64) -> None:
        self._listeners: set[queue.Queue[bytes]] = set()
        self._lock = threading.Lock()
        self._max_queue = max_queue

    def register(self) -> queue.Queue[bytes]:
        listener: queue.Queue[bytes] = queue.Queue(maxsize=self._max_queue)
        with self._lock:
            self._listeners.add(listener)
        return listener

    def unregister(self, listener: queue.Queue[bytes]) -> None:
        with self._lock:
            self._listeners.discard(listener)

    def publish(self, chunk: bytes) -> None:
        to_remove: list[queue.Queue[bytes]] = []
        with self._lock:
            for listener in list(self._listeners):
                try:
                    listener.put_nowait(chunk)
                except queue.Full:
                    try:
                        listener.get_nowait()
                        listener.put_nowait(chunk)
                    except queue.Empty:
                        pass
                    except queue.Full:
                        to_remove.append(listener)
            for listener in to_remove:
                self._listeners.discard(listener)


class AudioStreamingNode(Node):
    """Reexpone audio vía HTTP en formato WAV continuo."""

    _AUDIO_TYPE = 'audio_common_msgs/msg/AudioData'

    def __init__(self) -> None:
        super().__init__('robot_web_audio_bridge')
        self._enabled = bool(
            self.declare_parameter('enable_audio_server', True).value
        )
        self._port = int(
            self.declare_parameter('audio_port', _env_int('AUDIO_PORT', 8081)).value
        )
        self._host = str(
            self.declare_parameter('audio_host', _env_str('AUDIO_HOST', '0.0.0.0')).value
        )
        self._audio_topic_param = str(
            self.declare_parameter('audio_topic', _env_str('AUDIO_TOPIC', '')).value
        ).strip()
        self._audio_device = str(
            self.declare_parameter('audio_device', _env_str('AUDIO_DEVICE', 'default')).value
        )
        self._sample_rate = int(
            self.declare_parameter('audio_sample_rate', _env_int('AUDIO_SAMPLE_RATE', 16000)).value
        )
        self._channels = int(
            self.declare_parameter('audio_channels', _env_int('AUDIO_CHANNELS', 1)).value
        )
        self._sample_width = int(
            self.declare_parameter('audio_sample_width', _env_int('AUDIO_SAMPLE_WIDTH', 2)).value
        )
        self._chunk_ms = float(
            self.declare_parameter('audio_chunk_ms', float(_env_int('AUDIO_CHUNK_MS', 50))).value
        )
        self._alsa_allowed = bool(
            self.declare_parameter(
                'enable_alsa_fallback', _env_bool('ENABLE_ALSA_FALLBACK', False)
            ).value
        )
        self._ros_subscription = None
        self._alsa_stream = None
        self._audio_source = 'none'
        self._dispatcher = _AudioChunkDispatcher()
        self._audio_catalog: Dict[str, str] = {}
        self._server = _AudioHTTPServer(self, self._host, self._port)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever, name='audio_http', daemon=True
        )
        self._server_thread.start()
        self.add_on_set_parameters_callback(self._on_parameter_update)
        if self._enabled:
            self.get_logger().info(
                f'Servidor de audio listo en http://{self._host}:{self._port}/audio'
            )
        else:
            self.get_logger().warn(
                'Servidor de audio deshabilitado vía parámetro enable_audio_server.'
            )
        self.create_timer(5.0, self._refresh_audio_topics)
        self._alsa_notice_sent = False
        self._refresh_audio_topics(initial=True)

    def _on_parameter_update(self, params):
        for param in params:
            if param.name == 'audio_topic':
                self._audio_topic_param = str(param.value).strip()
                self._switch_audio_source()
        return SetParametersResult(success=True)

    def _refresh_audio_topics(self, initial: bool = False) -> None:
        topics = self.get_topic_names_and_types()
        catalog: Dict[str, str] = {}
        for name, types in topics:
            for type_name in types:
                if type_name == self._AUDIO_TYPE:
                    catalog[name] = type_name
        self._audio_catalog = catalog
        if not self._enabled:
            return
        if self._audio_topic_param:
            if initial:
                self._switch_audio_source()
            return
        if catalog:
            best_topic = sorted(catalog.keys())[0]
            if best_topic != getattr(self, '_current_audio_topic', None):
                self._audio_topic_param = best_topic
                self._switch_audio_source()
        elif initial:
            if self._alsa_allowed:
                self.get_logger().info(
                    'No hay tópico de audio en ROS; intentaré capturar desde ALSA.'
                )
                self._switch_audio_source()
            else:
                self._audio_source = 'none'
                if not self._alsa_notice_sent:
                    self.get_logger().info(
                        'No hay tópico de audio y el fallback ALSA está deshabilitado. '
                        'Activa enable_alsa_fallback para capturar micrófono (podría interferir con wake word).'
                    )
                    self._alsa_notice_sent = True

    def _switch_audio_source(self) -> None:
        if not self._enabled:
            return
        topic = self._audio_topic_param
        if topic and topic in self._audio_catalog:
            self._start_ros_audio(topic)
        elif self._alsa_allowed:
            self._start_alsa_capture()
        else:
            self._audio_source = 'none'

    def _start_ros_audio(self, topic: str) -> None:
        if self._ros_subscription:
            self.destroy_subscription(self._ros_subscription)
            self._ros_subscription = None
        if self._alsa_stream:
            self._stop_alsa_capture()
        qos = QoSProfile(depth=10)
        self._ros_subscription = self.create_subscription(
            AudioData, topic, self._on_audio_msg, qos
        )
        self._current_audio_topic = topic
        self._audio_source = 'ros'
        self.get_logger().info(
            f'Servidor de audio escuchando datos en {topic}. '
            'Asumo PCM 16-bit; ajusta audio_sample_rate si es necesario.'
        )

    def _on_audio_msg(self, msg: AudioData) -> None:
        self._dispatcher.publish(bytes(msg.data))

    def _start_alsa_capture(self) -> None:
        if self._alsa_stream:
            return
        try:
            import sounddevice as sd  # type: ignore
        except ImportError:
            self.get_logger().error(
                'sounddevice no está instalado; instala python3-sounddevice para capturar ALSA.'
            )
            return
        dtype = {1: 'int8', 2: 'int16', 4: 'int32'}.get(self._sample_width, 'int16')
        blocksize = max(256, int(self._sample_rate * self._chunk_ms / 1000))
        device = None if self._audio_device in ('', 'default') else self._audio_device
        try:
            self._alsa_stream = sd.RawInputStream(  # type: ignore
                channels=self._channels,
                samplerate=self._sample_rate,
                dtype=dtype,
                blocksize=blocksize,
                device=device,
                callback=self._on_alsa_samples,
            )
            self._alsa_stream.start()
            self._audio_source = 'alsa'
            self.get_logger().info(
                f'Servidor de audio capturando desde ALSA dispositivo '
                f'{device or "default"} a {self._sample_rate} Hz.'
            )
        except Exception as exc:  # pragma: no cover - depende del hardware
            self._alsa_stream = None
            self.get_logger().error(
                f'No se pudo abrir ALSA ({self._audio_device}): {exc}'
            )

    def _stop_alsa_capture(self) -> None:
        if not self._alsa_stream:
            return
        try:
            self._alsa_stream.stop()
            self._alsa_stream.close()
        except Exception:  # pragma: no cover - limpieza
            pass
        self._alsa_stream = None

    def _on_alsa_samples(self, indata, frames, time_info, status):  # pragma: no cover
        if status:
            self.get_logger().debug(f'ALSA status: {status}')
        self._dispatcher.publish(bytes(indata))

    def handle_http_request(self, handler: BaseHTTPRequestHandler) -> None:
        parsed = urlparse(handler.path)
        if parsed.path in ('/', '/status'):
            self._respond_status(handler)
            return
        if parsed.path != '/audio':
            handler.send_error(HTTPStatus.NOT_FOUND, 'Ruta no disponible.')
            return
        if not self._enabled:
            handler.send_error(
                HTTPStatus.SERVICE_UNAVAILABLE, 'Audio deshabilitado por parámetro.'
            )
            return
        if self._audio_source == 'none':
            self._switch_audio_source()
        if self._audio_source == 'none':
            handler.send_error(
                HTTPStatus.SERVICE_UNAVAILABLE,
                'No hay fuente de audio disponible: publica audio_common_msgs/AudioData '
                'o habilita el fallback ALSA.',
            )
            return
        self._stream_audio(handler)

    def _stream_audio(self, handler: BaseHTTPRequestHandler) -> None:
        listener = self._dispatcher.register()
        handler.send_response(HTTPStatus.OK)
        handler.send_header('Content-Type', 'audio/wav')
        handler.send_header('Cache-Control', 'no-store')
        handler.send_header('Access-Control-Allow-Origin', '*')
        handler.end_headers()
        handler.wfile.write(self._wav_header())
        try:
            while True:
                try:
                    chunk = listener.get(timeout=5.0)
                except queue.Empty:
                    silence = bytes(self._sample_width * self._channels * 128)
                    handler.wfile.write(silence)
                    continue
                handler.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().debug('Cliente de audio desconectado.')
        finally:
            self._dispatcher.unregister(listener)

    def _wav_header(self) -> bytes:
        byte_rate = self._sample_rate * self._channels * self._sample_width
        block_align = self._channels * self._sample_width
        data_size = 0xFFFFFFFF & 0xFFFFFFFF
        return b''.join(
            [
                b'RIFF',
                struct.pack('<I', data_size),
                b'WAVEfmt ',
                struct.pack('<IHHIIHH', 16, 1, self._channels, self._sample_rate, byte_rate, block_align, self._sample_width * 8),
                b'data',
                struct.pack('<I', data_size),
            ]
        )

    def _respond_status(self, handler: BaseHTTPRequestHandler) -> None:
        import json

        payload = (
            json.dumps(
                {'fuente': self._audio_source, 'puerto': self._port}, ensure_ascii=False
            )
            + '\n'
        ).encode('utf-8')
        handler.send_response(HTTPStatus.OK)
        handler.send_header('Content-Type', 'application/json')
        handler.send_header('Content-Length', str(len(payload)))
        handler.end_headers()
        handler.wfile.write(payload)

    def destroy_node(self) -> bool:
        self.get_logger().info('Apagando servidor de audio.')
        self._server.shutdown()
        self._server.server_close()
        self._stop_alsa_capture()
        return super().destroy_node()


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = AudioStreamingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
