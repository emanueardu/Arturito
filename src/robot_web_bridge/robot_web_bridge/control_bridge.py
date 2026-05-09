from __future__ import annotations

import os
import shlex
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, String

def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    return value if value else default


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


class ControlBridgeNode(Node):
    """Traduce comandos simples del frontend a servicios/tópicos internos."""

    def __init__(self) -> None:
        super().__init__('robot_web_control_bridge')
        qos = QoSProfile(depth=10)
        self._movement_topic = str(
            self.declare_parameter(
                'movement_topic', _env_str('MOVEMENT_TOPIC', '/movement_cmds')
            ).value
        )
        self._tilt_input_topic = str(
            self.declare_parameter(
                'tilt_input_topic', _env_str('TILT_INPUT_TOPIC', '/robot_web/tilt_deg')
            ).value
        )
        self._head_tilt_topic = str(
            self.declare_parameter(
                'head_tilt_topic', _env_str('HEAD_TILT_TOPIC', '/head/tilt')
            ).value
        )
        self._tilt_command_dedupe_window_sec = float(
            self.declare_parameter('tilt_command_dedupe_window_sec', 0.25).value
        )
        self._tilt_prefix = str(
            self.declare_parameter(
                'tilt_command_prefix', _env_str('TILT_COMMAND_PREFIX', 't')
            ).value
        )
        self._tts_topic = str(
            self.declare_parameter('tts_topic', _env_str('TTS_TOPIC', '/tts_input')).value
        )
        self._wander_tilt_deg = float(
            self.declare_parameter('wander_tilt_deg', _env_float('WANDER_TILT_DEG', 45.0)).value
        )
        self._tilt_min = float(
            self.declare_parameter(
                'tilt_min_deg', _env_float('TILT_MIN_DEG', -20.0)
            ).value
        )
        self._tilt_max = float(
            self.declare_parameter(
                'tilt_max_deg', _env_float('TILT_MAX_DEG', 45.0)
            ).value
        )

        self._vacuum_input_topic = str(
            self.declare_parameter(
                'vacuum_input_topic',
                _env_str('VACUUM_INPUT_TOPIC', '/robot_web/vacuum_enable'),
            ).value
        )
        self._brush_input_topic = str(
            self.declare_parameter(
                'brush_input_topic',
                _env_str('BRUSH_INPUT_TOPIC', '/robot_web/brush_enable'),
            ).value
        )
        self._vacuum_passthrough_topic = str(
            self.declare_parameter(
                'vacuum_passthrough_topic',
                _env_str('VACUUM_PASSTHROUGH_TOPIC', '/vacuum/enabled'),
            ).value
        )
        self._brush_passthrough_topic = str(
            self.declare_parameter(
                'brush_passthrough_topic',
                _env_str('BRUSH_PASSTHROUGH_TOPIC', '/brush/enabled'),
            ).value
        )
        self._wander_input_topic = str(
            self.declare_parameter(
                'wander_input_topic',
                _env_str('WANDER_INPUT_TOPIC', '/robot_web/wander_enable'),
            ).value
        )
        self._wander_status_topic = str(
            self.declare_parameter(
                'wander_status_topic',
                _env_str('WANDER_STATUS_TOPIC', '/robot_web/wander_status'),
            ).value
        )
        self._wander_command = str(
            self.declare_parameter(
                'wander_command',
                _env_str('WANDER_COMMAND', 'ros2 run robertito wander_avoid_node'),
            ).value
        )
        self._clean_input_topic = str(
            self.declare_parameter(
                'clean_input_topic',
                _env_str('CLEAN_INPUT_TOPIC', '/robot_web/clean_enable'),
            ).value
        )
        self._clean_status_topic = str(
            self.declare_parameter(
                'clean_status_topic',
                _env_str('CLEAN_STATUS_TOPIC', '/robot_web/clean_status'),
            ).value
        )
        self._clean_command = str(
            self.declare_parameter(
                'clean_command',
                _env_str('CLEAN_COMMAND', 'ros2 run robertito clean_quick_node'),
            ).value
        )

        self._movement_pub = self.create_publisher(String, self._movement_topic, qos)
        self._head_tilt_pub = self.create_publisher(Float32, self._head_tilt_topic, qos)
        self._tts_pub = self.create_publisher(String, self._tts_topic, qos)
        self.create_subscription(Float32, self._tilt_input_topic, self._on_tilt_msg, qos)
        self.create_subscription(Float32, self._head_tilt_topic, self._on_head_tilt_msg, qos)
        self._last_forwarded_tilt: float | None = None
        self._last_forwarded_tilt_at: float = 0.0
        self.get_logger().info(
            f'Bridge de tilt listo: escuchando {self._tilt_input_topic} y {self._head_tilt_topic}, '
            f'enviando a {self._movement_topic}.'
        )

        self._vacuum_pub = self.create_publisher(Bool, self._vacuum_passthrough_topic, qos)
        self._brush_pub = self.create_publisher(Bool, self._brush_passthrough_topic, qos)
        self.create_subscription(Bool, self._vacuum_input_topic, self._handle_vacuum, qos)
        self.create_subscription(Bool, self._brush_input_topic, self._handle_brush, qos)

        self._vacuum_state: bool | None = None
        self._brush_state: bool | None = None

        self._wander_pub = self.create_publisher(Bool, self._wander_status_topic, qos)
        self.create_subscription(Bool, self._wander_input_topic, self._handle_wander, qos)
        self._wander_process: subprocess.Popen | None = None
        self._wander_state: bool = False
        self._wander_force_off_until: float = 0.0
        self._wander_monitor = self.create_timer(1.0, self._check_wander_process)
        self._wander_status_timer = self.create_timer(2.0, self._publish_wander_state)
        self._publish_wander_state()
        self._clean_pub = self.create_publisher(Bool, self._clean_status_topic, qos)
        self.create_subscription(Bool, self._clean_input_topic, self._handle_clean, qos)
        self._clean_process: subprocess.Popen | None = None
        self._clean_state: bool = False
        self._clean_force_off_until: float = 0.0
        self._clean_monitor = self.create_timer(1.0, self._check_clean_process)
        self._clean_status_timer = self.create_timer(2.0, self._publish_clean_state)
        self._publish_clean_state()
        self._wifi_monitor = self.create_timer(2.0, self._check_wifi_process)

        self._wifi_command = str(
            self.declare_parameter(
                'wifi_command',
                _env_str('WIFI_COMMAND', 'ros2 run robertito wifi_localization_node'),
            ).value
        )
        self._wifi_start_topic = str(
            self.declare_parameter(
                'wifi_start_topic',
                _env_str('WIFI_START_TOPIC', '/robot_web/start_wifi_localization'),
            ).value
        )
        self._wifi_process: subprocess.Popen | None = None
        self._wifi_state: bool = False
        self.create_subscription(Bool, self._wifi_start_topic, self._handle_wifi_request, qos)

    # region Tilt -----------------------------------------------------------
    def _on_tilt_msg(self, msg: Float32) -> None:
        self._forward_tilt_command(float(msg.data), publish_head_tilt=True)

    def _on_head_tilt_msg(self, msg: Float32) -> None:
        self._forward_tilt_command(float(msg.data), publish_head_tilt=False)

    def _forward_tilt_command(self, target: float, *, publish_head_tilt: bool) -> None:
        clamped = max(self._tilt_min, min(self._tilt_max, target))
        now = time.monotonic()
        if (
            self._last_forwarded_tilt is not None
            and abs(self._last_forwarded_tilt - clamped) < 0.05
            and (now - self._last_forwarded_tilt_at) < self._tilt_command_dedupe_window_sec
        ):
            return
        command = f'{self._tilt_prefix}{int(round(clamped))}'
        self._movement_pub.publish(String(data=command))
        if publish_head_tilt:
            self._head_tilt_pub.publish(Float32(data=clamped))
        self._last_forwarded_tilt = clamped
        self._last_forwarded_tilt_at = now
        self.get_logger().debug(f'Comando tilt -> {command}')

    # region Vacuum / brush -------------------------------------------------
    def _handle_vacuum(self, msg: Bool) -> None:
        desired = bool(msg.data)
        if self._vacuum_state is not None and desired == self._vacuum_state:
            return
        self._vacuum_state = desired
        command = f'VAC{1 if desired else 0}'
        self._vacuum_pub.publish(Bool(data=desired))
        self._movement_pub.publish(String(data=command))
        self.get_logger().info(
            f'Aspiradora {"encendida" if desired else "apagada"} (comando {command}).'
        )

    def _handle_brush(self, msg: Bool) -> None:
        desired = bool(msg.data)
        if self._brush_state is not None and desired == self._brush_state:
            return
        self._brush_state = desired
        command = f'BRUSH{1 if desired else 0}'
        self._brush_pub.publish(Bool(data=desired))
        self._movement_pub.publish(String(data=command))
        self.get_logger().info(
            f'Escobillas {"encendidas" if desired else "apagadas"} (comando {command}).'
        )

    # region Wander avoid -------------------------------------------------
    def _handle_wander(self, msg: Bool) -> None:
        desired = bool(msg.data)
        self.get_logger().info(f'Recibido wander_enable={desired}.')
        if desired == self._wander_state:
            return
        if desired:
            self._start_wander()
        else:
            self._stop_wander()

    def _start_wander(self) -> None:
        existing = self._count_wander_nodes()
        if existing > 0:
            self._wander_state = True
            self._publish_wander_state()
            self._set_wander_tilt(active=True)
            self._say('Modo navegación activado.')
            self.get_logger().warn(
                f'Wander Avoid ya está activo ({existing} instancia/s). No se inicia otro.'
            )
            return
        if self._wander_process and self._wander_process.poll() is None:
            self._wander_state = True
            self._publish_wander_state()
            return
        try:
            args = shlex.split(self._wander_command)
            self._wander_process = subprocess.Popen(args, start_new_session=True)
            self._wander_state = True
            self._publish_wander_state()
            self._set_wander_tilt(active=True)
            self._say('Modo navegación activado.')
            self.get_logger().info('Wander Avoid activado.')
        except Exception as exc:
            self._wander_state = False
            self._publish_wander_state()
            self.get_logger().error(f'No se pudo iniciar wander_avoid: {exc}')

    def _stop_wander(self) -> None:
        self._wander_force_off_until = time.monotonic() + 3.0
        self.get_logger().info('Solicitando apagado de Wander Avoid.')
        if self._wander_process and self._wander_process.poll() is None:
            try:
                os.killpg(self._wander_process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self._wander_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self._wander_process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        elif self._count_wander_nodes() > 0:
            self._kill_wander_nodes()
        self._wander_process = None
        self._wander_state = False
        self._publish_wander_state()
        self._set_wander_tilt(active=False)
        self._say('Modo navegación desactivado.')
        self.get_logger().info('Wander Avoid desactivado.')

    def _check_wander_process(self) -> None:
        if not self._wander_process:
            if time.monotonic() < self._wander_force_off_until:
                return
            if self._count_wander_nodes() > 0:
                if not self._wander_state:
                    self._wander_state = True
                    self._publish_wander_state()
            else:
                if self._wander_state:
                    self._wander_state = False
                    self._publish_wander_state()
            return
        if self._wander_process.poll() is None:
            return
        self._wander_process = None
        if self._wander_state:
            self._wander_state = False
            self._publish_wander_state()
            self.get_logger().warn('Wander Avoid se detuvo inesperadamente.')

    def _publish_wander_state(self) -> None:
        self._wander_pub.publish(Bool(data=bool(self._wander_state)))

    def _set_wander_tilt(self, *, active: bool) -> None:
        target = self._wander_tilt_deg if active else 0.0
        command = f'{self._tilt_prefix}{int(round(target))}'
        self._movement_pub.publish(String(data=command))
        self._head_tilt_pub.publish(Float32(data=target))

    def _say(self, text: str) -> None:
        self._tts_pub.publish(String(data=text))

    def _kill_wander_nodes(self) -> None:
        try:
            subprocess.run(
                ['ros2', 'node', 'kill', '/wander_avoid'],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass
        try:
            output = subprocess.check_output(
                ['/usr/bin/pgrep', '-f', 'robertito/wander_avoid_node'],
                text=True,
            )
            for line in output.splitlines():
                try:
                    pid = int(line.strip())
                except ValueError:
                    continue
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    continue
            time.sleep(0.2)
            output = subprocess.check_output(
                ['/usr/bin/pgrep', '-f', 'robertito/wander_avoid_node'],
                text=True,
            )
            for line in output.splitlines():
                try:
                    pid = int(line.strip())
                except ValueError:
                    continue
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    continue
        except Exception:
            pass

    def _count_wander_nodes(self) -> int:
        count = 0
        for name, namespace in self.get_node_names_and_namespaces():
            full = f'{namespace.rstrip("/")}/{name}'
            if full == '/wander_avoid':
                count += 1
        return count

    # region Clean quick -------------------------------------------------
    def _handle_clean(self, msg: Bool) -> None:
        desired = bool(msg.data)
        self.get_logger().info(f'Recibido clean_enable={desired}.')
        if desired == self._clean_state:
            return
        if desired:
            self._start_clean()
        else:
            self._stop_clean()

    def _start_clean(self) -> None:
        existing = self._count_clean_nodes()
        if existing > 0:
            self._clean_state = True
            self._publish_clean_state()
            self._say('Modo limpieza activado.')
            self.get_logger().warn(
                f'Clean Quick ya está activo ({existing} instancia/s). No se inicia otro.'
            )
            return
        if self._clean_process and self._clean_process.poll() is None:
            self._clean_state = True
            self._publish_clean_state()
            return
        try:
            args = shlex.split(self._clean_command)
            self._clean_process = subprocess.Popen(args, start_new_session=True)
            self._clean_state = True
            self._publish_clean_state()
            self._say('Modo limpieza activado.')
            self.get_logger().info('Clean Quick activado.')
        except Exception as exc:
            self._clean_state = False
            self._publish_clean_state()
            self.get_logger().error(f'No se pudo iniciar clean_quick: {exc}')

    def _stop_clean(self) -> None:
        self._clean_force_off_until = time.monotonic() + 3.0
        self.get_logger().info('Solicitando apagado de Clean Quick.')
        if self._clean_process and self._clean_process.poll() is None:
            try:
                os.killpg(self._clean_process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self._clean_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self._clean_process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        elif self._count_clean_nodes() > 0:
            self._kill_clean_nodes()
        self._clean_process = None
        self._clean_state = False
        self._publish_clean_state()
        self._say('Modo limpieza desactivado.')
        self.get_logger().info('Clean Quick desactivado.')

    def _check_clean_process(self) -> None:
        if not self._clean_process:
            if time.monotonic() < self._clean_force_off_until:
                return
            if self._count_clean_nodes() > 0:
                if not self._clean_state:
                    self._clean_state = True
                    self._publish_clean_state()
            else:
                if self._clean_state:
                    self._clean_state = False
                    self._publish_clean_state()
            return
        if self._clean_process.poll() is None:
            return
        self._clean_process = None
        if self._clean_state:
            self._clean_state = False
            self._publish_clean_state()
            self.get_logger().warn('Clean Quick se detuvo inesperadamente.')

    def _publish_clean_state(self) -> None:
        self._clean_pub.publish(Bool(data=bool(self._clean_state)))

    def _kill_clean_nodes(self) -> None:
        try:
            subprocess.run(
                ['ros2', 'node', 'kill', '/clean_quick'],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass
        try:
            output = subprocess.check_output(
                ['/usr/bin/pgrep', '-f', 'robertito/clean_quick_node'],
                text=True,
            )
            for line in output.splitlines():
                try:
                    pid = int(line.strip())
                except ValueError:
                    continue
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    continue
            time.sleep(0.2)
            output = subprocess.check_output(
                ['/usr/bin/pgrep', '-f', 'robertito/clean_quick_node'],
                text=True,
            )
            for line in output.splitlines():
                try:
                    pid = int(line.strip())
                except ValueError:
                    continue
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    continue
        except Exception:
            pass

    def _count_clean_nodes(self) -> int:
        count = 0
        for name, namespace in self.get_node_names_and_namespaces():
            full = f'{namespace.rstrip("/")}/{name}'
            if full == '/clean_quick':
                count += 1
        return count

    def _handle_wifi_request(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._wifi_process and self._wifi_process.poll() is None:
            return
        self._start_wifi_process()

    def _start_wifi_process(self) -> bool:
        if self._wifi_process and self._wifi_process.poll() is None:
            self._wifi_state = True
            return True
        self.get_logger().info('Iniciando nodo de localización WiFi.')
        try:
            args = shlex.split(self._wifi_command)
            self._wifi_process = subprocess.Popen(args, start_new_session=True)
            self._wifi_state = True
            self.get_logger().info('Nodo de localización WiFi iniciado.')
            return True
        except Exception as exc:
            self._wifi_process = None
            self._wifi_state = False
            self.get_logger().error(f'No se pudo iniciar wifi_localization_node: {exc}')
            return False

    def _check_wifi_process(self) -> None:
        if not self._wifi_process:
            self._wifi_state = False
            return
        if self._wifi_process.poll() is None:
            return
        self._wifi_process = None
        self._wifi_state = False

    def destroy_node(self) -> bool:
        try:
            self._stop_wander()
            self._stop_clean()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
