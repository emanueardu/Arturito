import curses
import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool


class ArturitoTeleop(Node):
    """Keyboard teleoperation to drive Arturito and toggle actuators."""

    def __init__(self) -> None:
        super().__init__('arturito_teleop')

        self._linear_step = float(self.declare_parameter('linear_speed', 0.18).value)
        self._angular_step = float(self.declare_parameter('angular_speed', 1.2).value)
        self._tilt_step = float(self.declare_parameter('tilt_step_deg', 5.0).value)
        self._tilt_min = float(self.declare_parameter('tilt_min_deg', -20.0).value)
        self._tilt_max = float(self.declare_parameter('tilt_max_deg', 45.0).value)
        self._tilt_topic = str(self.declare_parameter('tilt_topic', 'head/tilt').value)
        if not self._tilt_topic.startswith('/'):
            self._tilt_topic = f'/{self._tilt_topic}'

        raw_cmd_topic = str(self.declare_parameter('cmd_vel_topic', '/cmd_vel/teleop').value)
        if not raw_cmd_topic.startswith('/'):
            raw_cmd_topic = f'/{raw_cmd_topic}'
        self._cmd_pub = self.create_publisher(Twist, raw_cmd_topic, 10)
        self._tilt_pub = self.create_publisher(Float32, self._tilt_topic, 10)
        self._vacuum_client = self.create_client(SetBool, 'arturito/set_vacuum')
        self._brush_client = self.create_client(SetBool, 'arturito/set_brush')

        self._current_linear = 0.0
        self._current_angular = 0.0
        self._current_tilt = 0.0
        self._vacuum_enabled = False
        self._brush_enabled = False
        self._lock = threading.Lock()

        self._publish_timer = self.create_timer(0.1, self._publish_commands)
        # Publish neutral commands on startup so hardware is initialized.
        self._publish_commands()
        self.get_logger().info('Arturito keyboard teleop ready. Press ? for help, q to quit.')

    # region Command helpers -------------------------------------------------
    def _publish_commands(self) -> None:
        with self._lock:
            twist = Twist()
            twist.linear.x = self._current_linear
            twist.angular.z = self._current_angular
        self._cmd_pub.publish(twist)

    def _stop_motion(self) -> None:
        with self._lock:
            self._current_linear = 0.0
            self._current_angular = 0.0
        self._publish_commands()
        self.get_logger().info('Movimiento detenido')

    def _update_motion(self, linear: float, angular: float) -> None:
        with self._lock:
            self._current_linear = linear
            self._current_angular = angular
        self._publish_commands()

    def _adjust_tilt(self, delta_deg: float) -> None:
        with self._lock:
            self._current_tilt = float(
                min(self._tilt_max, max(self._tilt_min, self._current_tilt + delta_deg))
            )
            tilt = self._current_tilt
        self._tilt_pub.publish(Float32(data=tilt))
        self.get_logger().info(f'Tilt -> {tilt:.1f}°')

    def _toggle_service(self, client: rclpy.client.Client, desired: bool, name: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warn(f'Servicio {name} no está disponible aún')
            return
        req = SetBool.Request()
        req.data = desired
        future = client.call_async(req)

        def _callback(fut: rclpy.task.Future) -> None:
            try:
                resp = fut.result()
                self.get_logger().info(f'{name}: {resp.message}')
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(f'Error llamando {name}: {exc}')

        future.add_done_callback(_callback)

    # endregion --------------------------------------------------------------

    def keyboard_loop(self) -> None:
        curses.wrapper(self._curses_main)

    def _curses_main(self, screen) -> None:
        screen.nodelay(True)
        screen.keypad(True)
        screen.clear()
        self._draw_help(screen)
        last_key_time = time.time()

        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except KeyboardInterrupt:
                break

            key = screen.getch()
            if key == -1:
                # si no se presiona nada por 1 s, detiene movimiento
                if (time.time() - last_key_time) > 1.0:
                    self._stop_motion()
                continue

            last_key_time = time.time()
            if key in (ord('q'), ord('Q')):
                self.get_logger().info('Saliendo teleop...')
                break
            if key in (ord(' '), ord('s')):
                self._stop_motion()
                continue
            if key in (ord('?'), ord('h'), ord('H')):
                self._draw_help(screen)
                continue

            if key == curses.KEY_UP:
                self._update_motion(self._linear_step, 0.0)
            elif key == curses.KEY_DOWN:
                self._update_motion(-self._linear_step, 0.0)
            elif key == curses.KEY_LEFT:
                self._update_motion(0.0, self._angular_step)
            elif key == curses.KEY_RIGHT:
                self._update_motion(0.0, -self._angular_step)
            elif key in (ord('v'), ord('V')):
                self._vacuum_enabled = not self._vacuum_enabled
                self._toggle_service(self._vacuum_client, self._vacuum_enabled, 'aspiradora')
            elif key in (ord('b'), ord('B')):
                self._brush_enabled = not self._brush_enabled
                self._toggle_service(self._brush_client, self._brush_enabled, 'cepillo')
            elif key in (ord('w'), curses.KEY_PPAGE):  # PageUp
                self._adjust_tilt(self._tilt_step)
            elif key in (ord('x'), curses.KEY_NPAGE):  # PageDown
                self._adjust_tilt(-self._tilt_step)
            else:
                continue

        self._stop_motion()

    def _draw_help(self, screen) -> None:
        help_lines = [
            'Controles Arturito Teleop',
            '',
            'Flecha ↑ : Avanzar',
            'Flecha ↓ : Retroceder',
            'Flecha ← : Giro Izquierdo (sobre eje)',
            'Flecha → : Giro Derecho (sobre eje)',
            'ESPACIO / s : Stop de movimiento',
            'v : Toggle aspiradora',
            'b : Toggle cepillo',
            'w / PageUp : Subir carcasa (tilt +)',
            'x / PageDown : Bajar carcasa (tilt -)',
            'h / ? : Mostrar ayuda',
            'q : Salir',
        ]
        screen.clear()
        for idx, line in enumerate(help_lines):
            screen.addstr(idx, 0, line)
        screen.refresh()

    def destroy_node(self) -> bool:
        if hasattr(self, '_publish_timer') and self._publish_timer is not None:
            self._publish_timer.cancel()
        self._stop_motion()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArturitoTeleop()
    try:
        node.keyboard_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
