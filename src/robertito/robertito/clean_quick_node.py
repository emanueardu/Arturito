import fcntl
import json
import math
import random
import re
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool


class CleanQuick(Node):
    """Modo limpieza: activa aspiradora/escobillas y navega evitando obstáculos."""

    def __init__(self) -> None:
        super().__init__('clean_quick')

        self._cmd_vel_topic = str(
            self.declare_parameter('cmd_vel_topic', 'arturito/cmd_vel_clean').value
        )
        if not self._cmd_vel_topic.startswith('/'):
            self._cmd_vel_topic = f'/{self._cmd_vel_topic}'
        self._status_topic = str(
            self.declare_parameter('status_topic', 'arturito/status_raw').value
        )
        if not self._status_topic.startswith('/'):
            self._status_topic = f'/{self._status_topic}'
        self._mode_topic = str(
            self.declare_parameter(
                'mode_topic', '/assistant/mode/cleaning_quick'
            ).value
        )
        mode_enabled_param = bool(self.declare_parameter('mode_enabled', True).value)
        self._mode_enabled = False
        self._max_speed_mps = float(self.declare_parameter('max_speed_mps', 0.25).value)
        self._speed_pwm = float(self.declare_parameter('speed_pwm', 200.0).value)
        self._forward_speed = self._max_speed_mps * (self._speed_pwm / 255.0)
        self._reverse_speed = float(self.declare_parameter('reverse_speed_mps', 0.08).value)
        self._reverse_time = float(self.declare_parameter('reverse_time_sec', 1.0).value)
        self._turn_speed = float(self.declare_parameter('turn_speed_radps', 1.2).value)
        self._turn_angle_deg = float(self.declare_parameter('turn_angle_deg', 90.0).value)
        self.declare_parameter('obstacle_distance_cm', 10.0)
        self.declare_parameter('obstacle_min_distance_cm', 5.0)
        self._status_timeout = float(self.declare_parameter('status_timeout_sec', 1.0).value)
        self._use_yaw_turn = bool(self.declare_parameter('use_yaw_turn', True).value)
        self._turn_direction_mode = str(
            self.declare_parameter('turn_direction', 'random').value
        ).lower()
        self._dist_key = str(self.declare_parameter('dist_key', 'dist').value)
        self._yaw_key = str(self.declare_parameter('yaw_key', 'yaw').value)
        self._dist_unit = str(self.declare_parameter('dist_unit', 'mm').value).lower()
        self._obstacle_confirm_count = int(
            self.declare_parameter('obstacle_confirm_count', 2).value
        )
        self._obstacle_cooldown_sec = float(
            self.declare_parameter('obstacle_cooldown_sec', 1.0).value
        )
        self._stop_settle_sec = float(
            self.declare_parameter('stop_settle_sec', 0.1).value
        )
        self._advance_after_turn_sec = float(
            self.declare_parameter('advance_after_turn_sec', 1.0).value
        )
        self._tilt_topic = str(self.declare_parameter('tilt_topic', '/head/tilt').value)
        if not self._tilt_topic.startswith('/'):
            self._tilt_topic = f'/{self._tilt_topic}'
        self._tilt_active_deg = float(self.declare_parameter('tilt_active_deg', 45.0).value)
        self._tilt_idle_deg = float(self.declare_parameter('tilt_idle_deg', 0.0).value)
        self._tilt_republish_count = int(
            self.declare_parameter('tilt_republish_count', 5).value
        )
        self._tilt_republish_period_sec = float(
            self.declare_parameter('tilt_republish_period_sec', 0.2).value
        )
        self._service_retry_period_sec = float(
            self.declare_parameter('service_retry_period_sec', 1.0).value
        )
        self._vacuum_service = str(
            self.declare_parameter('vacuum_service', 'arturito/set_vacuum').value
        )
        self._brush_service = str(
            self.declare_parameter('brush_service', 'arturito/set_brush').value
        )
        self._service_wait_timeout = float(
            self.declare_parameter('service_wait_timeout_sec', 1.5).value
        )
        self._say_topic = str(
            self.declare_parameter('say_topic', '/assistant/say').value
        )
        self._tts_msg_enter = str(
            self.declare_parameter('tts_enter_message', 'Empezando limpieza').value
        )
        self._tts_msg_exit = str(
            self.declare_parameter('tts_exit_message', 'Limpieza terminada').value
        )
        self._mode_service_name = str(
            self.declare_parameter(
                'mode_service', '/assistant/mode/cleaning_quick/set'
            ).value
        )

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._tilt_pub = self.create_publisher(Float32, self._tilt_topic, 10)
        self._say_pub = self.create_publisher(String, self._say_topic, 10)
        # Autoridad de modo limpieza: este nodo publica clean_status para que
        # uart_node abra/cierre el gate sin depender del web bridge.
        self._clean_status_pub = self.create_publisher(
            Bool, '/robot_web/clean_status', 10
        )
        self.create_subscription(String, self._status_topic, self._on_status, 10)
        if self._mode_topic:
            self.create_subscription(Bool, self._mode_topic, self._on_mode_change, 10)
        self.create_service(SetBool, self._mode_service_name, self._on_mode_service)

        self._vacuum_client = self.create_client(SetBool, self._vacuum_service)
        self._brush_client = self.create_client(SetBool, self._brush_service)

        self._lock_file = self._acquire_singleton_lock()
        self._state = 'forward'
        self._turn_end_time: Optional[rclpy.time.Time] = None
        self._turn_timeout_deadline: Optional[rclpy.time.Time] = None
        self._turn_direction = 1.0
        self._last_dist_mm: Optional[float] = None
        self._last_yaw_deg: Optional[float] = None
        self._last_status_stamp: Optional[rclpy.time.Time] = None
        self._turn_start_yaw: Optional[float] = None
        self._turn_target_angle_deg = self._turn_angle_deg
        self._reverse_end_time: Optional[rclpy.time.Time] = None
        self._bumper_left = False
        self._bumper_right = False
        self._bumper_left_prev = False
        self._bumper_right_prev = False
        self._pending_bumper_turn: Optional[float] = None
        self._active_bumper_turn: Optional[float] = None
        self._obs_dir = 1.0
        self._obs_phase_end_time: Optional[rclpy.time.Time] = None
        self._obs_turn_start_yaw: Optional[float] = None
        self._obs_turn_target_deg: float = 0.0
        self._obs_confirm_counter = 0
        self._obs_cooldown_until: Optional[rclpy.time.Time] = None
        self._last_obstacle_dir: Optional[float] = None
        self._tilt_republish_remaining = 0
        self._tilt_next_republish_time: Optional[rclpy.time.Time] = None
        self._vacuum_desired = False
        self._brush_desired = False
        self._vacuum_confirmed = False
        self._brush_confirmed = False
        self._vacuum_last_request: Optional[rclpy.time.Time] = None
        self._brush_last_request: Optional[rclpy.time.Time] = None
        self._bumper_active = False

        self._timer = self.create_timer(0.1, self._on_timer)
        self._publish_stop()
        # Publicar estado inicial (False) para que el gate de uart_node arranque
        # explícitamente abierto a /cmd_vel principal.
        self._publish_clean_status()
        self._apply_mode(mode_enabled_param)
        self.get_logger().info('CleanQuick listo: navegando y limpiando.')

    def _on_status(self, msg: String) -> None:
        payload = msg.data
        data = self._parse_status(payload)
        dist = None
        yaw = None
        bumper_left = None
        bumper_right = None
        if data is not None:
            dist = data.get(self._dist_key)
            yaw = data.get(self._yaw_key)
            bumper_left = data.get('bL')
            bumper_right = data.get('bR')
        else:
            dist = self._extract_number(payload, self._dist_key)
            yaw = self._extract_number(payload, self._yaw_key)
            bumper_left = self._extract_number(payload, 'bL')
            bumper_right = self._extract_number(payload, 'bR')

        if dist is not None:
            try:
                raw_dist = float(dist)
            except (TypeError, ValueError):
                pass
            else:
                if self._dist_unit == 'cm':
                    raw_dist *= 10.0
                self._last_dist_mm = raw_dist
        if yaw is not None:
            try:
                self._last_yaw_deg = float(yaw)
            except (TypeError, ValueError):
                pass
        if bumper_left is not None:
            try:
                self._bumper_left = bool(int(bumper_left))
            except (TypeError, ValueError):
                pass
        if bumper_right is not None:
            try:
                self._bumper_right = bool(int(bumper_right))
            except (TypeError, ValueError):
                pass

        if (self._bumper_left and not self._bumper_left_prev) or (
            self._bumper_right and not self._bumper_right_prev
        ):
            if self._bumper_left and self._bumper_right:
                self._pending_bumper_turn = 1.0 if random.random() >= 0.5 else -1.0
            elif self._bumper_left:
                self._pending_bumper_turn = 1.0
            else:
                self._pending_bumper_turn = -1.0

        self._bumper_left_prev = self._bumper_left
        self._bumper_right_prev = self._bumper_right
        self._last_status_stamp = self.get_clock().now()

    def _on_mode_change(self, msg: Bool) -> None:
        if msg is None:
            return
        self._apply_mode(bool(msg.data))

    def _on_mode_service(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._apply_mode(bool(request.data))
        response.success = True
        response.message = 'Solicitud de modo limpieza rápida procesada.'
        return response

    def _apply_mode(self, enabled: bool) -> None:
        if enabled == self._mode_enabled:
            return
        self._mode_enabled = enabled
        # Publicar status inmediatamente al toggle, así el gate de uart_node
        # se actualiza sin depender del web bridge.
        self._publish_clean_status()
        now = self.get_clock().now()
        if enabled:
            self.get_logger().info("Modo limpieza rápida activado por comando de voz.")
            if self._tts_msg_enter:
                self._publish_tts(self._tts_msg_enter)
            self._state = 'forward'
            self._reset_obstacle_state()
            self._tilt_republish_remaining = max(0, self._tilt_republish_count - 1)
            if self._tilt_republish_remaining > 0:
                self._tilt_next_republish_time = now + Duration(
                    seconds=self._tilt_republish_period_sec
                )
            else:
                self._tilt_next_republish_time = None
            self._publish_tilt(self._tilt_active_deg)
            self._vacuum_desired = True
            self._brush_desired = True
            self._vacuum_confirmed = False
            self._brush_confirmed = False
            self._vacuum_last_request = None
            self._brush_last_request = None
            self._request_service_state(
                self._vacuum_client,
                True,
                'aspiradora',
                '_vacuum_confirmed',
                '_vacuum_last_request',
                now,
            )
            self._request_service_state(
                self._brush_client,
                True,
                'escobillas',
                '_brush_confirmed',
                '_brush_last_request',
                now,
            )
            return
        self.get_logger().info("Modo limpieza rápida desactivado por comando de voz.")
        if self._tts_msg_exit:
            self._publish_tts(self._tts_msg_exit)
        self._publish_stop()
        self._tilt_republish_remaining = 0
        self._tilt_next_republish_time = None
        self._publish_tilt(self._tilt_idle_deg)
        self._vacuum_desired = False
        self._brush_desired = False
        # NO resetear _vacuum_confirmed/_brush_confirmed a False acá: el
        # callback del enable los dejó en True. Si los reseteamos antes de
        # _request_service_state, la guard `confirmed == desired` (False==False)
        # hace early-return y el servicio jamás se llama → vacuum/brush quedan
        # prendidos hasta apagado externo. Bug confirmado por log de prod.
        # Nulificamos last_request para bypassear el retry period.
        self._vacuum_last_request = None
        self._brush_last_request = None
        self._request_service_state(
            self._vacuum_client,
            False,
            'aspiradora',
            '_vacuum_confirmed',
            '_vacuum_last_request',
            now,
        )
        self._request_service_state(
            self._brush_client,
            False,
            'escobillas',
            '_brush_confirmed',
            '_brush_last_request',
            now,
        )
        self._reset_obstacle_state()

    def _status_is_stale(self, now: rclpy.time.Time) -> bool:
        if self._last_status_stamp is None:
            return True
        age = now - self._last_status_stamp
        return age > Duration(seconds=self._status_timeout)

    def _start_turn(
        self,
        now: rclpy.time.Time,
        angle_deg: Optional[float] = None,
        direction: Optional[float] = None,
        state: Optional[str] = None,
    ) -> None:
        if direction is not None:
            self._turn_direction = direction
        self._turn_target_angle_deg = angle_deg if angle_deg is not None else self._turn_angle_deg
        self._turn_start_yaw = self._last_yaw_deg
        angle_rad = math.radians(self._turn_target_angle_deg)
        duration = angle_rad / max(abs(self._turn_speed), 1e-6)
        self._turn_timeout_deadline = now + Duration(seconds=duration)
        if not self._use_yaw_turn or self._turn_start_yaw is None:
            self._turn_end_time = self._turn_timeout_deadline
        else:
            self._turn_end_time = None
        self._state = state if state is not None else 'turning'

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_clean_status(self) -> None:
        """Publica el estado del modo limpieza para uart_node y otros listeners."""
        msg = Bool()
        msg.data = bool(self._mode_enabled)
        self._clean_status_pub.publish(msg)
        self.get_logger().info(f"clean_status publicado: {msg.data}")

    def _publish_tilt(self, tilt_deg: float) -> None:
        self._tilt_pub.publish(Float32(data=float(tilt_deg)))

    def _publish_tts(self, text: str) -> None:
        if self._say_pub is None:
            return
        self._say_pub.publish(String(data=str(text)))

    def _ensure_tilt_republish(self, now: rclpy.time.Time) -> None:
        if not self._mode_enabled or self._tilt_republish_remaining <= 0:
            return
        if self._tilt_next_republish_time is None or now < self._tilt_next_republish_time:
            return
        self._publish_tilt(self._tilt_active_deg)
        self._tilt_republish_remaining -= 1
        if self._tilt_republish_remaining > 0:
            self._tilt_next_republish_time = now + Duration(seconds=self._tilt_republish_period_sec)
        else:
            self._tilt_next_republish_time = None

    def _request_service_state(
        self,
        client: rclpy.client.Client,
        desired: bool,
        name: str,
        confirmed_attr: str,
        last_request_attr: str,
        now: rclpy.time.Time,
        wait_timeout_sec: Optional[float] = None,
    ) -> None:
        if getattr(self, confirmed_attr) == desired:
            return
        last_request = getattr(self, last_request_attr)
        if last_request is not None:
            if now < last_request + Duration(seconds=self._service_retry_period_sec):
                return
        setattr(self, last_request_attr, now)
        timeout = wait_timeout_sec if wait_timeout_sec is not None else self._service_wait_timeout
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(
                f'Servicio {name} no respondió tras {timeout:.2f}s'
            )
            return
        req = SetBool.Request()
        req.data = desired
        future = client.call_async(req)

        def _callback(fut: rclpy.task.Future) -> None:
            try:
                resp = fut.result()
                self.get_logger().info(f'{name}: {resp.message}')
                if resp.success:
                    setattr(self, confirmed_attr, desired)
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(f'Error llamando {name}: {exc}')

        future.add_done_callback(_callback)

    def _ensure_actuators(self, now: rclpy.time.Time) -> None:
        self._ensure_tilt_republish(now)
        self._request_service_state(
            self._vacuum_client,
            self._vacuum_desired,
            'aspiradora',
            '_vacuum_confirmed',
            '_vacuum_last_request',
            now,
            wait_timeout_sec=0.1,
        )
        self._request_service_state(
            self._brush_client,
            self._brush_desired,
            'escobillas',
            '_brush_confirmed',
            '_brush_last_request',
            now,
            wait_timeout_sec=0.1,
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if not self._mode_enabled:
            self._publish_stop()
            return
        self._ensure_actuators(now)
        if self._status_is_stale(now):
            self._publish_stop()
            self._state = 'stale'
            return
        self._maybe_start_bumper(now)

        if self._state == 'obs_stop':
            if self._obs_phase_end_time is not None and now >= self._obs_phase_end_time:
                self._start_obstacle_turn(now, 'obs_turn1')
        elif self._state == 'obs_turn1':
            if self._turn_should_finish(now):
                self._turn_end_time = None
                self._turn_timeout_deadline = None
                self._state = 'obs_forward1'
                self._obs_phase_end_time = now + Duration(seconds=self._advance_after_turn_sec)
        elif self._state == 'obs_forward1':
            if self._obs_phase_end_time is not None and now >= self._obs_phase_end_time:
                self._start_obstacle_turn(now, 'obs_turn2')
        elif self._state == 'obs_turn2':
            if self._turn_should_finish(now):
                self._turn_end_time = None
                self._turn_timeout_deadline = None
                self._state = 'forward'
                self._obs_phase_end_time = None
                self._obs_confirm_counter = 0
                self._obs_cooldown_until = now + Duration(seconds=self._obstacle_cooldown_sec)
        elif self._state == 'turning':
            if self._turn_should_finish(now):
                self._finish_bumper_turn()
        elif self._state == 'backing':
            if self._reverse_end_time is not None and now >= self._reverse_end_time:
                self._reverse_end_time = None
                direction = self._active_bumper_turn
                self._active_bumper_turn = None
                if direction is not None:
                    self._start_turn(now, angle_deg=45.0, direction=direction, state='turning')
                else:
                    self._state = 'forward'
        elif self._state == 'forward':
            if self._should_confirm_obstacle(now):
                self._obs_confirm_counter += 1
                if self._obs_confirm_counter >= self._obstacle_confirm_count:
                    self._obs_confirm_counter = 0
                    self._start_obstacle_maneuver(now)
            else:
                self._obs_confirm_counter = 0

        twist = Twist()
        if self._state in ('turning', 'obs_turn1', 'obs_turn2'):
            twist.angular.z = self._turn_direction * self._turn_speed
        elif self._state == 'backing':
            twist.linear.x = -abs(self._reverse_speed)
        elif self._state in ('forward', 'obs_forward1'):
            twist.linear.x = self._forward_speed

        self._cmd_pub.publish(twist)

    def _turn_should_finish(self, now: rclpy.time.Time) -> bool:
        if self._turn_completed_by_yaw():
            return True
        if (self._turn_end_time is not None) and now >= self._turn_end_time:
            return True
        if self._turn_timeout_deadline is not None and now >= self._turn_timeout_deadline:
            return True
        return False

    def _finish_bumper_turn(self) -> None:
        self._turn_end_time = None
        self._turn_timeout_deadline = None
        self._turn_start_yaw = None
        self._state = 'forward'
        self._bumper_active = False
        self._obs_confirm_counter = 0

    def _maybe_start_bumper(self, now: rclpy.time.Time) -> None:
        if self._pending_bumper_turn is None or self._bumper_active:
            return
        direction = self._pending_bumper_turn
        self._pending_bumper_turn = None
        self._cancel_obstacle_state()
        self._bumper_active = True
        self._active_bumper_turn = direction
        self._state = 'backing'
        self._reverse_end_time = now + Duration(seconds=self._reverse_time)

    def _should_confirm_obstacle(self, now: rclpy.time.Time) -> bool:
        if self._obs_cooldown_until is not None and now < self._obs_cooldown_until:
            return False
        dist = self._last_dist_mm
        if dist is None or not math.isfinite(dist) or dist == 0.0:
            return False
        return 50.0 <= dist <= 150.0

    def _start_obstacle_maneuver(self, now: rclpy.time.Time) -> None:
        self._obs_dir = self._choose_obstacle_direction()
        self._obs_confirm_counter = 0
        self._obs_phase_end_time = now + Duration(seconds=self._stop_settle_sec)
        self._state = 'obs_stop'
        self._obs_turn_start_yaw = None
        self._obs_turn_target_deg = 0.0

    def _start_obstacle_turn(self, now: rclpy.time.Time, phase: str) -> None:
        self._start_turn(now, angle_deg=90.0, direction=self._obs_dir, state=phase)
        self._obs_turn_start_yaw = self._turn_start_yaw
        self._obs_turn_target_deg = self._turn_target_angle_deg
        self._obs_phase_end_time = None

    def _cancel_obstacle_state(self) -> None:
        if self._state.startswith('obs'):
            self._state = 'forward'
        self._obs_phase_end_time = None
        self._obs_confirm_counter = 0
        self._obs_turn_start_yaw = None
        self._obs_turn_target_deg = 0.0
        self._turn_end_time = None
        self._turn_timeout_deadline = None
        self._turn_start_yaw = None
        self._obs_cooldown_until = None

    def _reset_obstacle_state(self) -> None:
        self._obs_phase_end_time = None
        self._obs_confirm_counter = 0
        self._obs_cooldown_until = None
        self._obs_turn_start_yaw = None
        self._obs_turn_target_deg = 0.0
        self._turn_end_time = None
        self._turn_timeout_deadline = None
        self._turn_start_yaw = None
        self._obs_dir = 1.0
        self._last_obstacle_dir = None

    def _choose_obstacle_direction(self) -> float:
        if self._last_obstacle_dir is None:
            if self._turn_direction_mode == 'right':
                direction = -1.0
            else:
                direction = 1.0
        else:
            direction = -self._last_obstacle_dir
        self._last_obstacle_dir = direction
        return direction

    def _turn_completed_by_yaw(self) -> bool:
        if self._turn_start_yaw is None or self._last_yaw_deg is None:
            return False
        delta = self._angular_distance_deg(self._turn_start_yaw, self._last_yaw_deg)
        if self._turn_direction > 0.0:
            return delta >= self._turn_target_angle_deg
        return delta <= -self._turn_target_angle_deg

    @staticmethod
    def _angular_distance_deg(start: float, end: float) -> float:
        return (end - start + 180.0) % 360.0 - 180.0

    def _parse_status(self, payload: str) -> Optional[dict]:
        raw = payload.strip()
        if not raw:
            return None
        if raw[0] != '{':
            start = raw.find('{')
            end = raw.rfind('}')
            if start != -1 and end != -1 and end > start:
                raw = raw[start:end + 1]
            else:
                raw = '{' + raw
                if not raw.endswith('}'):
                    raw += '}'
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return None

    @staticmethod
    def _extract_number(payload: str, key: str) -> Optional[float]:
        pattern = rf'"?{re.escape(key)}"?\s*:\s*(-?\d+(?:\.\d+)?)'
        match = re.search(pattern, payload)
        if not match:
            return None
        try:
            return float(match.group(1))
        except ValueError:
            return None

    def _set_service(self, client: rclpy.client.Client, desired: bool, name: str) -> None:
        if not client.wait_for_service(timeout_sec=self._service_wait_timeout):
            self.get_logger().warn(
                f'Servicio {name} no respondió tras {self._service_wait_timeout:.2f}s'
            )
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

    def _acquire_singleton_lock(self):
        lock_path = Path('/tmp/clean_quick.lock')
        lock_file = lock_path.open('a+')
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            lock_file.close()
            raise RuntimeError('Ya existe una instancia de clean_quick en ejecucion.')
        return lock_file

    def destroy_node(self) -> bool:
        self._timer.cancel()
        self._publish_stop()
        self._publish_tilt(self._tilt_idle_deg)
        self._set_service(self._vacuum_client, False, 'aspiradora')
        self._set_service(self._brush_client, False, 'escobillas')
        if hasattr(self, '_lock_file') and self._lock_file:
            try:
                fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
            except OSError:
                pass
            try:
                self._lock_file.close()
            except OSError:
                pass
        self.get_logger().info('Modo limpieza desactivado.')
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CleanQuick()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
