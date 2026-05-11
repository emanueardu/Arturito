import fcntl
import json
import random
import re
from enum import Enum
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool


class CleanState(Enum):
    IDLE = 0
    FORWARD = 1
    SLOW = 2
    BACKING = 3
    TURNING = 4


class CleanQuick(Node):
    """Modo limpieza: activa aspiradora/escobillas y navega evitando obstáculos.

    FSM reactivo (PR4.2): estados IDLE/FORWARD/SLOW/BACKING/TURNING. Sin yaw,
    sin lógica de obstáculos por confirmación. El ultrasonic frontal anticipa
    paredes (slow / stop) y los bumpers + back+turn responden a colisión.
    """

    def __init__(self) -> None:
        super().__init__('clean_quick')

        # ─────────── Topics y modo ───────────
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
        self._ultrasonic_topic = str(
            self.declare_parameter('ultrasonic_topic', '/arturito/ultrasonic').value
        )
        self._mode_topic = str(
            self.declare_parameter(
                'mode_topic', '/assistant/mode/cleaning_quick'
            ).value
        )
        mode_enabled_param = bool(self.declare_parameter('mode_enabled', True).value)
        self._mode_enabled = False

        # ─────────── Velocidades históricas (mantenidas por compat) ───────────
        # NO usadas por el FSM nuevo, que usa los params *_speed_mps específicos
        # de abajo. Se preservan para no romper YAMLs/launches existentes.
        self._max_speed_mps = float(self.declare_parameter('max_speed_mps', 0.25).value)
        self._speed_pwm = float(self.declare_parameter('speed_pwm', 200.0).value)
        self._forward_speed_legacy = self._max_speed_mps * (self._speed_pwm / 255.0)
        self._reverse_speed = float(self.declare_parameter('reverse_speed_mps', 0.08).value)
        self._reverse_time = float(self.declare_parameter('reverse_time_sec', 1.0).value)
        self._turn_speed = float(self.declare_parameter('turn_speed_radps', 1.2).value)

        # ─────────── Params status_raw / parsing ───────────
        self.declare_parameter('obstacle_distance_cm', 10.0)
        self._status_timeout = float(
            self.declare_parameter('status_timeout_sec', 1.0).value
        )
        self._dist_key = str(self.declare_parameter('dist_key', 'dist').value)
        self._yaw_key = str(self.declare_parameter('yaw_key', 'yaw').value)
        self._dist_unit = str(self.declare_parameter('dist_unit', 'mm').value).lower()

        # ─────────── Tilt ───────────
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

        # ─────────── Servicios actuadores ───────────
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

        # ─────────── TTS ───────────
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

        # ─────────── FSM params (PR4.2) ───────────
        self._fsm_forward_speed = float(
            self.declare_parameter('forward_speed_mps', 0.141).value
        )
        self._fsm_slow_speed = float(
            self.declare_parameter('slow_speed_mps', 0.07).value
        )
        self._fsm_back_speed = float(
            self.declare_parameter('back_speed_mps', -0.08).value
        )
        self._fsm_turn_speed = float(
            self.declare_parameter('fsm_turn_speed_radps', 0.9).value
        )
        self._fsm_us_slow_m = float(
            self.declare_parameter('us_slow_m', 0.30).value
        )
        self._fsm_us_stop_m = float(
            self.declare_parameter('us_stop_m', 0.15).value
        )
        self._fsm_us_resume_m = float(
            self.declare_parameter('us_resume_m', 0.40).value
        )
        self._fsm_back_duration_s = float(
            self.declare_parameter('back_duration_s', 0.4).value
        )
        self._fsm_turn_duration_s = float(
            self.declare_parameter('turn_duration_s', 0.25).value
        )
        self._fsm_random_turn_gap_s = float(
            self.declare_parameter('random_turn_gap_s', 20.0).value
        )

        # ─────────── Pubs / Subs / Services ───────────
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._tilt_pub = self.create_publisher(Float32, self._tilt_topic, 10)
        self._say_pub = self.create_publisher(String, self._say_topic, 10)
        self._clean_status_pub = self.create_publisher(
            Bool, '/robot_web/clean_status', 10
        )
        self.create_subscription(String, self._status_topic, self._on_status, 10)
        self.create_subscription(
            Range, self._ultrasonic_topic, self._on_ultrasonic, 10
        )
        if self._mode_topic:
            self.create_subscription(Bool, self._mode_topic, self._on_mode_change, 10)
        self.create_service(SetBool, self._mode_service_name, self._on_mode_service)

        self._vacuum_client = self.create_client(SetBool, self._vacuum_service)
        self._brush_client = self.create_client(SetBool, self._brush_service)

        # ─────────── Estado runtime ───────────
        self._lock_file = self._acquire_singleton_lock()
        self._last_dist_mm: Optional[float] = None
        self._last_yaw_deg: Optional[float] = None
        self._last_status_stamp: Optional[rclpy.time.Time] = None
        self._bumper_left = False
        self._bumper_right = False
        self._bumper_left_prev = False
        self._bumper_right_prev = False
        self._tilt_republish_remaining = 0
        self._tilt_next_republish_time: Optional[rclpy.time.Time] = None
        self._vacuum_desired = False
        self._brush_desired = False
        self._vacuum_confirmed = False
        self._brush_confirmed = False
        self._vacuum_last_request: Optional[rclpy.time.Time] = None
        self._brush_last_request: Optional[rclpy.time.Time] = None

        # ─────────── Estado FSM (PR4.2) ───────────
        now0 = self.get_clock().now()
        self._fsm_state = CleanState.IDLE
        self._fsm_state_enter_time = now0
        self._fsm_last_bumper_side: Optional[str] = None
        self._fsm_last_random_turn_time = now0
        self._fsm_ultrasonic_m = float('inf')
        self._fsm_ultrasonic_last_stamp: Optional[rclpy.time.Time] = None

        self._timer = self.create_timer(0.1, self._on_timer)
        self._publish_stop()
        self._publish_clean_status()
        self._apply_mode(mode_enabled_param)
        self.get_logger().info('CleanQuick listo: navegando y limpiando.')

    # ─────────── Callbacks de entrada ───────────

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

        # Bumper edge detection lo hace _fsm_tick, no acá. Solo registramos
        # los flags actuales y dejamos que el FSM compare con prev.
        self._last_status_stamp = self.get_clock().now()

    def _on_ultrasonic(self, msg: Range) -> None:
        """Actualiza distancia frontal medida por ultrasonic.

        Filtra ruido: si range fuera del rango válido del sensor (5cm-4m),
        ignora el sample y no actualiza el cache.
        """
        try:
            r = float(msg.range)
        except (TypeError, ValueError):
            return
        if not (0.05 <= r <= 4.0):
            return
        self._fsm_ultrasonic_m = r
        self._fsm_ultrasonic_last_stamp = self.get_clock().now()

    def _on_mode_change(self, msg: Bool) -> None:
        if msg is None:
            return
        self._apply_mode(bool(msg.data))

    def _on_mode_service(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self._apply_mode(bool(request.data))
        response.success = True
        response.message = 'Solicitud de modo limpieza rápida procesada.'
        return response

    # ─────────── Modo: enter/exit ───────────

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
            self._fsm_reset(now)
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
        self._fsm_reset(now)
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

    def _status_is_stale(self, now: rclpy.time.Time) -> bool:
        if self._last_status_stamp is None:
            return True
        age = now - self._last_status_stamp
        return age > Duration(seconds=self._status_timeout)

    # ─────────── Helpers de publicación ───────────

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

    # ─────────── Servicios actuadores ───────────

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

    # ─────────── Timer principal ───────────

    def _on_timer(self) -> None:
        """Timer callback @10Hz: tickea actuadores + FSM."""
        now = self.get_clock().now()
        self._ensure_actuators(now)
        self._ensure_tilt_republish(now)
        if not self._mode_enabled:
            self._fsm_state = CleanState.IDLE
            return
        if self._status_is_stale(now):
            self._publish_stop()
            return
        # Ultrasonic stale check (>2s sin datos = treat as inf, no bloquear)
        if self._fsm_ultrasonic_last_stamp is not None:
            dt = (now - self._fsm_ultrasonic_last_stamp).nanoseconds / 1e9
            if dt > 2.0:
                self._fsm_ultrasonic_m = float('inf')
        self._fsm_tick(now)

    # ─────────── FSM ───────────

    def _fsm_reset(self, now: rclpy.time.Time) -> None:
        """Vuelve el FSM a IDLE y sincroniza el edge-detector de bumpers.

        Se llama al activar y desactivar el modo limpieza. Sincronizar
        bumper_*_prev evita falsos triggers si el bumper ya estaba
        presionado al activar.
        """
        self._fsm_state = CleanState.IDLE
        self._fsm_state_enter_time = now
        self._fsm_last_bumper_side = None
        self._fsm_last_random_turn_time = now
        self._bumper_left_prev = self._bumper_left
        self._bumper_right_prev = self._bumper_right

    def _fsm_tick(self, now: rclpy.time.Time) -> None:
        """Ejecuta una iteración del FSM y publica el Twist correspondiente.

        Estados:
          IDLE    → FORWARD al activar.
          FORWARD → SLOW    si ultrasonic < us_slow_m.
                  → BACKING si bumper_l/r o ultrasonic < us_stop_m.
                  → TURNING si pasan random_turn_gap_s sin obstáculos.
          SLOW    → FORWARD si ultrasonic > us_resume_m.
                  → BACKING si bumper o ultrasonic < us_stop_m.
          BACKING → TURNING tras back_duration_s.
          TURNING → FORWARD tras turn_duration_s.
        """
        state = self._fsm_state
        elapsed_s = (now - self._fsm_state_enter_time).nanoseconds / 1e9

        if state == CleanState.IDLE:
            self._fsm_enter(CleanState.FORWARD, now)
            state = CleanState.FORWARD

        bumper_event = (
            self._bumper_left and not self._bumper_left_prev
        ) or (
            self._bumper_right and not self._bumper_right_prev
        )

        # Transiciones desde FORWARD
        if state == CleanState.FORWARD:
            if bumper_event:
                if self._bumper_left and self._bumper_right:
                    self._fsm_last_bumper_side = 'both'
                elif self._bumper_left:
                    self._fsm_last_bumper_side = 'L'
                else:
                    self._fsm_last_bumper_side = 'R'
                self._fsm_enter(CleanState.BACKING, now)
            elif self._fsm_ultrasonic_m < self._fsm_us_stop_m:
                self._fsm_last_bumper_side = 'random'
                self._fsm_enter(CleanState.BACKING, now)
            elif self._fsm_ultrasonic_m < self._fsm_us_slow_m:
                self._fsm_enter(CleanState.SLOW, now)
            else:
                random_gap_elapsed = (
                    now - self._fsm_last_random_turn_time
                ).nanoseconds / 1e9
                if random_gap_elapsed > self._fsm_random_turn_gap_s:
                    self._fsm_last_bumper_side = 'random'
                    self._fsm_enter(CleanState.TURNING, now)

        # Transiciones desde SLOW
        elif state == CleanState.SLOW:
            if bumper_event:
                if self._bumper_left and self._bumper_right:
                    self._fsm_last_bumper_side = 'both'
                elif self._bumper_left:
                    self._fsm_last_bumper_side = 'L'
                else:
                    self._fsm_last_bumper_side = 'R'
                self._fsm_enter(CleanState.BACKING, now)
            elif self._fsm_ultrasonic_m < self._fsm_us_stop_m:
                self._fsm_last_bumper_side = 'random'
                self._fsm_enter(CleanState.BACKING, now)
            elif self._fsm_ultrasonic_m > self._fsm_us_resume_m:
                self._fsm_enter(CleanState.FORWARD, now)

        # Transiciones desde BACKING
        elif state == CleanState.BACKING:
            if elapsed_s >= self._fsm_back_duration_s:
                self._fsm_enter(CleanState.TURNING, now)

        # Transiciones desde TURNING
        elif state == CleanState.TURNING:
            if elapsed_s >= self._fsm_turn_duration_s:
                self._fsm_enter(CleanState.FORWARD, now)
                self._fsm_last_random_turn_time = now  # resetea random gap

        # Construir y publicar Twist según estado actual
        self._fsm_publish_twist()

        # Update bumper prev para edge detection del próximo tick.
        self._bumper_left_prev = self._bumper_left
        self._bumper_right_prev = self._bumper_right

    def _fsm_enter(self, new_state: 'CleanState', now: rclpy.time.Time) -> None:
        """Transición de estado con log y reset de timer."""
        old_state = self._fsm_state
        self._fsm_state = new_state
        self._fsm_state_enter_time = now
        self.get_logger().info(
            f"FSM: {old_state.name} -> {new_state.name} "
            f"(bumper_side={self._fsm_last_bumper_side}, "
            f"us={self._fsm_ultrasonic_m:.2f}m)"
        )

    def _fsm_publish_twist(self) -> None:
        """Construye y publica Twist según estado actual."""
        t = Twist()
        if self._fsm_state == CleanState.FORWARD:
            t.linear.x = self._fsm_forward_speed
        elif self._fsm_state == CleanState.SLOW:
            t.linear.x = self._fsm_slow_speed
        elif self._fsm_state == CleanState.BACKING:
            t.linear.x = self._fsm_back_speed
        elif self._fsm_state == CleanState.TURNING:
            direction = self._fsm_compute_turn_direction()
            t.angular.z = self._fsm_turn_speed * direction
        # CleanState.IDLE → Twist cero (frenado)
        self._cmd_pub.publish(t)

    def _fsm_compute_turn_direction(self) -> float:
        """Devuelve +1.0 (izquierda) o -1.0 (derecha) según último side.

        Bumper izq → giro a la derecha (lado contrario).
        Bumper der → giro a la izquierda.
        Both / random / None → giro random.
        """
        # Signos calibrados empíricamente para este robot (mayo 2026):
        # tests con bumper izq+der confirmaron que +0.9 produce giro IZQ
        # en cmd_vel directo, pero el efecto neto despues del FSM era
        # girar al MISMO lado del bumper. Invertimos para que el robot
        # gire siempre al lado CONTRARIO del bumper (esquivar el obstaculo).
        if self._fsm_last_bumper_side == 'L':
            return +1.0  # bumper izq -> giro derecha (lado contrario)
        elif self._fsm_last_bumper_side == 'R':
            return -1.0  # bumper der -> giro izquierda
        return 1.0 if random.random() < 0.5 else -1.0

    # ─────────── Helpers de parsing ───────────

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
