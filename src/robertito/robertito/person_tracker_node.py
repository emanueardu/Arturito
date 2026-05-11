"""Person tracker reescrito (PR5).

Diseño:
- Tilt sigue cara verticalmente con low-pass smoothing (alpha=0.3).
- angular.z chiquito (max 0.5 rad/s) con clamp por delta_yaw acumulado ±45°.
- linear.x = 0 SIEMPRE — el tracker NO mueve al robot hacia adelante/atrás.
- Yaw RELATIVO al yaw_at_start fijado al activar (el sensor tiene drift).
- Control TOTAL por orchestrator vía /person_tracker/active (Bool).
- Mute inmediato por bumper o /assistant/mode/cleaning_quick=true.
- Al desactivarse: estado RETURNING gira de vuelta al yaw_at_start.
"""
import json
from enum import Enum
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from vision_msgs.msg import Detection2D, Detection2DArray


class TrackerState(Enum):
    IDLE = 0       # No hace nada (estado por default y al cancelar).
    ACTIVE = 1     # Siguiendo cara: angular.z + tilt.
    RETURNING = 2  # Volviendo a yaw_at_start tras desactivar.


class ArturitoPersonTracker(Node):
    """Tracker reactivo de cara: tilt + giro acotado, sin avance."""

    def __init__(self) -> None:
        super().__init__('arturito_person_tracker')

        # ─────────── Topics ───────────
        self._detection_topic = str(
            self.declare_parameter('detection_topic', 'arturito/camera/faces').value
        )
        self._cmd_vel_topic = str(
            self.declare_parameter('cmd_vel_topic', '/cmd_vel/person_track').value
        )
        if not self._cmd_vel_topic.startswith('/'):
            self._cmd_vel_topic = f'/{self._cmd_vel_topic}'
        self._tilt_topic = str(
            self.declare_parameter('tilt_topic', '/head/tilt').value
        )
        if not self._tilt_topic.startswith('/'):
            self._tilt_topic = '/' + self._tilt_topic
        self._status_raw_topic = str(
            self.declare_parameter('status_raw_topic', '/arturito/status_raw').value
        )
        self._active_topic = str(
            self.declare_parameter('active_topic', '/person_tracker/active').value
        )
        self._cleaning_mode_topic = str(
            self.declare_parameter(
                'cleaning_mode_topic', '/assistant/mode/cleaning_quick'
            ).value
        )

        # ─────────── Frame / detección ───────────
        self._frame_width = float(self.declare_parameter('frame_width', 640.0).value)
        self._frame_height = float(self.declare_parameter('frame_height', 480.0).value)
        self._flip_horizontal = bool(
            self.declare_parameter('flip_horizontal', False).value
        )
        self._flip_vertical = bool(
            self.declare_parameter('flip_vertical', False).value
        )
        self._lost_timeout = float(
            self.declare_parameter('lost_timeout_sec', 1.5).value
        )

        # ─────────── Tilt (vertical) ───────────
        self._neutral_tilt = float(
            self.declare_parameter('neutral_tilt_deg', 10.0).value
        )
        self._tilt_gain = float(self.declare_parameter('tilt_gain_deg', 30.0).value)
        self._tilt_min = float(self.declare_parameter('tilt_min_deg', -20.0).value)
        self._tilt_max = float(self.declare_parameter('tilt_max_deg', 45.0).value)
        self._tilt_smoothing_alpha = float(
            self.declare_parameter('tilt_smoothing_alpha', 0.3).value
        )

        # ─────────── Yaw acotado (horizontal) ───────────
        self._angular_gain = float(
            self.declare_parameter('angular_gain', 0.6).value
        )
        self._max_angular_speed = float(
            self.declare_parameter('max_angular_speed', 0.5).value
        )
        self._max_delta_yaw_deg = float(
            self.declare_parameter('max_delta_yaw_deg', 45.0).value
        )
        self._return_threshold_deg = float(
            self.declare_parameter('return_threshold_deg', 2.0).value
        )
        self._return_angular_speed = float(
            self.declare_parameter('return_angular_speed', 0.4).value
        )

        # ─────────── Deadbands ───────────
        self._deadband_x = float(self.declare_parameter('deadband_x', 0.08).value)
        self._deadband_y = float(self.declare_parameter('deadband_y', 0.05).value)

        # ─────────── Control loop ───────────
        self._control_period = float(
            self.declare_parameter('control_period_sec', 0.05).value
        )
        # 'start_active' se preserva por compat con el launch, ya no controla.
        self.declare_parameter('start_active', False)

        # ─────────── Pub/Sub ───────────
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._tilt_pub = self.create_publisher(Float32, self._tilt_topic, 10)
        self.create_subscription(
            Detection2DArray, self._detection_topic, self._on_detections, 10
        )
        self.create_subscription(
            String, self._status_raw_topic, self._on_status_raw, 10
        )
        self.create_subscription(
            Bool, self._active_topic, self._on_active_signal, 10
        )
        self.create_subscription(
            Bool, self._cleaning_mode_topic, self._on_cleaning_mode, 10
        )

        # ─────────── Estado runtime ───────────
        self._state = TrackerState.IDLE
        self._tracker_active_signal = False
        self._cleaning_active = False

        self._current_yaw_deg = 0.0
        self._yaw_at_start_deg: Optional[float] = None
        self._bumper_l = False
        self._bumper_r = False
        self._bumper_l_prev = False
        self._bumper_r_prev = False

        self._last_detection: Optional[Tuple[float, float]] = None
        self._last_detection_time = None
        self._smoothed_tilt: float = self._neutral_tilt

        # PR6: estado para RETURNING robusto. Logueamos progreso cada 1s y
        # forzamos IDLE si pasan returning_timeout_sec sin completar el retorno
        # (evita atasco por drift acumulado del sensor).
        self._returning_start_time = self.get_clock().now()
        self._last_returning_log = self.get_clock().now()
        self._returning_timeout_sec = float(
            self.declare_parameter('returning_timeout_sec', 8.0).value
        )

        # Inicializar tilt en posición neutra y arrancar timer.
        self._publish_tilt(self._neutral_tilt)
        self._control_timer = self.create_timer(
            self._control_period, self._control_tick
        )
        self.get_logger().info(
            f"PersonTracker listo (state={self._state.name}, "
            f"detection={self._detection_topic}, active_signal={self._active_topic}, "
            f"max_delta_yaw=±{self._max_delta_yaw_deg}°)"
        )

    # ─────────── Callbacks ───────────

    def _on_status_raw(self, msg: String) -> None:
        """Parsea yaw + bumpers del JSON crudo del firmware ESP32."""
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        try:
            if 'yaw' in data:
                self._current_yaw_deg = float(data['yaw'])
            if 'bL' in data:
                self._bumper_l = bool(int(data['bL']))
            if 'bR' in data:
                self._bumper_r = bool(int(data['bR']))
        except (TypeError, ValueError):
            pass

    def _on_active_signal(self, msg: Bool) -> None:
        """Señal de activación enviada por presence_orchestrator."""
        self._tracker_active_signal = bool(msg.data)

    def _on_cleaning_mode(self, msg: Bool) -> None:
        """Mute total si cleaning_quick=true (igual que el orchestrator)."""
        self._cleaning_active = bool(msg.data)

    def _on_detections(self, msg: Detection2DArray) -> None:
        """Guarda la última detección de cara (lógica de control va en _control_tick)."""
        selection = self._select_detection(msg)
        if selection is None:
            return
        detection, _area = selection
        center_x = float(detection.bbox.center.position.x)
        center_y = float(detection.bbox.center.position.y)
        self._last_detection = (center_x, center_y)
        self._last_detection_time = self.get_clock().now()

    def _select_detection(
        self, msg: Detection2DArray
    ) -> Optional[Tuple[Detection2D, float]]:
        """Devuelve la detección de mayor área (más cercana)."""
        best: Optional[Detection2D] = None
        best_area = -1.0
        for det in msg.detections:
            area = float(det.bbox.size_x) * float(det.bbox.size_y)
            if area > best_area:
                best = det
                best_area = area
        if best is None:
            return None
        return best, best_area

    # ─────────── Helpers ───────────

    def _delta_yaw_deg(self) -> float:
        """Diferencia angular entre yaw actual y yaw_at_start, en (-180, 180]."""
        if self._yaw_at_start_deg is None:
            return 0.0
        diff = (
            self._current_yaw_deg - self._yaw_at_start_deg + 180.0
        ) % 360.0 - 180.0
        return diff

    def _enter_state(self, new_state: TrackerState, reason: str = '') -> None:
        """Transición de estado con log.

        PR6: al entrar a RETURNING se resetea el cronómetro del timeout y el
        último log de progreso, así cada sesión RETURNING tiene su propia
        ventana de 8s independiente.
        """
        if new_state == TrackerState.RETURNING:
            now = self.get_clock().now()
            self._returning_start_time = now
            self._last_returning_log = now
        old = self._state
        self._state = new_state
        self.get_logger().info(
            f"Tracker: {old.name} -> {new_state.name} "
            f"(reason={reason}, delta_yaw={self._delta_yaw_deg():.1f}°)"
        )

    def _publish_zero_velocity(self) -> None:
        """Publica Twist cero (paro motores)."""
        self._cmd_pub.publish(Twist())

    def _publish_tilt(self, target: float) -> None:
        """Publica tilt absoluto sin smoothing (uso interno)."""
        self._tilt_pub.publish(Float32(data=float(target)))

    def _publish_smoothed_tilt(self, target: float) -> None:
        """Low-pass filter del tilt antes de publicar."""
        alpha = self._tilt_smoothing_alpha
        self._smoothed_tilt = alpha * target + (1.0 - alpha) * self._smoothed_tilt
        self._publish_tilt(self._smoothed_tilt)

    # ─────────── Control loop ───────────

    def _control_tick(self) -> None:
        """Loop principal del FSM (período = control_period_sec)."""
        now = self.get_clock().now()

        # MUTE 1: cleaning_quick=true → forzar IDLE.
        if self._cleaning_active:
            if self._state != TrackerState.IDLE:
                self._publish_zero_velocity()
                self._yaw_at_start_deg = None
                self._enter_state(TrackerState.IDLE, reason='cleaning_quick activo')
            self._bumper_l_prev = self._bumper_l
            self._bumper_r_prev = self._bumper_r
            return

        # MUTE 2: bumper edge (flanco ascendente) → cancel inmediato.
        bumper_event = (
            (self._bumper_l and not self._bumper_l_prev)
            or (self._bumper_r and not self._bumper_r_prev)
        )
        self._bumper_l_prev = self._bumper_l
        self._bumper_r_prev = self._bumper_r
        if bumper_event and self._state != TrackerState.IDLE:
            self._publish_zero_velocity()
            self._yaw_at_start_deg = None
            self._enter_state(TrackerState.IDLE, reason='bumper')
            return

        # Transición IDLE → ACTIVE (cuando llega la señal del orchestrator).
        if self._state == TrackerState.IDLE:
            if self._tracker_active_signal:
                self._yaw_at_start_deg = self._current_yaw_deg
                self._smoothed_tilt = self._neutral_tilt
                self._enter_state(TrackerState.ACTIVE, reason='active_signal=true')
            else:
                return

        # Transición ACTIVE → RETURNING (orchestrator quita la señal).
        if self._state == TrackerState.ACTIVE and not self._tracker_active_signal:
            self._enter_state(TrackerState.RETURNING, reason='active_signal=false')

        # Estado RETURNING: girar hacia yaw_at_start.
        if self._state == TrackerState.RETURNING:
            delta = self._delta_yaw_deg()
            elapsed = (now - self._returning_start_time).nanoseconds / 1e9

            # Log de progreso cada 1s para diagnosticar atascos.
            log_age = (now - self._last_returning_log).nanoseconds / 1e9
            if log_age >= 1.0:
                self.get_logger().info(
                    f"RETURNING: delta_yaw={delta:.1f}°, elapsed={elapsed:.1f}s"
                )
                self._last_returning_log = now

            # Timeout de seguridad: si tras returning_timeout_sec no llegamos
            # al umbral, forzamos IDLE para evitar quedar girando indefinidamente
            # (drift del yaw o contaminación por gestos paralelos).
            if elapsed > self._returning_timeout_sec:
                self.get_logger().warn(
                    f"RETURNING timeout tras {elapsed:.1f}s con delta={delta:.1f}°. "
                    "Forzando IDLE para evitar acumulación."
                )
                self._publish_zero_velocity()
                self._publish_smoothed_tilt(self._neutral_tilt)
                self._yaw_at_start_deg = None
                self._enter_state(TrackerState.IDLE, reason='return timeout')
                return

            # Llegamos al umbral.
            if abs(delta) < self._return_threshold_deg:
                self._publish_zero_velocity()
                self._publish_smoothed_tilt(self._neutral_tilt)
                self._yaw_at_start_deg = None
                self._enter_state(TrackerState.IDLE, reason='return completed')
                return

            # Si delta > 0, hay que girar en sentido negativo para volver a 0.
            sign = -1.0 if delta > 0 else 1.0
            twist = Twist()
            twist.angular.z = sign * self._return_angular_speed
            self._cmd_pub.publish(twist)
            self._publish_smoothed_tilt(self._neutral_tilt)
            return

        # Estado ACTIVE: tracking real.
        if self._state == TrackerState.ACTIVE:
            self._tracking_tick(now)

    def _tracking_tick(self, now) -> None:
        """Lógica de tracking durante state=ACTIVE."""
        target_visible = (
            self._last_detection is not None
            and self._last_detection_time is not None
            and (now - self._last_detection_time)
            < Duration(seconds=self._lost_timeout)
        )
        if not target_visible:
            # No hay cara: parar movimiento, mantener tilt.
            self._publish_zero_velocity()
            return

        center_x, center_y = self._last_detection
        offset_x = (
            (center_x - self._frame_width / 2.0)
            / max(self._frame_width / 2.0, 1.0)
        )
        offset_y = (
            (center_y - self._frame_height / 2.0)
            / max(self._frame_height / 2.0, 1.0)
        )
        if self._flip_horizontal:
            offset_x *= -1.0
        if self._flip_vertical:
            offset_y *= -1.0

        # Angular con doble clamp: velocidad max + delta_yaw acumulado.
        angular = 0.0
        if abs(offset_x) > self._deadband_x:
            desired = self._angular_gain * offset_x
            desired = max(
                -self._max_angular_speed,
                min(self._max_angular_speed, desired),
            )
            delta = self._delta_yaw_deg()
            if delta > self._max_delta_yaw_deg and desired > 0:
                angular = 0.0  # Ya giró +max_delta, no permite más derecha.
            elif delta < -self._max_delta_yaw_deg and desired < 0:
                angular = 0.0  # Ya giró -max_delta, no permite más izquierda.
            else:
                angular = desired

        # Tilt con smoothing.
        if abs(offset_y) > self._deadband_y:
            target_tilt = self._neutral_tilt - (self._tilt_gain * offset_y)
            target_tilt = max(self._tilt_min, min(self._tilt_max, target_tilt))
        else:
            target_tilt = self._smoothed_tilt

        twist = Twist()
        twist.angular.z = angular
        # linear.x = 0 SIEMPRE (decisión de diseño: el tracker no avanza).
        self._cmd_pub.publish(twist)
        self._publish_smoothed_tilt(target_tilt)

    def destroy_node(self) -> bool:
        """Restaurar estado neutro al apagar."""
        self.get_logger().info('Deteniendo person tracker; restaurando estado neutro')
        self._publish_zero_velocity()
        self._publish_tilt(self._neutral_tilt)
        if hasattr(self, '_control_timer') and self._control_timer is not None:
            self._control_timer.cancel()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArturitoPersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
