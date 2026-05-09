"""Helpers de movimiento corto para el orchestrator.

DISEÑO:
  * Gestos EXPRESIVOS (engaged_micro_motion: tiny_advance, mini_spin)
    son simétricos: van y vuelven a la pose inicial. Sin odometría, así
    no acumulamos drift por los gestos de "vida" frecuentes.
  * Gestos de SAFETY (reverse en cliff/startled, bump_reaction) NO vuelven:
    si tocaste algo o algo se acerca, hay que retroceder Y QUEDARSE
    retrocedido. Volver al punto de impacto rompe la semántica de safety.
"""
from __future__ import annotations

import logging
import math
import random
import threading
import time
from typing import Callable, Optional

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# Pausa entre fases (ej: terminó reverse → pausa → empieza advance de retorno).
# Le da tiempo al motor a frenar limpiamente antes de invertir el sentido.
_PHASE_PAUSE_S = 0.15


class MotorGestures:
    def __init__(
        self,
        cmd_vel_publisher,
        head_tilt_publisher,
        linear_speed_mps: float = 0.08,
        angular_speed_radps: float = 0.5,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._cmd_vel_pub = cmd_vel_publisher
        self._tilt_pub = head_tilt_publisher
        self._linear = float(linear_speed_mps)
        self._angular = float(angular_speed_radps)
        self._logger = logger or logging.getLogger(__name__)
        self._busy_lock = threading.Lock()
        self._busy: bool = False

    @property
    def busy(self) -> bool:
        return self._busy

    # ─────────── Twist helpers ───────────

    def _publish_twist(self, lin: float = 0.0, ang: float = 0.0) -> None:
        if self._cmd_vel_pub is None:
            return
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self._cmd_vel_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_twist(0.0, 0.0)

    def _run_in_thread(self, fn: Callable[[], None], name: str) -> bool:
        with self._busy_lock:
            if self._busy:
                self._logger.debug(f"motor busy, skipping {name}")
                return False
            self._busy = True

        def _wrapper() -> None:
            try:
                fn()
            except Exception:  # pragma: no cover
                self._logger.exception(f"error en gesto {name}")
            finally:
                self._stop()
                with self._busy_lock:
                    self._busy = False

        threading.Thread(target=_wrapper, name=f"motor-{name}", daemon=True).start()
        return True

    # ─────────── Phases (single direction) ───────────

    def _drive_phase(self, lin: float, ang: float, duration: float) -> None:
        """Publica un Twist constante por `duration` segundos y para."""
        self._publish_twist(lin, ang)
        time.sleep(max(0.0, duration))
        self._publish_twist(0.0, 0.0)

    # ─────────── Gestos de SAFETY (no vuelven) ───────────

    def reverse(self, distance_m: float, duration_s_max: float = 2.0) -> bool:
        """SAFETY: retrocede `distance_m` y se queda. NO vuelve.

        Usado por cliff/startled. Si algo invade el espacio o aparece un
        borde, la semántica correcta es alejarse — no volver al punto de
        peligro al instante.
        """
        if self._linear <= 0:
            return False
        duration = min(abs(distance_m) / self._linear, duration_s_max)

        def run() -> None:
            self._drive_phase(-self._linear, 0.0, duration)

        return self._run_in_thread(run, "reverse")

    def bump_reaction(self, side: str, distance_m: float, angle_deg: float) -> bool:
        """SAFETY: retrocede + spin de evasión. No vuelve a la pose original.

        Side dicta el sentido del spin:
          - "left"  → gira a la derecha (alejarse del bumper L)
          - "right" → gira a la izquierda
          - resto   → random
        """
        if self._linear <= 0 or self._angular <= 0:
            return False
        rev_duration = min(abs(distance_m) / self._linear, 2.0)
        spin_rad = math.radians(abs(angle_deg))
        spin_duration = min(spin_rad / self._angular, 3.0)
        if side == "left":
            spin_dir = -1
        elif side == "right":
            spin_dir = +1
        else:
            spin_dir = random.choice((-1, +1))
        ang = self._angular * spin_dir

        def run() -> None:
            self._drive_phase(-self._linear, 0.0, rev_duration)
            time.sleep(_PHASE_PAUSE_S)
            self._drive_phase(0.0, ang, spin_duration)

        return self._run_in_thread(run, "bump_reaction")

    # ─────────── Gestos EXPRESIVOS (simétricos: vuelven al origen) ───────────

    def tiny_advance(self, distance_m: float, duration_s_max: float = 1.5) -> bool:
        """ENGAGED: avanza y vuelve. Net displacement = 0."""
        if self._linear <= 0:
            return False
        duration = min(abs(distance_m) / self._linear, duration_s_max)

        def run() -> None:
            self._drive_phase(self._linear, 0.0, duration)
            time.sleep(_PHASE_PAUSE_S)
            self._drive_phase(-self._linear, 0.0, duration)

        return self._run_in_thread(run, "tiny_advance")

    def spin(self, angle_deg: float, direction: int = 1, symmetric: bool = True) -> bool:
        """Spin de `angle_deg` en `direction`.

        symmetric=True (default, gestos expresivos): gira y vuelve.
        symmetric=False (raro, no usado por defecto): gira y se queda.
        """
        if self._angular <= 0:
            return False
        angle_rad = math.radians(abs(angle_deg))
        duration = min(angle_rad / self._angular, 3.0)
        sign = 1.0 if direction >= 0 else -1.0
        ang = self._angular * sign

        def run() -> None:
            self._drive_phase(0.0, ang, duration)
            if symmetric:
                time.sleep(_PHASE_PAUSE_S)
                self._drive_phase(0.0, -ang, duration)

        return self._run_in_thread(run, f"spin_{direction}")

    def mini_spin_left(self) -> bool:
        return self.spin(angle_deg=random.uniform(8.0, 14.0), direction=+1, symmetric=True)

    def mini_spin_right(self) -> bool:
        return self.spin(angle_deg=random.uniform(8.0, 14.0), direction=-1, symmetric=True)

    # ─────────── Tilt helpers (no acumula drift) ───────────

    def tilt_to(self, angle_deg: float) -> None:
        if self._tilt_pub is None:
            return
        self._tilt_pub.publish(Float32(data=float(angle_deg)))

    def tilt_random(self, min_deg: float = -15.0, max_deg: float = 15.0) -> None:
        self.tilt_to(random.uniform(min_deg, max_deg))
