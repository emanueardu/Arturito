from __future__ import annotations

import json
import math
import statistics
import time
from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

try:
    from smbus2 import SMBus
except ImportError:  # pragma: no cover - handled at runtime on the robot
    SMBus = None  # type: ignore


def normalize_180(angle_deg: float) -> float:
    """Normalize angle to (-180, 180]."""
    angle = (float(angle_deg) + 180.0) % 360.0 - 180.0
    if angle <= -180.0:
        return 180.0
    return angle


def normalize_360(angle_deg: float) -> float:
    """Normalize angle to [0, 360)."""
    return float(angle_deg) % 360.0


def angular_delta_deg(current: float, reference: float) -> float:
    """Return shortest signed angular delta current-reference in (-180, 180]."""
    return normalize_180(float(current) - float(reference))


def _int16_le(lsb: int, msb: int) -> int:
    value = ((msb & 0xFF) << 8) | (lsb & 0xFF)
    return value - 65536 if value & 0x8000 else value


def _int16_be(msb: int, lsb: int) -> int:
    value = ((msb & 0xFF) << 8) | (lsb & 0xFF)
    return value - 65536 if value & 0x8000 else value


class MagnetometerReader(Protocol):
    model: str

    def configure(self) -> None:
        ...

    def read_xy(self) -> Tuple[float, float]:
        ...

    def close(self) -> None:
        ...


@dataclass
class SMBusMagnetometerReader:
    bus: object
    address: int
    model: str

    def __post_init__(self) -> None:
        self.model = self.model.lower().strip()
        self.configure()

    def configure(self) -> None:
        if self.model == "qmc5883l":
            # Soft reset/set-reset period, then continuous 200 Hz, 8G, 512 OSR.
            self.bus.write_byte_data(self.address, 0x0B, 0x01)
            self.bus.write_byte_data(self.address, 0x09, 0x1D)
        elif self.model == "hmc5883l":
            # 8-sample average, 15 Hz, normal measurement; gain 1.3G; continuous.
            self.bus.write_byte_data(self.address, 0x00, 0x70)
            self.bus.write_byte_data(self.address, 0x01, 0x20)
            self.bus.write_byte_data(self.address, 0x02, 0x00)
        elif self.model == "lis3mdl":
            # Ultra-high performance XY, 20 Hz, +/-4 gauss, continuous mode.
            self.bus.write_byte_data(self.address, 0x20, 0x70)
            self.bus.write_byte_data(self.address, 0x21, 0x00)
            self.bus.write_byte_data(self.address, 0x22, 0x00)
            self.bus.write_byte_data(self.address, 0x23, 0x0C)
        else:
            raise ValueError(f"Unsupported magnetometer model: {self.model}")
        time.sleep(0.01)

    def read_xy(self) -> Tuple[float, float]:
        if self.model == "qmc5883l":
            data = self.bus.read_i2c_block_data(self.address, 0x00, 6)
            return float(_int16_le(data[0], data[1])), float(_int16_le(data[2], data[3]))
        if self.model == "hmc5883l":
            data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
            # HMC5883L order is X, Z, Y.
            return float(_int16_be(data[0], data[1])), float(_int16_be(data[4], data[5]))
        if self.model == "lis3mdl":
            data = self.bus.read_i2c_block_data(self.address, 0x28 | 0x80, 6)
            return float(_int16_le(data[0], data[1])), float(_int16_le(data[2], data[3]))
        raise ValueError(f"Unsupported magnetometer model: {self.model}")

    def close(self) -> None:
        # The SMBus handle is owned by CompassNode and closed once at shutdown.
        pass


def _probe_model(bus: object, address: int, requested: str) -> str:
    model = requested.lower().strip()
    if model != "auto":
        return model
    try:
        who_am_i = bus.read_byte_data(address, 0x0F)
        if who_am_i == 0x3D:
            return "lis3mdl"
    except Exception:
        pass
    if address == 0x0D:
        return "qmc5883l"
    # Address 0x1e is commonly HMC5883L, but some LIS3MDL boards can use it.
    return "hmc5883l"


class CompassNode(Node):
    """I2C magnetometer compass with home-orientation services."""

    def __init__(self) -> None:
        super().__init__("compass_node")

        self._i2c_bus = int(self.declare_parameter("i2c_bus", 1).value)
        self._i2c_address = int(self.declare_parameter("i2c_address", 0x1E).value)
        self._sensor_model = str(self.declare_parameter("sensor_model", "auto").value)
        self._frame_id = str(self.declare_parameter("frame_id", "base_link").value)
        self._publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)

        self._yaw_offset_deg = float(self.declare_parameter("yaw_offset_deg", 0.0).value)
        self._yaw_sign = float(self.declare_parameter("yaw_sign", 1.0).value)
        self._declination_deg = float(self.declare_parameter("declination_deg", 0.0).value)
        self._hard_x = float(self.declare_parameter("hard_iron_offset_x", 0.0).value)
        self._hard_y = float(self.declare_parameter("hard_iron_offset_y", 0.0).value)
        self._soft_x = float(self.declare_parameter("soft_iron_scale_x", 1.0).value)
        self._soft_y = float(self.declare_parameter("soft_iron_scale_y", 1.0).value)

        self._lowpass_alpha = self._clamp(
            float(self.declare_parameter("lowpass_alpha", 0.25).value), 0.0, 1.0
        )
        self._median_window = max(1, int(self.declare_parameter("median_window", 5).value))
        self._angular_tolerance_deg = float(
            self.declare_parameter("angular_tolerance_deg", 5.0).value
        )
        self._deadband_deg = max(0.0, float(self.declare_parameter("deadband_deg", 1.5).value))
        self._stale_timeout_sec = max(
            0.05, float(self.declare_parameter("stale_timeout_sec", 0.5).value)
        )
        self._max_jump_deg = max(0.0, float(self.declare_parameter("max_jump_deg", 45.0).value))
        self._yaw_output_mode = str(
            self.declare_parameter("yaw_output_mode", "-180_180").value
        )

        self._reader: Optional[MagnetometerReader] = None
        self._bus = None
        self._reader_model: Optional[str] = None
        self._last_error: Optional[str] = None
        self._last_error_log_time = 0.0
        self._next_i2c_retry_time = 0.0
        self._i2c_retry_period_sec = 2.0
        self._last_valid_time: Optional[float] = None
        self._last_raw_yaw: Optional[float] = None
        self._filtered_yaw: Optional[float] = None
        self._yaw_samples: list[float] = []
        self._home_yaw_deg: Optional[float] = None

        self._yaw_pub = self.create_publisher(Float32, "/robertito/compass/yaw", 10)
        self._home_delta_pub = self.create_publisher(
            Float32, "/robertito/compass/home_delta", 10
        )
        self._status_pub = self.create_publisher(String, "/robertito/compass/status", 10)

        self.create_service(Trigger, "/robertito/compass/set_home", self._srv_set_home)
        self.create_service(Trigger, "/robertito/compass/clear_home", self._srv_clear_home)
        self.create_service(Trigger, "/robertito/compass/is_home", self._srv_is_home)

        self._ensure_reader(time.monotonic())
        period = 1.0 / max(self._publish_rate_hz, 0.1)
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "Compass node ready: source=magnetometer_i2c "
            f"bus={self._i2c_bus} address=0x{self._i2c_address:02x} "
            f"model={self._reader_model or self._sensor_model} frame={self._frame_id} "
            f"yaw_output_mode={self._yaw_output_mode}"
        )

    def _ensure_bus(self) -> bool:
        if self._bus is not None:
            return True
        if SMBus is None:
            self._last_error = "smbus2 is not installed"
            return False
        self.get_logger().info(f"Opening I2C bus /dev/i2c-{self._i2c_bus}")
        self._bus = SMBus(self._i2c_bus)
        return True

    def _ensure_reader(self, now: float) -> None:
        if self._reader is not None:
            return
        if now < self._next_i2c_retry_time:
            return
        try:
            if not self._ensure_bus():
                self._next_i2c_retry_time = now + self._i2c_retry_period_sec
                self._throttled_warn(f"{self._last_error}, retrying in 2s")
                return
            model = _probe_model(self._bus, self._i2c_address, self._sensor_model)
            self._reader = SMBusMagnetometerReader(self._bus, self._i2c_address, model)
            self._reader_model = model
            self._last_error = None
            self.get_logger().info(
                f"Magnetometer connected model={model} addr=0x{self._i2c_address:02x}"
            )
        except Exception as exc:  # pylint: disable=broad-except
            self._reader = None
            self._reader_model = None
            self._last_error = f"I2C magnetometer unavailable: {exc}"
            self._next_i2c_retry_time = now + self._i2c_retry_period_sec
            self._throttled_warn(f"{self._last_error}, retrying in 2s")

    def _on_timer(self) -> None:
        now = time.monotonic()
        self._ensure_reader(now)
        if self._reader is not None:
            try:
                raw_x, raw_y = self._reader.read_xy()
                yaw = self._compute_yaw(raw_x, raw_y)
                if self._accept_yaw(yaw, now):
                    self._last_error = None
            except Exception as exc:  # pylint: disable=broad-except
                self._last_error = str(exc)
                self._reader = None
                self._reader_model = None
                self._next_i2c_retry_time = now + self._i2c_retry_period_sec
                self._throttled_warn(f"I2C read failed, retrying in 2s: {exc}")

        self._publish(now)

    def _compute_yaw(self, raw_x: float, raw_y: float) -> float:
        x = (float(raw_x) - self._hard_x) * self._soft_x
        y = (float(raw_y) - self._hard_y) * self._soft_y
        yaw = math.degrees(math.atan2(y, x))
        yaw = self._yaw_sign * yaw + self._declination_deg + self._yaw_offset_deg
        return normalize_180(yaw)

    def _accept_yaw(self, yaw: float, now: float) -> bool:
        if self._last_raw_yaw is not None and self._max_jump_deg > 0.0:
            jump = abs(angular_delta_deg(yaw, self._last_raw_yaw))
            if jump > self._max_jump_deg:
                self._last_error = f"rejected yaw jump {jump:.1f} deg"
                self._throttled_warn(self._last_error)
                return False

        self._last_raw_yaw = yaw
        self._last_valid_time = now
        self._yaw_samples.append(yaw)
        if len(self._yaw_samples) > self._median_window:
            self._yaw_samples = self._yaw_samples[-self._median_window :]

        target = self._median_angle(self._yaw_samples, yaw)
        if self._filtered_yaw is None:
            self._filtered_yaw = target
            return True

        delta = angular_delta_deg(target, self._filtered_yaw)
        if abs(delta) <= self._deadband_deg:
            return True
        self._filtered_yaw = normalize_180(self._filtered_yaw + self._lowpass_alpha * delta)
        return True

    @staticmethod
    def _median_angle(samples: list[float], reference: float) -> float:
        if not samples:
            return reference
        unwrapped = [reference + angular_delta_deg(sample, reference) for sample in samples]
        return normalize_180(statistics.median(unwrapped))

    def _publish(self, now: float) -> None:
        stale = self._is_stale(now)
        valid = self._filtered_yaw is not None and not stale
        yaw_out = self._format_yaw(self._filtered_yaw) if self._filtered_yaw is not None else None
        home_delta = self._home_delta() if valid and self._home_yaw_deg is not None else None
        is_home = (
            home_delta is not None and abs(home_delta) <= self._angular_tolerance_deg
        )

        if valid and yaw_out is not None:
            self._yaw_pub.publish(Float32(data=float(yaw_out)))
        if home_delta is not None:
            self._home_delta_pub.publish(Float32(data=float(home_delta)))

        status = {
            "ok": bool(valid and self._last_error is None),
            "source": "magnetometer_i2c",
            "sensor_model": self._reader_model or self._sensor_model,
            "yaw_deg": yaw_out,
            "home_set": self._home_yaw_deg is not None,
            "home_yaw_deg": self._format_yaw(self._home_yaw_deg)
            if self._home_yaw_deg is not None
            else None,
            "home_delta_deg": home_delta,
            "is_home": bool(is_home),
            "stale": bool(stale),
            "error": self._last_error,
        }
        self._status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def _srv_set_home(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        now = time.monotonic()
        if self._filtered_yaw is None or self._is_stale(now):
            resp.success = False
            resp.message = json.dumps({"ok": False, "error": "no valid compass reading"})
            return resp
        self._home_yaw_deg = self._filtered_yaw
        resp.success = True
        resp.message = json.dumps(
            {"ok": True, "home_yaw_deg": self._format_yaw(self._home_yaw_deg)}
        )
        return resp

    def _srv_clear_home(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        self._home_yaw_deg = None
        resp.success = True
        resp.message = json.dumps({"ok": True, "home_set": False})
        return resp

    def _srv_is_home(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        now = time.monotonic()
        delta = self._home_delta() if self._home_yaw_deg is not None else None
        is_home = (
            self._filtered_yaw is not None
            and not self._is_stale(now)
            and delta is not None
            and abs(delta) <= self._angular_tolerance_deg
        )
        resp.success = bool(is_home)
        resp.message = json.dumps(
            {
                "ok": bool(self._filtered_yaw is not None and not self._is_stale(now)),
                "home_set": self._home_yaw_deg is not None,
                "is_home": bool(is_home),
                "home_delta_deg": delta,
                "angular_tolerance_deg": self._angular_tolerance_deg,
                "stale": self._is_stale(now),
                "error": self._last_error,
            },
            sort_keys=True,
        )
        return resp

    def _home_delta(self) -> Optional[float]:
        if self._filtered_yaw is None or self._home_yaw_deg is None:
            return None
        return angular_delta_deg(self._filtered_yaw, self._home_yaw_deg)

    def _is_stale(self, now: float) -> bool:
        if self._last_valid_time is None:
            return True
        return now - self._last_valid_time > self._stale_timeout_sec

    def _format_yaw(self, yaw: Optional[float]) -> Optional[float]:
        if yaw is None:
            return None
        if self._yaw_output_mode == "0_360":
            return normalize_360(yaw)
        return normalize_180(yaw)

    def _throttled_warn(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_time >= 1.0:
            self.get_logger().warn(message)
            self._last_error_log_time = now

    def _close_i2c(self) -> None:
        self._reader = None
        self._reader_model = None
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def destroy_node(self) -> bool:
        self._close_i2c()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CompassNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
