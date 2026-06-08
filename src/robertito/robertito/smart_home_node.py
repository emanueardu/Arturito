import json
import os
import threading
import time
from datetime import datetime, time as dt_time
from typing import Any, Dict, Optional
from zoneinfo import ZoneInfo

import rclpy
import tinytuya
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


def _expand_path(path: str) -> str:
    return os.path.expanduser((path or "").strip())


def _default_config_path() -> str:
    return os.path.join(get_package_share_directory("robertito"), "config", "smart_home.yaml")


def _load_json(path: str) -> Any:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Config YAML invalido: {path}")
    return data


def _extract_devices(payload: Any) -> list[Dict[str, Any]]:
    if isinstance(payload, list):
        return [item for item in payload if isinstance(item, dict)]
    if isinstance(payload, dict):
        for key in ("devices", "result"):
            value = payload.get(key)
            if isinstance(value, list):
                return [item for item in value if isinstance(item, dict)]
        if all(isinstance(value, dict) for value in payload.values()):
            return [value for value in payload.values() if isinstance(value, dict)]
    raise ValueError("Formato de devices.json no reconocido")


def _device_version(device: Dict[str, Any]) -> float:
    return float(device.get("version", device.get("ver", 3.4)))


def _device_local_key(device: Dict[str, Any]) -> str:
    return str(device.get("local_key") or device.get("key") or "")


def _parse_hhmm(value: Any) -> dt_time:
    hour_text, minute_text = str(value).strip().split(":", 1)
    return dt_time(hour=int(hour_text), minute=int(minute_text))


class SmartHomeNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "smart_home_node",
            start_parameter_services=False,
            parameter_overrides=[
                Parameter("start_type_description_service", Parameter.Type.BOOL, False)
            ],
        )

        self._callback_group = ReentrantCallbackGroup()
        self._config_path = _expand_path(
            str(self.declare_parameter("config_path", _default_config_path()).value)
        )
        config = _load_yaml(self._config_path)

        self._poll_period_s = float(config.get("poll_period_s", 30.0))
        self._tuya_timeout_s = float(config.get("tuya_timeout_s", 3.0))
        self._tz = ZoneInfo(str(config.get("timezone", "America/Argentina/Buenos_Aires")))
        devices_json_path = _expand_path(
            str(config.get("devices_json_path", "~/.config/robertito/smarthome/devices.json"))
        )
        lights = config.get("lights", {})
        if not isinstance(lights, dict) or not lights:
            raise ValueError("smart_home.yaml debe definir lights como dict no vacio")

        devices_by_ip = self._load_devices_by_ip(devices_json_path)
        self._lights: Dict[str, Dict[str, Any]] = {}
        self._devices_by_ip: Dict[str, Dict[str, Any]] = {}
        self._locks_by_ip: Dict[str, threading.Lock] = {}
        self._state_lock = threading.Lock()
        self._state: Dict[str, Optional[bool]] = {}
        self._groups = self._load_groups(config.get("groups", {}))
        self._always_on = self._load_always_on(config.get("always_on", {}))

        for name, item in lights.items():
            if not isinstance(item, dict):
                raise ValueError(f"Light {name!r} debe ser un dict")
            ip = str(item.get("ip", "")).strip()
            dps = int(item.get("dps"))
            if ip not in devices_by_ip:
                raise ValueError(f"Light {name!r} usa IP {ip}, pero no existe en devices.json")
            device = devices_by_ip[ip]
            if ip not in self._devices_by_ip:
                self._devices_by_ip[ip] = device
                self._locks_by_ip[ip] = threading.Lock()
            self._lights[str(name)] = {"ip": ip, "dps": dps}
            self._state[str(name)] = None

        for group_name, members in self._groups.items():
            for member in members:
                if member not in self._lights:
                    raise ValueError(f"Grupo {group_name!r} referencia luz desconocida {member!r}")
        for name in self._always_on:
            if name not in self._lights:
                raise ValueError(f"always_on referencia luz desconocida {name!r}")

        self._state_pub = self.create_publisher(String, "/smart_home/state", 10)
        for name in self._lights:
            self.create_service(
                SetBool,
                f"/smart_home/{name}/set",
                self._make_set_callback(name),
                callback_group=self._callback_group,
            )
        for group_name in self._groups:
            self.create_service(
                Trigger,
                f"/smart_home/{group_name}/off",
                self._make_group_off_callback(group_name),
                callback_group=self._callback_group,
            )
        self.create_service(
            Trigger,
            "/smart_home/refresh",
            self._on_refresh,
            callback_group=self._callback_group,
        )
        self._timer = self.create_timer(
            self._poll_period_s,
            self._poll,
            callback_group=self._callback_group,
        )

        light_summary = ", ".join(
            f"{name}=ip:{cfg['ip']} dps:{cfg['dps']}" for name, cfg in sorted(self._lights.items())
        )
        device_summary = ", ".join(
            f"{ip}({device.get('name', 'sin nombre')}, v{_device_version(device)})"
            for ip, device in sorted(self._devices_by_ip.items())
        )
        self.get_logger().info(
            f"SmartHome listo. Luces: {light_summary}. Switches detectados: {device_summary}"
        )
        if self._groups:
            self.get_logger().info(f"Grupos smart_home: {self._groups}")
        if self._always_on:
            self.get_logger().info(f"Reglas always_on smart_home: {self._always_on}")
        self._publish_state()
        threading.Thread(target=self._poll, daemon=True).start()

    def _load_groups(self, raw_groups: Any) -> Dict[str, list[str]]:
        if not isinstance(raw_groups, dict):
            return {}
        groups: Dict[str, list[str]] = {}
        for group_name, members in raw_groups.items():
            if not isinstance(members, list):
                raise ValueError(f"Grupo {group_name!r} debe ser una lista")
            groups[str(group_name)] = [str(member).strip() for member in members if str(member).strip()]
        return groups

    def _load_always_on(self, raw_rules: Any) -> Dict[str, Dict[str, dt_time]]:
        if not isinstance(raw_rules, dict):
            return {}
        rules: Dict[str, Dict[str, dt_time]] = {}
        for name, item in raw_rules.items():
            if not isinstance(item, dict):
                raise ValueError(f"always_on {name!r} debe ser un dict")
            rules[str(name)] = {
                "start": _parse_hhmm(item.get("start", "18:00")),
                "end": _parse_hhmm(item.get("end", "07:00")),
            }
        return rules

    def _load_devices_by_ip(self, devices_json_path: str) -> Dict[str, Dict[str, Any]]:
        devices = _extract_devices(_load_json(devices_json_path))
        by_ip: Dict[str, Dict[str, Any]] = {}
        for device in devices:
            ip = str(device.get("ip", "")).strip()
            dev_id = str(device.get("id", "")).strip()
            local_key = _device_local_key(device)
            if not ip:
                continue
            if not dev_id or not local_key:
                raise ValueError(f"Device Tuya {ip} no tiene id/local_key en devices.json")
            by_ip[ip] = device
        return by_ip

    def _make_device(self, ip: str) -> tinytuya.OutletDevice:
        info = self._devices_by_ip[ip]
        device = tinytuya.OutletDevice(
            str(info["id"]),
            ip,
            _device_local_key(info),
            connection_timeout=self._tuya_timeout_s,
            version=_device_version(info),
        )
        device.set_version(_device_version(info))
        return device

    def _read_status_for_ip(self, ip: str) -> Optional[Dict[str, Any]]:
        with self._locks_by_ip[ip]:
            try:
                response = self._make_device(ip).status()
            except Exception as exc:
                self.get_logger().warn(f"No se pudo leer switch {ip}: {exc}")
                return None
        if not isinstance(response, dict):
            self.get_logger().warn(f"Respuesta status invalida de {ip}: {response!r}")
            return None
        dps = response.get("dps")
        if not isinstance(dps, dict):
            self.get_logger().warn(f"Respuesta status sin dps de {ip}: {response!r}")
            return None
        return dps

    def _poll(self) -> None:
        next_state: Dict[str, Optional[bool]] = {}
        status_by_ip: Dict[str, Optional[Dict[str, Any]]] = {}
        for ip in self._devices_by_ip:
            status_by_ip[ip] = self._read_status_for_ip(ip)
        for name, cfg in self._lights.items():
            dps_status = status_by_ip.get(cfg["ip"])
            if dps_status is None:
                next_state[name] = None
                continue
            value = dps_status.get(str(cfg["dps"]), dps_status.get(cfg["dps"]))
            next_state[name] = bool(value) if isinstance(value, bool) else None
        with self._state_lock:
            self._state.update(next_state)
        self._publish_state()
        self._enforce_always_on()

    def _is_always_on_active(self, name: str) -> bool:
        rule = self._always_on.get(name)
        if not rule:
            return False
        now = datetime.now(self._tz).time()
        start = rule["start"]
        end = rule["end"]
        if start <= end:
            return start <= now < end
        return now >= start or now < end

    def _enforce_always_on(self) -> None:
        for name in self._always_on:
            if not self._is_always_on_active(name):
                continue
            with self._state_lock:
                current = self._state.get(name)
            if current is True:
                continue
            ok, message = self._set_light(name, True, force=True)
            if ok:
                self.get_logger().info(f"always_on: {name} encendida por horario")
            else:
                self.get_logger().warn(f"always_on: no se pudo encender {name}: {message}")

    def _set_light(self, name: str, on: bool, force: bool = False) -> tuple[bool, str]:
        if not on and not force and self._is_always_on_active(name):
            return False, f"{name} debe quedar encendida entre 18:00 y 07:00"
        cfg = self._lights[name]
        ip = cfg["ip"]
        dps = int(cfg["dps"])
        with self._locks_by_ip[ip]:
            try:
                response = self._make_device(ip).set_status(bool(on), switch=dps)
            except Exception as exc:
                with self._state_lock:
                    self._state[name] = None
                self._publish_state()
                return False, f"No se pudo setear {name} ({ip} dps {dps}): {exc}"
        success = not (isinstance(response, dict) and response.get("Error"))
        if success:
            with self._state_lock:
                self._state[name] = bool(on)
            self._publish_state()
            return True, f"{name}={'on' if on else 'off'}"
        return False, f"Tinytuya rechazo {name}: {response!r}"

    def _set_group_off(self, group_name: str) -> tuple[bool, str]:
        members = self._groups[group_name]
        failures = []
        for name in members:
            ok, message = self._set_light(name, False)
            if not ok:
                failures.append(f"{name}: {message}")
            time.sleep(0.35)
        time.sleep(0.8)
        observed = self._read_light_states(members)
        retry_members = [name for name, value in observed.items() if value is not False]
        for name in retry_members:
            ok, message = self._set_light(name, False)
            if not ok:
                failures.append(f"{name}: {message}")
            time.sleep(0.35)
        if retry_members:
            time.sleep(0.8)
            observed = self._read_light_states(members)
        still_on = [name for name, value in observed.items() if value is not False]
        if still_on:
            failures.append("siguen encendidas/desconocidas: " + ", ".join(still_on))
        if failures:
            return False, "; ".join(failures)
        return True, f"{group_name}=off ({', '.join(members)})"

    def _read_light_states(self, names: list[str]) -> Dict[str, Optional[bool]]:
        status_by_ip: Dict[str, Optional[Dict[str, Any]]] = {}
        for name in names:
            ip = self._lights[name]["ip"]
            if ip not in status_by_ip:
                status_by_ip[ip] = self._read_status_for_ip(ip)
        observed: Dict[str, Optional[bool]] = {}
        for name in names:
            cfg = self._lights[name]
            dps_status = status_by_ip.get(cfg["ip"])
            if dps_status is None:
                observed[name] = None
                continue
            value = dps_status.get(str(cfg["dps"]), dps_status.get(cfg["dps"]))
            observed[name] = bool(value) if isinstance(value, bool) else None
        with self._state_lock:
            self._state.update(observed)
        self._publish_state()
        return observed

    def _make_set_callback(self, name: str):
        def callback(request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
            response.success, response.message = self._set_light(name, bool(request.data))
            return response

        return callback

    def _make_group_off_callback(self, group_name: str):
        def callback(request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            del request
            response.success, response.message = self._set_group_off(group_name)
            return response

        return callback

    def _on_refresh(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._poll()
        response.success = True
        response.message = "refresh completo"
        return response

    def _publish_state(self) -> None:
        with self._state_lock:
            payload = dict(sorted(self._state.items()))
        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SmartHomeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
