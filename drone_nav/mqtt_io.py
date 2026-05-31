"""MQTT transport: telemetry in, commands out.

Thin wrapper over paho-mqtt. The network runs on paho's background thread; the
latest telemetry is cached behind a lock so the control loop (any thread) can
read a consistent snapshot without blocking.
"""

from __future__ import annotations

import json
import threading
from typing import Optional

import paho.mqtt.client as mqtt

from .config import MQTTConfig
from .telemetry import Command, Telemetry


class MqttLink:
    def __init__(self, cfg: MQTTConfig):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._latest: Optional[Telemetry] = None
        self._vanes = (0.0, 0.0, 0.0, 0.0)   # latest raw vane angles (rad)
        self._connected = threading.Event()
        self.last_error: Optional[str] = None

        # paho-mqtt 2.x requires the callback API version.
        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=cfg.client_id
        )
        if cfg.username:
            self._client.username_pw_set(cfg.username, cfg.password)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

    # ── lifecycle ──────────────────────────────────────────────────────────────
    def connect(self, timeout: float = 5.0) -> bool:
        try:
            self._client.connect(self.cfg.host, self.cfg.port, self.cfg.keepalive)
        except OSError as exc:
            self.last_error = str(exc)
            return False
        self._client.loop_start()
        return self._connected.wait(timeout)

    def close(self) -> None:
        try:
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            pass

    # ── callbacks ────────────────────────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        client.subscribe(self.cfg.topic_telemetry)
        client.subscribe(self.cfg.topic_vane_input)
        client.subscribe(self.cfg.topic_imu)
        self._connected.set()

    def _on_message(self, client, userdata, msg):
        if msg.topic == self.cfg.topic_telemetry:
            try:
                tlm = Telemetry.from_json(msg.payload)
            except (ValueError, TypeError):
                return
            with self._lock:
                self._latest = tlm
        elif msg.topic == self.cfg.topic_imu:
            tlm = _telemetry_from_imu(msg.payload, self.latest_telemetry())
            if tlm is not None:
                with self._lock:
                    self._latest = tlm
        elif msg.topic == self.cfg.topic_vane_input:
            vanes = _parse_vanes(msg.payload)
            if vanes is not None:
                with self._lock:
                    self._vanes = vanes

    # ── data access ──────────────────────────────────────────────────────────────
    def latest_telemetry(self) -> Optional[Telemetry]:
        with self._lock:
            return self._latest

    def latest_vanes(self) -> tuple:
        """Most recent raw vane angles (vane1..vane4); zeros until one arrives."""
        with self._lock:
            return self._vanes

    def publish_command(self, cmd: Command) -> None:
        self._client.publish(self.cfg.topic_command, cmd.to_json())

    def publish_hw_command(self, hw: dict) -> None:
        """Publish the ESP32 hardware command (ESC fraction + servo degrees)."""
        self._client.publish(self.cfg.topic_hw_cmd, json.dumps(hw))

    def publish_status(self, text: str) -> None:
        self._client.publish(self.cfg.topic_status, text)


def _telemetry_from_imu(payload, prev: Optional[Telemetry]) -> Optional[Telemetry]:
    """Fold an ESP32 IMU message into a :class:`Telemetry` snapshot.

    The MPU6050 only gives orientation, so this updates ``yaw`` (the IMU reports
    degrees; we convert to radians) and carries position/velocity forward from
    the previous snapshot — the IMU cannot observe them. ``prop_speed`` is left
    as-is so any live-thrust estimate keeps working.
    """
    import math

    if isinstance(payload, (bytes, bytearray)):
        payload = payload.decode("utf-8")
    try:
        data = json.loads(payload)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict) or "yaw" not in data:
        return None
    base = prev or Telemetry()
    return Telemetry(
        t=float(data.get("t", base.t)),
        x=base.x, y=base.y, z=base.z,
        vx=base.vx, vy=base.vy, vz=base.vz,
        yaw=math.radians(float(data["yaw"])),
        prop_speed=base.prop_speed,
    )


def _parse_vanes(payload) -> Optional[tuple]:
    """Parse a raw vane-input message into (v1, v2, v3, v4) radians.

    Accepts either an object {"vane1":..,"vane2":..,..} (also v1/v2/.. aliases)
    or a 4-element list [v1, v2, v3, v4].
    """
    if isinstance(payload, (bytes, bytearray)):
        payload = payload.decode("utf-8")
    try:
        data = json.loads(payload)
    except (ValueError, TypeError):
        return None
    if isinstance(data, list) and len(data) >= 4:
        return tuple(float(data[i]) for i in range(4))
    if isinstance(data, dict):
        def pick(*keys):
            for k in keys:
                if k in data:
                    return float(data[k])
            return 0.0
        return (pick("vane1", "v1"), pick("vane2", "v2"),
                pick("vane3", "v3"), pick("vane4", "v4"))
    return None
