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
from .obstacles import DEFAULT_AXES, DEFAULT_FLIP, ObstacleField
from .telemetry import Command, Telemetry


class MqttLink:
    def __init__(self, cfg: MQTTConfig, obstacle_axes: str = DEFAULT_AXES,
                 obstacle_flip: str = DEFAULT_FLIP):
        self.cfg = cfg
        self.obstacle_axes = obstacle_axes   # w/t/h → X,Y,Z mapping for drone/obs
        self.obstacle_flip = obstacle_flip   # centre axes to negate (source→world)
        self._lock = threading.Lock()
        self._latest: Optional[Telemetry] = None
        self._vanes = (0.0, 0.0, 0.0, 0.0)   # latest raw vane angles (rad)
        self._obstacles = ObstacleField([])  # latest obstacle set
        self._obs_version = 0                # bumps on every new obstacle message
        self._goto: Optional[tuple] = None   # latest live A→B target (x,y,z)
        self._goto_version = 0               # bumps on every new goto message
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
        client.subscribe(self.cfg.topic_obstacles)
        client.subscribe(self.cfg.topic_goto)
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
        elif msg.topic == self.cfg.topic_obstacles:
            try:
                field = ObstacleField.from_payload(
                    msg.payload, self.obstacle_axes, self.obstacle_flip)
            except (ValueError, TypeError):
                return
            with self._lock:
                self._obstacles = field
                self._obs_version += 1
        elif msg.topic == self.cfg.topic_goto:
            target = _parse_goto(msg.payload)
            if target is not None:
                with self._lock:
                    self._goto = target
                    self._goto_version += 1

    # ── data access ──────────────────────────────────────────────────────────────
    def latest_telemetry(self) -> Optional[Telemetry]:
        with self._lock:
            return self._latest

    def latest_vanes(self) -> tuple:
        """Most recent raw vane angles (vane1..vane4); zeros until one arrives."""
        with self._lock:
            return self._vanes

    def latest_obstacles(self) -> tuple:
        """Return ``(ObstacleField, version)``; version bumps on each new message."""
        with self._lock:
            return self._obstacles, self._obs_version

    def latest_goto(self) -> tuple:
        """Return ``((x,y,z) | None, version)`` for the live drone/goto target."""
        with self._lock:
            return self._goto, self._goto_version

    def publish_command(self, cmd: Command) -> None:
        self._client.publish(self.cfg.topic_command, cmd.to_json())

    def publish_hw_command(self, hw: dict) -> None:
        """Publish the ESP32 hardware command (ESC fraction + servo degrees)."""
        self._client.publish(self.cfg.topic_hw_cmd, json.dumps(hw))

    def publish_status(self, text: str) -> None:
        self._client.publish(self.cfg.topic_status, text)

    def publish_path(self, waypoints) -> None:
        """Publish the planned route (list of [x,y,z]) for world/UI consumers."""
        pts = [[float(p[0]), float(p[1]), float(p[2])] for p in waypoints]
        self._client.publish(self.cfg.topic_path, json.dumps({"waypoints": pts}))


def _parse_goto(payload) -> Optional[tuple]:
    """Parse a ``drone/goto`` target into ``(x, y, z)``.

    Accepts ``[x, y, z]``, ``{"x":,"y":,"z":}`` (also tx/ty/tz aliases, or a
    nested ``{"goto":[...]}`` / ``{"target":[...]}``).
    """
    if isinstance(payload, (bytes, bytearray)):
        payload = payload.decode("utf-8")
    try:
        data = json.loads(payload)
    except (ValueError, TypeError):
        return None
    if isinstance(data, dict):
        for key in ("goto", "target", "point"):
            if key in data:
                data = data[key]
                break
    if isinstance(data, dict):
        def pick(*keys):
            for k in keys:
                if k in data:
                    return float(data[k])
            return None
        x, y, z = pick("x", "tx"), pick("y", "ty"), pick("z", "tz")
        if x is None or y is None or z is None:
            return None
        return (x, y, z)
    if isinstance(data, (list, tuple)) and len(data) >= 3:
        try:
            return (float(data[0]), float(data[1]), float(data[2]))
        except (ValueError, TypeError):
            return None
    return None


def _telemetry_from_imu(payload, prev: Optional[Telemetry]) -> Optional[Telemetry]:
    """Fold an ESP32 IMU message into a :class:`Telemetry` snapshot.

    The MPU6050 only gives orientation, so this updates ``yaw`` and the ``gz``
    yaw rate (the IMU reports both in degrees / deg·s⁻¹; we convert to radians)
    and carries position/velocity forward from the previous snapshot — the IMU
    cannot observe them. ``prop_speed`` is left as-is so any live-thrust estimate
    keeps working.
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
        gz=math.radians(float(data.get("gz", math.degrees(base.gz)))),
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
