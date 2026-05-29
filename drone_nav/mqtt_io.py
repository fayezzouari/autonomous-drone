"""MQTT transport: telemetry in, commands out.

Thin wrapper over paho-mqtt. The network runs on paho's background thread; the
latest telemetry is cached behind a lock so the control loop (any thread) can
read a consistent snapshot without blocking.
"""

from __future__ import annotations

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
        self._connected.set()

    def _on_message(self, client, userdata, msg):
        if msg.topic == self.cfg.topic_telemetry:
            try:
                tlm = Telemetry.from_json(msg.payload)
            except (ValueError, TypeError):
                return
            with self._lock:
                self._latest = tlm

    # ── data access ──────────────────────────────────────────────────────────────
    def latest_telemetry(self) -> Optional[Telemetry]:
        with self._lock:
            return self._latest

    def publish_command(self, cmd: Command) -> None:
        self._client.publish(self.cfg.topic_command, cmd.to_json())

    def publish_status(self, text: str) -> None:
        self._client.publish(self.cfg.topic_status, text)
