"""`phone-imu` — use an Android phone as the IMU instead of the MPU6050.

This bridges an Android phone's fused orientation sensor onto the same MQTT
topic (and exact wire format) the ESP32's MPU6050 publishes, so the rest of the
stack (navigator, web bridge, webapp) can't tell the difference.

The ESP32 publishes ``drone/imu`` at ~50 Hz as::

    {"t": <s>, "yaw": <deg>, "pitch": <deg>, "roll": <deg>, "gz": <deg/s>}

with angles in DEGREES and the yaw rate ``gz`` in DEG/S, in the drone body
frame (X forward, Y left, Z up; aerospace ZYX). The MPU6050_light library zeroes
its angles at boot, so this script does the same: it captures the phone's pose at
startup as the reference and reports orientation *relative* to it. That makes the
phone behave like the MPU6050 regardless of how it's physically mounted.

── Phone side (no coding) ───────────────────────────────────────────────────────
Install **SensorServer** (free, open source: github.com/umer0586/SensorServer),
which runs a WebSocket server exposing the phone's sensors. In the app:
  1. Enable the server and note the IP + port it prints (e.g. 192.168.1.50:8080).
  2. Keep the phone on the same Wi-Fi/LAN as the broker and this machine.
Recommended mounting for a 1:1 match with the drone body frame: phone flat,
screen up, the TOP edge of the phone pointing the drone's forward direction.
Then "lay it down, start the script" zeroes everything to 0/0/0.

    phone-imu --phone 192.168.1.50:8080            # WebSocket host of SensorServer
    phone-imu --phone 192.168.1.50:8080 --rate 50  # publish rate (Hz)
    phone-imu --phone 192.168.1.50 --absolute      # don't zero at startup
    phone-imu --phone 192.168.1.50 --roll-sign -1  # flip an axis if mounted differently

Press Ctrl-C to stop. Re-running re-zeros to the current pose.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from pathlib import Path
from typing import Optional, Tuple

from .config import load_config

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "config.yaml"

# SensorServer sensor type strings.
ROTATION_VECTOR = "android.sensor.rotation_vector"
GYROSCOPE = "android.sensor.gyroscope"


# ── quaternion helpers ───────────────────────────────────────────────────────────
Quat = Tuple[float, float, float, float]  # (w, x, y, z)


def _quat_from_rotation_vector(values) -> Quat:
    """Build a (w, x, y, z) quaternion from an Android ROTATION_VECTOR reading.

    Android reports the rotation vector as [x, y, z] (the vector part) and, on
    most devices, a 4th element w. When w is absent we reconstruct it; the
    rotation vector is a unit quaternion so w = sqrt(1 - |v|²).
    """
    x, y, z = float(values[0]), float(values[1]), float(values[2])
    if len(values) >= 4 and values[3] is not None:
        w = float(values[3])
    else:
        t = 1.0 - (x * x + y * y + z * z)
        w = math.sqrt(t) if t > 0.0 else 0.0
    return (w, x, y, z)


def _quat_conjugate(q: Quat) -> Quat:
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quat_mul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _euler_zyx_deg(q: Quat) -> Tuple[float, float, float]:
    """Decompose a quaternion into aerospace yaw/pitch/roll (ZYX), in degrees.

    Matches the MPU6050's convention: roll about X (forward), pitch about Y
    (left), yaw about Z (up). Returns (yaw, pitch, roll).
    """
    w, x, y, z = q
    # roll (X)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # pitch (Y) — clamp to avoid NaN at the poles
    sp = 2.0 * (w * y - z * x)
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    # yaw (Z)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


# ── the bridge ───────────────────────────────────────────────────────────────────
class PhoneImuBridge:
    def __init__(self, args, mqtt_cfg):
        self.args = args
        self.mqtt_cfg = mqtt_cfg
        self.ref: Optional[Quat] = None          # startup reference pose
        self.latest_q: Optional[Quat] = None     # most recent device orientation
        self.gz_dps: float = 0.0                 # most recent yaw rate (deg/s)
        self._client = None

    def _connect_mqtt(self):
        import paho.mqtt.client as mqtt

        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="phone-imu")
        if self.mqtt_cfg.username:
            c.username_pw_set(self.mqtt_cfg.username, self.mqtt_cfg.password)
        c.connect(self.mqtt_cfg.host, self.mqtt_cfg.port, self.mqtt_cfg.keepalive)
        c.loop_start()
        self._client = c

    def _on_reading(self, sensor_type: str, values) -> None:
        if sensor_type == ROTATION_VECTOR:
            q = _quat_from_rotation_vector(values)
            if self.ref is None and not self.args.absolute:
                self.ref = q  # first reading becomes the zero reference
            self.latest_q = q
        elif sensor_type == GYROSCOPE:
            # values are rad/s about device X, Y, Z. The yaw axis is Z.
            self.gz_dps = math.degrees(float(values[self.args.gyro_axis]))

    def _current_orientation(self) -> Optional[Tuple[float, float, float]]:
        if self.latest_q is None:
            return None
        q = self.latest_q
        if self.ref is not None:
            # orientation relative to the startup pose: q_rel = ref⁻¹ · q
            q = _quat_mul(_quat_conjugate(self.ref), q)
        yaw, pitch, roll = _euler_zyx_deg(q)
        a = self.args
        return (yaw * a.yaw_sign + a.yaw_offset,
                pitch * a.pitch_sign,
                roll * a.roll_sign)

    def _publish_loop(self):
        """Sample the latest orientation at a fixed rate and publish like the ESP32."""
        period = 1.0 / self.args.rate
        next_t = time.monotonic()
        verbose_next = 0.0
        while True:
            ori = self._current_orientation()
            if ori is not None:
                yaw, pitch, roll = ori
                payload = json.dumps({
                    "t": round(time.time(), 3),
                    "yaw": round(yaw, 2),
                    "pitch": round(pitch, 2),
                    "roll": round(roll, 2),
                    "gz": round(self.gz_dps, 2),
                })
                self._client.publish(self.mqtt_cfg.topic_imu, payload)
                if self.args.verbose and time.monotonic() >= verbose_next:
                    print(payload)
                    verbose_next = time.monotonic() + 0.5
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()  # we fell behind; resync

    async def _read_phone(self):
        import websockets

        types = json.dumps([ROTATION_VECTOR, GYROSCOPE])
        host = self.args.phone
        if ":" not in host:
            host = f"{host}:8080"  # SensorServer's default port
        url = f"ws://{host}/sensors/connect?types={types}"
        print(f"connecting to SensorServer at {url}")
        async for ws in websockets.connect(url, ping_interval=20, max_queue=8):
            try:
                print("phone connected — streaming orientation + gyro")
                async for raw in ws:
                    try:
                        msg = json.loads(raw)
                    except (ValueError, TypeError):
                        continue
                    stype = msg.get("type")
                    values = msg.get("values")
                    if stype and isinstance(values, list):
                        self._on_reading(stype, values)
            except Exception as exc:  # connection dropped — websockets.connect retries
                print(f"phone link lost ({exc}); reconnecting…")
                await asyncio.sleep(1.0)

    async def run(self):
        self._connect_mqtt()
        print(f"publishing to {self.mqtt_cfg.host}:{self.mqtt_cfg.port} "
              f"topic '{self.mqtt_cfg.topic_imu}' at {self.args.rate} Hz")
        if not self.args.absolute:
            print("zeroing to the phone's current pose at first reading "
                  "(hold it in the mounted orientation)…")
        # The publisher is a plain blocking loop; run it off the event loop so
        # the async websocket reader keeps draining the socket.
        pub = asyncio.get_event_loop().run_in_executor(None, self._publish_loop)
        try:
            await self._read_phone()
        finally:
            pub.cancel()
            if self._client is not None:
                self._client.loop_stop()
                self._client.disconnect()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Use an Android phone (via SensorServer) as the drone IMU")
    ap.add_argument("-c", "--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--phone", required=True,
                    help="SensorServer WebSocket host, e.g. 192.168.1.50:8080")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="publish rate in Hz (default 50, matches the ESP32)")
    ap.add_argument("--absolute", action="store_true",
                    help="report absolute orientation instead of zeroing at startup")
    ap.add_argument("--gyro-axis", type=int, default=2, choices=(0, 1, 2),
                    help="phone gyro axis used for gz/yaw-rate (0=X,1=Y,2=Z; default Z)")
    ap.add_argument("--yaw-sign", type=float, default=1.0)
    ap.add_argument("--pitch-sign", type=float, default=1.0)
    ap.add_argument("--roll-sign", type=float, default=1.0)
    ap.add_argument("--yaw-offset", type=float, default=0.0,
                    help="constant degrees added to yaw (e.g. to align with a heading)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print published samples (~2 Hz)")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    bridge = PhoneImuBridge(args, cfg.mqtt)
    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        print("\nstopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
