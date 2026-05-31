"""Bench tool: stream a fixed servo/ESC command to the ESP32 so the vanes hold.

The firmware fails safe (vanes -> 90deg, ESC idle) if no command arrives for
400 ms, so a one-shot publish only twitches the servos. This streams the same
command at a steady rate (default 20 Hz) until you Ctrl-C, so a vane stays
deflected and you can see exactly which one moves.

    # hold vane 1 at 40deg, the rest centred (others default to 90):
    uv run python tools/servo_test.py --s1 40

    # sweep test: move each vane in turn (2 s each), then exit:
    uv run python tools/servo_test.py --sweep

    # custom broker / all four:
    uv run python tools/servo_test.py --host 192.168.1.184 --s1 40 --s4 140

Throttle stays 0 unless you pass --throttle, so the motor never spins.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import paho.mqtt.client as mqtt

from drone_nav.config import load_config

TOPIC_HW = "drone/hw"
DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "config.yaml"


def _client(host: str, port: int) -> mqtt.Client:
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="servo-test")
    c.connect(host, port, 30)
    c.loop_start()
    return c


def _publish(c: mqtt.Client, throttle, s1, s2, s3, s4) -> None:
    c.publish(TOPIC_HW, json.dumps(
        {"throttle": throttle, "s1": s1, "s2": s2, "s3": s3, "s4": s4}))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Stream a servo/ESC command to the ESP32")
    ap.add_argument("--host", default=None, help="broker host (default: from config.yaml)")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--rate", type=float, default=20.0, help="publish Hz (>2.5 to beat failsafe)")
    ap.add_argument("--throttle", type=float, default=0.0)
    for i in range(1, 5):
        ap.add_argument(f"--s{i}", type=float, default=90.0, help=f"vane {i} angle deg")
    ap.add_argument("--sweep", action="store_true",
                    help="move each vane to 40 then 140 in turn (2 s each), then exit")
    ap.add_argument("--hop", action="store_true",
                    help="OPEN-LOOP vertical hop: arm, ramp throttle up, hold, ramp down")
    ap.add_argument("--hop-throttle", type=float, default=0.6,
                    help="peak throttle during the hop (tune this for ~your height)")
    ap.add_argument("--hop-time", type=float, default=2.0, help="seconds to hold peak")
    ap.add_argument("--arm-time", type=float, default=2.0, help="idle-arm seconds first")
    args = ap.parse_args(argv)

    cfg = load_config(str(DEFAULT_CONFIG))
    host = args.host or cfg.mqtt.host
    port = args.port or cfg.mqtt.port

    c = _client(host, port)
    period = 1.0 / args.rate
    print(f"streaming to {host}:{port} '{TOPIC_HW}' at {args.rate} Hz "
          f"(Ctrl-C to stop & let it re-centre)")

    def hold(throttle, seconds):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            _publish(c, throttle, 90.0, 90.0, 90.0, 90.0)
            time.sleep(period)

    def ramp(t0, t1, seconds):
        start = time.monotonic()
        while True:
            f = (time.monotonic() - start) / seconds
            if f >= 1.0:
                break
            _publish(c, t0 + (t1 - t0) * f, 90.0, 90.0, 90.0, 90.0)
            time.sleep(period)

    try:
        if args.hop:
            print(f"  ARM (idle {args.arm_time:.0f}s) — keep clear / tethered...")
            hold(0.0, args.arm_time)
            print(f"  CLIMB -> throttle {args.hop_throttle:.2f}")
            ramp(0.0, args.hop_throttle, 0.8)
            print(f"  HOLD {args.hop_time:.0f}s")
            hold(args.hop_throttle, args.hop_time)
            print("  DESCEND -> idle")
            ramp(args.hop_throttle, 0.0, 1.2)
            hold(0.0, 0.5)
            print("  hop done.")
        elif args.sweep:
            for i in range(4):
                for ang in (40.0, 140.0, 90.0):
                    s = [90.0, 90.0, 90.0, 90.0]
                    s[i] = ang
                    print(f"  vane {i+1} -> {ang:.0f}deg")
                    t_end = time.monotonic() + 2.0
                    while time.monotonic() < t_end:
                        _publish(c, 0.0, *s)
                        time.sleep(period)
        else:
            print(f"  holding throttle={args.throttle} "
                  f"s=({args.s1:.0f},{args.s2:.0f},{args.s3:.0f},{args.s4:.0f})")
            while True:
                _publish(c, args.throttle, args.s1, args.s2, args.s3, args.s4)
                time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopping — sending neutral.")
        _publish(c, 0.0, 90.0, 90.0, 90.0, 90.0)
        time.sleep(0.1)
    finally:
        c.loop_stop()
        c.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
