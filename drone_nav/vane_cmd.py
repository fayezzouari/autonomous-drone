"""`vane-cmd` — publish raw vane angles to the navigator's input topic.

The navigator subscribes to ``drone/vanes`` and merges these four angles with
its altitude-hold throttle. This CLI lets you set the vanes by hand:

    vane-cmd --v1 0.15 --v3 -0.15          # tilt the fore/aft pair
    vane-cmd --deg --v2 10 --v4 -10        # angles in degrees instead of radians
    vane-cmd --zero                        # everything back to neutral

Angles are in radians unless --deg is given.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .config import load_config

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "config.yaml"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Publish raw vane angles to the navigator")
    ap.add_argument("-c", "--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--deg", action="store_true", help="interpret angles as degrees")
    ap.add_argument("--zero", action="store_true", help="set all four vanes to 0")
    for i in range(1, 5):
        ap.add_argument(f"--v{i}", type=float, default=0.0)
    args = ap.parse_args(argv)

    import paho.mqtt.client as mqtt

    cfg = load_config(args.config)
    vals = [0.0, 0.0, 0.0, 0.0] if args.zero else [args.v1, args.v2, args.v3, args.v4]
    if args.deg:
        vals = [math.radians(v) for v in vals]

    payload = json.dumps({"vane1": vals[0], "vane2": vals[1],
                          "vane3": vals[2], "vane4": vals[3]})

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="vane-cmd")
    if cfg.mqtt.username:
        client.username_pw_set(cfg.mqtt.username, cfg.mqtt.password)
    client.connect(cfg.mqtt.host, cfg.mqtt.port, cfg.mqtt.keepalive)
    client.loop_start()
    info = client.publish(cfg.mqtt.topic_vane_input, payload, qos=1)
    info.wait_for_publish(timeout=5)
    client.loop_stop()
    client.disconnect()
    print(f"published to {cfg.mqtt.topic_vane_input}: {payload}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
