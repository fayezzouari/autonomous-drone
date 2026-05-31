"""Lock the MQTT wire format shared by the controller and the Blender bridge."""

import math

from drone_nav.telemetry import Command, Telemetry
from tools.sim_stub import SimStub


def test_telemetry_roundtrip():
    t = Telemetry(t=1.0, x=1.5, y=-2.0, z=3.0, vx=0.1, vy=0.2, vz=-0.3,
                  yaw=0.5, prop_speed=480.0)
    back = Telemetry.from_json(t.to_json())
    assert back == t


def test_command_roundtrip():
    c = Command(throttle=0.68, vane1=0.12, vane2=-0.07, vane3=0.05, vane4=0.0)
    back = Command.from_json(c.to_json())
    assert back == c
    assert c.vanes == (0.12, -0.07, 0.05, 0.0)


def test_telemetry_tolerates_unknown_keys():
    # The bridge may add fields later; nav must ignore extras gracefully.
    t = Telemetry.from_json('{"x": 1.0, "z": 2.0, "future_field": 99}')
    assert t.x == 1.0 and t.z == 2.0


def test_bridge_telemetry_keys_match_dataclass():
    # These are exactly the keys the Blender bridge publishes (_publish_telemetry).
    bridge_keys = {"t", "x", "y", "z", "vx", "vy", "vz", "yaw", "gz", "prop_speed"}
    assert bridge_keys == set(Telemetry().__dataclass_fields__)


def test_stub_emits_valid_telemetry():
    sim = SimStub()
    tlm = sim.step(Command(throttle=0.68), dt=0.02)
    # Round-trips through JSON just like over the wire.
    assert Telemetry.from_json(tlm.to_json()) == tlm
    assert not math.isnan(tlm.z)
