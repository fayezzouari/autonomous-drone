"""PID navigation controller for the singlecopter Blender simulation.

The package is split into small, single-responsibility modules:

  config      — typed config + YAML loader (drone params mirror the sim)
  telemetry   — Telemetry / Command dataclasses + JSON (de)serialisation
  pid         — a single reusable PID controller
  controller  — the cascaded position/velocity/altitude flight controller
  mission     — waypoint sequencing and arrival logic
  mqtt_io     — paho-mqtt transport wiring telemetry in / commands out
  main        — entry point (real MQTT, or an in-process sim for offline demos)
"""

__version__ = "0.1.0"
