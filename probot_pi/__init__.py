"""probot_pi — Raspberry Pi fuzzy supervisor for the probot differential drive.

Reads telemetry from the ESP32 over UART, runs a 2-block Mamdani supervisor
(yaw-stability + traction) and sends modulated wheel-speed setpoints back.
"""
__version__ = "0.1.0"
