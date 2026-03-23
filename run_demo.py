"""Closed-loop demo: a Python PID controller regulates the tank through Modbus/TCP in real time.

    python run_demo.py [--seconds 30] [--rate 200] [--plot out.png]

Scenario: hold 1000 mm, step the setpoint to 1400 mm, then open the outlet (a +25% demand disturbance), then ask for
1900 mm to show that the independent high-level interlock overrides the controller. Prints performance and latency.
"""
import argparse

from tank.control import PID, run_loop, summarize
from tank.modbus import ModbusClient
from tank.plant import PlantServer


