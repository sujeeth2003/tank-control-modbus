"""Nonlinear tank process + a tiny 'PLC' safety layer, exposed as Modbus registers.

Physics (Torricelli outflow):   A * dh/dt = q_in - k * (1 + demand) * sqrt(h)
    q_in = q_max * valve,  valve follows the commanded position through a first-order lag + rate limit (an actuator).
Sensor: level with Gaussian noise, quantised to millimetres (like a 16-bit transmitter).

Register map (Modbus addresses)
  input   0  level_mm (noisy measurement)        holding 0  valve_cmd  (0..1000 permille)
  input   1  valve_position (permille, actual)    holding 1  setpoint_mm (informational, written by the controller)
  input   2  alarm bits: 1=high level, 2=interlock active   holding 2  outlet_demand (0..1000 permille: disturbance)
  input   3  heartbeat counter (increments each plant step; lets a client detect a frozen simulation)
  input   4  true level_mm (noise-free, for evaluation only)

Safety interlock (independent of the controller, like a hard-wired trip): above HIGH_TRIP the valve is forced closed
until the level drops below HIGH_RESET (hysteresis).
"""
import math
import random
import threading
import time

