"""Real-time controllers that read the level over Modbus/TCP and write the valve command, timing every decision."""
import time

from .modbus import ModbusClient


class PID:
    """PID with derivative-on-measurement (no kick on setpoint steps), a filtered derivative and anti-windup by clamping."""

    def __init__(self, kp, ki, kd, out_min=0.0, out_max=1000.0, d_filter=0.1):
        self.kp, self.ki, self.kd, self.lo, self.hi, self.alpha = kp, ki, kd, out_min, out_max, d_filter
        self.integral, self.prev_meas, self.d = 0.0, None, 0.0

    def update(self, setpoint, meas, dt):
        err = setpoint - meas
        if self.prev_meas is not None and dt > 0:
            raw_d = -(meas - self.prev_meas) / dt
            self.d += self.alpha * (raw_d - self.d)
        self.prev_meas = meas
        u_unsat = self.kp * err + self.integral + self.kd * self.d
        u = min(self.hi, max(self.lo, u_unsat))
        if u == u_unsat or (u_unsat > self.hi and err < 0) or (u_unsat < self.lo and err > 0):   # only integrate when it helps
            self.integral += self.ki * err * dt
        return u


class Hysteresis:
    """On/off (bang-bang) controller with a dead band, as a simple PLC would do it."""

    def __init__(self, band=30.0, on=1000.0, off=0.0):
        self.band, self.on, self.off, self.state = band, on, off, False

    def update(self, setpoint, meas, dt):
        if meas < setpoint - self.band: self.state = True
        elif meas > setpoint + self.band: self.state = False
        return self.on if self.state else self.off


