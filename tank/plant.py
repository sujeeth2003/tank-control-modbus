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

from .modbus import ModbusServer, Registers


class TankPlant:
    def __init__(self, regs: Registers, area=0.02, k=0.01, q_max=0.02, h0=0.3, tau_valve=0.4, rate_limit=1.0,
                 noise_mm=2.0, high_trip_mm=1800, high_reset_mm=1650, seed=0):
        self.regs, self.A, self.k, self.q_max = regs, area, k, q_max
        self.h, self.valve = h0, 0.0                     # metres, 0..1
        self.tau, self.rate_limit, self.noise_mm = tau_valve, rate_limit, noise_mm
        self.trip, self.reset, self.tripped = high_trip_mm, high_reset_mm, False
        self.rng, self.steps = random.Random(seed), 0

    def step(self, dt):
        with self.regs.lock:
            cmd = self.regs.holding[0] / 1000.0
            demand = self.regs.holding[2] / 1000.0
        level_mm = self.h * 1000
        if level_mm >= self.trip: self.tripped = True
        elif level_mm <= self.reset: self.tripped = False
        if self.tripped: cmd = 0.0
        # actuator: first-order lag with a slew-rate limit
        dv = (cmd - self.valve) / self.tau * dt
        dv = max(-self.rate_limit * dt, min(self.rate_limit * dt, dv))
        self.valve = min(1.0, max(0.0, self.valve + dv))
        q_in = self.q_max * self.valve
        q_out = self.k * (1.0 + demand) * math.sqrt(max(self.h, 0.0))
        self.h = min(2.0, max(0.0, self.h + (q_in - q_out) / self.A * dt))
        self.steps += 1
        meas = int(round(self.h * 1000 + self.rng.gauss(0, self.noise_mm)))
        with self.regs.lock:
            self.regs.input[0] = max(0, min(65535, meas))
            self.regs.input[1] = int(self.valve * 1000)
            self.regs.input[2] = (1 if level_mm >= self.trip - 100 else 0) | (2 if self.tripped else 0)
            self.regs.input[3] = self.steps & 0xFFFF
            self.regs.input[4] = int(self.h * 1000)


class PlantServer:
    """Runs the plant in real time (dt = wall-clock time since the last step) next to a Modbus/TCP server."""

    def __init__(self, host="127.0.0.1", port=0, step_s=0.002, **plant_kw):
        self.regs = Registers()
        self.plant = TankPlant(self.regs, **plant_kw)
        self.server = ModbusServer(self.regs, host, port)
        self.addr, self.step_s = self.server.addr, step_s
        self._stop = threading.Event()
        self.log = []                                    # (t, true level mm, valve permille) sampled at ~50 Hz

    def start(self, run=True):
        """run=False serves Modbus but leaves the plant frozen, so tests can step it deterministically."""
        self.server.start()
        if run:
            threading.Thread(target=self._run, daemon=True).start()
        return self

    def _run(self):
        t0 = last = time.perf_counter()
        next_log = 0.0
        while not self._stop.is_set():
            now = time.perf_counter()
            self.plant.step(now - last)
            last = now
            if now - t0 >= next_log:
                self.log.append((now - t0, self.plant.h * 1000, self.plant.valve * 1000))
                next_log += 0.02
            time.sleep(self.step_s)

    def stop(self):
        self._stop.set()
        self.server.stop()
