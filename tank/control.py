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


def run_loop(host, port, controller, setpoint_mm, rate_hz=100, seconds=20.0, setpoint_fn=None, on_tick=None):
    """Fixed-rate loop. Each cycle: read level -> decide -> write valve. Returns per-cycle records and latencies (ns).
    'decision latency' = time from starting the read to the write being acknowledged (a full sensor-to-actuator round trip)."""
    cli = ModbusClient(host, port)
    period = 1.0 / rate_hz
    lat, rec = [], []
    t0 = nxt = time.perf_counter()
    last = t0
    while True:
        now = time.perf_counter()
        if now - t0 >= seconds:
            break
        if now < nxt:
            time.sleep(max(0.0, nxt - now - 0.0005))       # coarse sleep, then spin for the last 0.5 ms
            while time.perf_counter() < nxt: pass
        c0 = time.perf_counter_ns()
        level = cli.read_input(0)[0]
        sp = setpoint_fn(time.perf_counter() - t0) if setpoint_fn else setpoint_mm
        u = controller.update(sp, level, time.perf_counter() - last)
        last = time.perf_counter()
        cli.write_single(0, int(round(u)))
        lat.append(time.perf_counter_ns() - c0)
        rec.append((last - t0, sp, level, u))
        if on_tick: on_tick(rec[-1])
        nxt += period
        if nxt < time.perf_counter() - period:              # overran badly: resync instead of bursting
            nxt = time.perf_counter() + period
    cli.close()
    return rec, lat


def summarize(rec, lat_ns, settle_band_mm=25):
    import statistics
    lat = sorted(lat_ns)
    pct = lambda q: lat[min(len(lat) - 1, int(q * len(lat)))] / 1000.0
    out = {"cycles": len(lat), "lat_p50_us": pct(0.5), "lat_p99_us": pct(0.99), "lat_max_us": lat[-1] / 1000.0}
    err = [abs(sp - lv) for _, sp, lv, _ in rec]
    out["iae_mm"] = sum(err) / len(err)
    return out
