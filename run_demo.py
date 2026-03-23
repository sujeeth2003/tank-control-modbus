"""Closed-loop demo: a Python PID controller regulates the tank through Modbus/TCP in real time.

    python run_demo.py [--seconds 30] [--rate 200] [--plot out.png]

Scenario: hold 1000 mm, step the setpoint to 1400 mm, then open the outlet (a +25% demand disturbance), then ask for
1900 mm to show that the independent high-level interlock overrides the controller. Prints performance and latency.
"""
import argparse

from tank.control import PID, run_loop, summarize
from tank.modbus import ModbusClient
from tank.plant import PlantServer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=32.0)
    ap.add_argument("--rate", type=float, default=200.0)
    ap.add_argument("--plot")
    a = ap.parse_args()

    srv = PlantServer(h0=0.3).start()
    host, port = srv.addr
    dist = ModbusClient(host, port)
    events = {"disturbed": False}

    def setpoint(t):
        return 1000 if t < 10 else 1400 if t < 24 else 1900

    def tick(r):
        if r[0] > 16 and not events["disturbed"]:
            dist.write_single(2, 250); events["disturbed"] = True      # operator opens the outlet valve (+25% demand)

    rec, lat = run_loop(host, port, PID(0.6, 0.25, 0.0), 1000, a.rate, a.seconds, setpoint, tick)
    stats = summarize(rec, lat)
    alarms = srv.regs.input[2]
    srv.stop()

    def seg(t0, t1):
        pts = [r for r in rec if t0 <= r[0] < t1]
        return sum(abs(r[1] - r[2]) for r in pts) / len(pts), max(r[2] for r in pts)
    print(f"{stats['cycles']} control cycles at {a.rate:.0f} Hz")
    print(f"decision latency (read level -> compute -> write valve, acknowledged): p50 {stats['lat_p50_us']:.0f} us, "
          f"p99 {stats['lat_p99_us']:.0f} us, max {stats['lat_max_us']:.0f} us")
    for name, (t0, t1) in {"hold 1000 mm (settled part)": (6, 10), "after step to 1400 mm (settled part)": (18, 24)}.items():
        e, peak = seg(t0, t1)
        print(f"  {name}: mean |error| {e:.1f} mm")
    if a.seconds > 26:
        _, peak = seg(24, a.seconds)
        print(f"  request 1900 mm: peak level {peak} mm (interlock trips at 1800 mm; alarm bits now {alarms})")
    if a.plot:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = [r[0] for r in rec]
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(9, 5))
        ax[0].plot(t, [r[2] for r in rec], label="measured level"); ax[0].plot(t, [r[1] for r in rec], "--", label="setpoint"); ax[0].legend(); ax[0].set_ylabel("mm")
        ax[1].plot(t, [r[3] for r in rec]); ax[1].set_ylabel("valve cmd (permille)"); ax[1].set_xlabel("s")
        fig.tight_layout(); fig.savefig(a.plot, dpi=120)


if __name__ == "__main__":
    main()
