"""Run the tank plant + Modbus/TCP server on its own so an external controller (e.g. the C++ one) can connect.

    python serve_plant.py [--port 1502]         # then:  controller/ctl 127.0.0.1 1502 1000 1000 20
"""
import argparse
import time

from tank.plant import PlantServer

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=1502)
ap.add_argument("--host", default="127.0.0.1")
a = ap.parse_args()
srv = PlantServer(a.host, a.port, h0=0.3).start()
print(f"tank plant serving Modbus/TCP on {a.host}:{a.port} (input reg 0 = level mm, holding reg 0 = valve permille). Ctrl-C to stop.")
try:
    while True:
        time.sleep(1)
        print(f"level {srv.plant.h * 1000:7.1f} mm   valve {srv.plant.valve * 1000:6.1f}   alarm bits {srv.regs.input[2]}")
except KeyboardInterrupt:
    srv.stop()
