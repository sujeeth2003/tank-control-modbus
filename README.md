# Tank Level Control over Modbus/TCP (real-time, with latency)

A nonlinear tank process is simulated in Python and exposed through a real **Modbus/TCP** server (the industrial protocol PLCs and SCADA systems speak). Controllers in **Python and C++** read the level sensor over the network, decide, and write the valve command, in real time, while measuring the **sensor-to-actuator latency of every decision**.

> A **MATLAB/Simulink model is not included**: MATLAB is not available here, and I would not ship a Simulink model I cannot run. The plant is a small ODE (below), so it ports directly if you want one.

```
 plant (Python, real time)  <--Modbus/TCP-->  controller (Python PID or C++ PID)
   Torricelli tank + actuator lag                 read level (FC04) -> decide -> write valve (FC06)
   noisy level sensor, PLC safety interlock       latency measured per cycle: p50 / p99 / p99.9 / max
```

## The process (`tank/plant.py`)
`A dh/dt = q_in - k(1 + demand) sqrt(h)`, with `q_in = q_max * valve`; the valve is an actuator (first-order lag + slew-rate limit); the sensor has Gaussian noise and 1 mm quantisation. An operator "demand" register acts as a disturbance (outlet opening). An independent **high-level interlock** (trip at 1800 mm, reset 1650 mm) forces the valve closed regardless of what the controller says, like a hard-wired safety trip.

Modbus map: input reg 0 = level (mm), 1 = valve position, 2 = alarm bits, 3 = heartbeat; holding reg 0 = valve command (permille), 2 = outlet demand.

## Protocol (`tank/modbus.py`)
Modbus/TCP written from the spec: MBAP framing, function codes 03/04/06/16, exception responses (illegal function / address / value), persistent connections, transaction-id checking. The C++ controller re-implements the client side independently, so the two interoperate over the wire.

## Controllers
- **PID** (`tank/control.py`, `controller/ctl.cpp`): derivative on measurement (no kick on setpoint steps), filtered derivative, anti-windup.
- **Hysteresis / bang-bang** (Python): the simple PLC-style alternative.

## Results (this machine: Windows laptop, loopback, single run each; not tuned or isolated)
Python PID at 200 Hz, 32 s scenario (`python run_demo.py`):
```
6401 control cycles at 200 Hz
decision latency (read -> compute -> write, acknowledged): p50 455 us, p99 1074 us, max 3065 us
hold 1000 mm: mean |error| 9.1 mm      request 1900 mm: peak 1814 mm (interlock caught it at 1800)
```
C++ PID against the same plant server (`build/ctl 127.0.0.1 1502 1000 <rate> <s>`):

| rate | cycles | overruns | latency p50 | p99 | p99.9 | max |
|---|---|---|---|---|---|---|
| 1 kHz, 12 s | 12,001 | 0 | 79 us | 434 us | 652 us | 1.3 ms |
| 5 kHz, 8 s | 39,816 | 56 | 60 us | 199 us | 450 us | 2.1 ms |

The latency is a full network round trip (two Modbus transactions over TCP loopback) into a Python server, so most of it is the server side; tail spikes are OS scheduling. On a real PLC over Ethernet expect the wire and the device to dominate instead. The "mean error" the controllers print includes the initial fill from 300 mm, so it looks large; the settled error is what the demo reports.

## Run
```bash
python -m unittest discover -s tests       # 8 tests, ~30 s: Modbus frames and exceptions, plant equilibrium, interlock, closed loop
python run_demo.py                         # closed-loop scenario with latency
