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

