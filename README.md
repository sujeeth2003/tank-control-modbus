# Tank Level Control over Modbus/TCP (real-time, with latency)

A nonlinear tank process is simulated in Python and exposed through a real **Modbus/TCP** server (the industrial protocol PLCs and SCADA systems speak). Controllers in **Python and C++** read the level sensor over the network, decide, and write the valve command, in real time, while measuring the **sensor-to-actuator latency of every decision**.

> A **MATLAB/Simulink model is not included**: MATLAB is not available here, and I would not ship a Simulink model I cannot run. The plant is a small ODE (below), so it ports directly if you want one.

```
 plant (Python, real time)  <--Modbus/TCP-->  controller (Python PID or C++ PID)
   Torricelli tank + actuator lag                 read level (FC04) -> decide -> write valve (FC06)
   noisy level sensor, PLC safety interlock       latency measured per cycle: p50 / p99 / p99.9 / max
```

