import os
import socket
import struct
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from tank.control import Hysteresis, PID, run_loop, summarize  # noqa: E402
from tank.modbus import ModbusClient, ModbusException, ModbusServer, Registers, frame, handle_pdu, read_frame  # noqa: E402
from tank.plant import PlantServer  # noqa: E402


class ModbusTests(unittest.TestCase):
    def setUp(self):
        self.regs = Registers()
        self.srv = ModbusServer(self.regs).start()
        self.cli = ModbusClient(*self.srv.addr)

    def tearDown(self):
        self.cli.close(); self.srv.stop()

    def test_known_frame_bytes(self):
        # Read 2 holding registers at address 0x006B, unit 0x11: the classic example from the Modbus spec
        pdu = struct.pack(">BHH", 0x03, 0x006B, 2)
        self.assertEqual(frame(1, 0x11, pdu).hex(), "00010000000611030" + "06b0002")

    def test_write_read_roundtrip(self):
        self.cli.write_single(3, 1234)
        self.assertEqual(self.cli.read_holding(3), [1234])
        self.cli.write_multiple(4, [1, 2, 3])
        self.assertEqual(self.cli.read_holding(3, 4), [1234, 1, 2, 3])
        self.regs.input[5] = 777
        self.assertEqual(self.cli.read_input(5), [777])

    def test_exceptions(self):
        with self.assertRaises(ModbusException) as e: self.cli.read_input(100, 1)
        self.assertEqual(e.exception.code, 2)                          # illegal data address
        with self.assertRaises(ModbusException) as e: self.cli.read_holding(0, 0)
        self.assertEqual(e.exception.code, 3)                          # illegal data value
        self.assertEqual(handle_pdu(self.regs, bytes([0x2B, 0, 0])), bytes([0xAB, 1]))   # unsupported function

    def test_many_requests_on_one_connection(self):
        for i in range(500):
            self.cli.write_single(0, i); self.assertEqual(self.cli.read_holding(0), [i])


class PlantTests(unittest.TestCase):
    def test_open_loop_fills_and_settles_to_torricelli_equilibrium(self):
        srv = PlantServer(h0=0.2, noise_mm=0).start(run=False)
        c = ModbusClient(*srv.addr); c.write_single(0, 500)             # valve 50%: q_in = 0.01 = k*sqrt(h) -> h = 1 m
        time.sleep(0.05)
        p = srv.plant
        for _ in range(30000): p.step(0.01)                             # fast-forward 300 s of plant time
        self.assertAlmostEqual(p.h, 1.0, delta=0.01)
        c.close(); srv.stop()

    def test_interlock_overrides_controller(self):
        srv = PlantServer(h0=1.7, noise_mm=0).start(run=False)
        c = ModbusClient(*srv.addr); c.write_single(0, 1000)            # controller demands full open
        peak, tripped = 0.0, False
        for _ in range(4000):
            srv.plant.step(0.01)
            peak = max(peak, srv.plant.h * 1000); tripped |= bool(srv.regs.input[2] & 2)
        self.assertTrue(tripped)                                        # the interlock did engage
        self.assertLessEqual(peak, 1850)                                # and the level never ran away past the trip point
        c.close(); srv.stop()


class ControlTests(unittest.TestCase):
    def test_pid_tracks_setpoint_in_real_time_and_reports_latency(self):
        srv = PlantServer(h0=0.4).start()
        rec, lat = run_loop(*srv.addr, PID(0.6, 0.25, 0.0), 1000, rate_hz=100, seconds=14.0)
        srv.stop()
        tail = [r for r in rec if r[0] > 11]
        self.assertLess(sum(abs(r[1] - r[2]) for r in tail) / len(tail), 15)   # settled within 15 mm mean error
        s = summarize(rec, lat)
        self.assertGreater(s["cycles"], 1000)
        self.assertLess(s["lat_p50_us"], 5000)

    def test_hysteresis_stays_inside_band(self):
        srv = PlantServer(h0=0.4, noise_mm=0).start()
        rec, _ = run_loop(*srv.addr, Hysteresis(band=40), 1000, rate_hz=100, seconds=14.0)
        srv.stop()
        tail = [r[2] for r in rec if r[0] > 9]
        self.assertLess(max(tail), 1120); self.assertGreater(min(tail), 880)


if __name__ == "__main__":
    unittest.main()
