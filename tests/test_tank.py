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

