"""Minimal Modbus/TCP (client + server) from the public spec, standard library only.

Frame = MBAP header (7 bytes) + PDU.
  MBAP: transaction id u16 | protocol id u16 (=0) | length u16 (unit id + PDU) | unit id u8
Function codes: 0x03 read holding registers, 0x04 read input registers, 0x06 write single register,
0x10 write multiple registers. Errors reply with fc|0x80 and an exception code (1 illegal function,
2 illegal data address, 3 illegal data value).
"""
import socket
import socketserver
import struct
import threading

READ_HOLDING, READ_INPUT, WRITE_SINGLE, WRITE_MULTIPLE = 0x03, 0x04, 0x06, 0x10


class ModbusException(Exception):
    def __init__(self, code):
        super().__init__(f"modbus exception {code}")
        self.code = code


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        part = sock.recv(n - len(buf))
        if not part:
            raise ConnectionError("closed")
        buf += part
    return bytes(buf)


def frame(tid, unit, pdu):
    return struct.pack(">HHHB", tid, 0, len(pdu) + 1, unit) + pdu


def read_frame(sock):
    tid, pid, length, unit = struct.unpack(">HHHB", recv_exact(sock, 7))
    if pid != 0 or not 1 <= length <= 254:
        raise ConnectionError("bad MBAP header")
    return tid, unit, recv_exact(sock, length - 1)


