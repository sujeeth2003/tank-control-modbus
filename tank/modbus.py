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


class Registers:
    """Thread-safe register bank shared between the protocol server and the plant model."""

    def __init__(self, n_input=16, n_holding=16):
        self.input = [0] * n_input
        self.holding = [0] * n_holding
        self.lock = threading.Lock()


def handle_pdu(regs: Registers, pdu: bytes) -> bytes:
    fc = pdu[0]
    try:
        if fc in (READ_HOLDING, READ_INPUT):
            addr, qty = struct.unpack(">HH", pdu[1:5])
            bank = regs.holding if fc == READ_HOLDING else regs.input
            if not 1 <= qty <= 125:
                raise ModbusException(3)
            if addr + qty > len(bank):
                raise ModbusException(2)
            with regs.lock:
                vals = bank[addr:addr + qty]
            return bytes([fc, qty * 2]) + struct.pack(f">{qty}H", *vals)
        if fc == WRITE_SINGLE:
            addr, val = struct.unpack(">HH", pdu[1:5])
            if addr >= len(regs.holding):
                raise ModbusException(2)
            with regs.lock:
                regs.holding[addr] = val
            return pdu[:5]
        if fc == WRITE_MULTIPLE:
            addr, qty, nbytes = struct.unpack(">HHB", pdu[1:6])
            if not 1 <= qty <= 123 or nbytes != qty * 2:
                raise ModbusException(3)
            if addr + qty > len(regs.holding):
                raise ModbusException(2)
            vals = struct.unpack(f">{qty}H", pdu[6:6 + nbytes])
            with regs.lock:
                regs.holding[addr:addr + qty] = list(vals)
            return pdu[:5]
        raise ModbusException(1)
    except ModbusException as e:
        return bytes([fc | 0x80, e.code])
    except struct.error:
        return bytes([fc | 0x80, 3])


class _Handler(socketserver.BaseRequestHandler):
    def handle(self):
        s = self.request
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            while True:                                      # persistent connection: many requests per socket
                tid, unit, pdu = read_frame(s)
                s.sendall(frame(tid, unit, handle_pdu(self.server.regs, pdu)))
        except (ConnectionError, OSError):
            pass


class ModbusServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, regs, host="127.0.0.1", port=0):
        super().__init__((host, port), _Handler)
        self.regs = regs
        self.addr = self.server_address

    def start(self):
        threading.Thread(target=self.serve_forever, daemon=True).start()
        return self

    def stop(self):
        self.shutdown(); self.server_close()


class ModbusClient:
    def __init__(self, host, port, unit=1, timeout=2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.unit, self.tid = unit, 0

    def _call(self, pdu):
        self.tid = (self.tid + 1) & 0xFFFF
        self.sock.sendall(frame(self.tid, self.unit, pdu))
        tid, _, resp = read_frame(self.sock)
        if tid != self.tid:
            raise ConnectionError("transaction id mismatch")
        if resp[0] & 0x80:
            raise ModbusException(resp[1])
        return resp

    def read_input(self, addr, qty=1):
        r = self._call(struct.pack(">BHH", READ_INPUT, addr, qty))
        return list(struct.unpack(f">{r[1] // 2}H", r[2:]))

    def read_holding(self, addr, qty=1):
        r = self._call(struct.pack(">BHH", READ_HOLDING, addr, qty))
        return list(struct.unpack(f">{r[1] // 2}H", r[2:]))

    def write_single(self, addr, value):
        self._call(struct.pack(">BHH", WRITE_SINGLE, addr, value & 0xFFFF))

    def write_multiple(self, addr, values):
        self._call(struct.pack(">BHHB", WRITE_MULTIPLE, addr, len(values), len(values) * 2) + struct.pack(f">{len(values)}H", *values))

    def close(self):
        self.sock.close()
