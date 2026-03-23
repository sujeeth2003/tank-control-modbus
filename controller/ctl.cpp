// High-rate C++ tank controller over Modbus/TCP with per-decision latency measurement.
//
//   ctl <host> <port> <setpoint_mm> <rate_hz> <seconds>
//
// Each cycle: read input register 0 (level) -> PID -> write holding register 0 (valve). The measured latency is the
// full sensor-to-actuator round trip: from just before the read request is sent until the write is acknowledged.
// Pure C++17, raw sockets, no dependencies (Winsock on Windows, POSIX elsewhere). Persistent connection, TCP_NODELAY.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  using sock_t = SOCKET;
  static void sock_close(sock_t s) { closesocket(s); }
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <sys/socket.h>
  #include <unistd.h>
  using sock_t = int;
  static void sock_close(sock_t s) { close(s); }
#endif

using clk = std::chrono::steady_clock;

static bool send_all(sock_t s, const uint8_t* p, size_t n) {
  while (n) { int k = send(s, (const char*)p, (int)n, 0); if (k <= 0) return false; p += k; n -= k; }
  return true;
}
static bool recv_all(sock_t s, uint8_t* p, size_t n) {
  while (n) { int k = recv(s, (char*)p, (int)n, 0); if (k <= 0) return false; p += k; n -= k; }
  return true;
}

struct Modbus {
  sock_t s{}; uint16_t tid = 0;
  // one request/response transaction; returns false on any error or Modbus exception
  bool call(const uint8_t* pdu, size_t n, uint8_t* resp, size_t& rn) {
    uint8_t buf[260]; ++tid;
    buf[0] = tid >> 8; buf[1] = tid & 0xFF; buf[2] = buf[3] = 0;
    buf[4] = (uint8_t)((n + 1) >> 8); buf[5] = (uint8_t)((n + 1) & 0xFF); buf[6] = 1;
    std::memcpy(buf + 7, pdu, n);
    if (!send_all(s, buf, 7 + n)) return false;
    uint8_t h[7];
    if (!recv_all(s, h, 7)) return false;
    size_t len = ((size_t)h[4] << 8 | h[5]) - 1;
    if (len == 0 || len > 253 || !recv_all(s, resp, len)) return false;
    rn = len;
    return !(resp[0] & 0x80) && (((uint16_t)h[0] << 8 | h[1]) == tid);
  }
  bool read_input(uint16_t addr, uint16_t& out) {
    uint8_t pdu[5] = {0x04, (uint8_t)(addr >> 8), (uint8_t)addr, 0, 1}, r[260]; size_t rn;
    if (!call(pdu, 5, r, rn) || rn < 4) return false;
    out = (uint16_t)(r[2] << 8 | r[3]);
    return true;
  }
  bool write_single(uint16_t addr, uint16_t v) {
    uint8_t pdu[5] = {0x06, (uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)(v >> 8), (uint8_t)v}, r[260]; size_t rn;
    return call(pdu, 5, r, rn);
  }
};

int main(int argc, char** argv) {
  if (argc < 6) { std::fprintf(stderr, "usage: %s host port setpoint_mm rate_hz seconds\n", argv[0]); return 2; }
  const char* host = argv[1]; int port = std::atoi(argv[2]);
  double sp = std::atof(argv[3]), rate = std::atof(argv[4]), seconds = std::atof(argv[5]);
#ifdef _WIN32
  WSADATA wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
  sock_t s = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port); inet_pton(AF_INET, host, &a.sin_addr);
  if (connect(s, (sockaddr*)&a, sizeof a) != 0) { std::perror("connect"); return 1; }
  int one = 1; setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof one);
  Modbus mb{s};

