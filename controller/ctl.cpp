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
