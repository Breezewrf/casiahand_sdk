// Exercise the real serial timeout/cleanup on a PTY, without robot hardware.
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <pty.h>
#include <string>
#include <unistd.h>
#include <vector>

#include <casia_hand_m.h>
#include <serial/serial.h>

namespace {
std::string test_port;
}

// Advertise only our PTY. The static serial library's list_ports object is not
// linked into this test; production SDK builds retain the real device discovery.
namespace serial {
std::vector<PortInfo> list_ports() {
  return {PortInfo{test_port, "CASIA test PTY", "test-only"}};
}
}

int main() {
  int master = -1, slave = -1;
  char name[128]{};
  if (openpty(&master, &slave, name, nullptr, nullptr) != 0) return 1;
  test_port = name;
  close(slave);
  casia::HandM::CasiaHandM hand(2, 32, 115200, test_port);
  const auto started_at = std::chrono::steady_clock::now();
  const bool initialized = hand.init(0.35);
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
  hand.shutdown();
  hand.shutdown();
  fcntl(master, F_SETFL, O_NONBLOCK);
  char request[64]{};
  const auto request_size = read(master, request, sizeof(request));
  close(master);
  if (initialized || elapsed < 0.30 || elapsed > 0.70 || request_size <= 0) {
    std::cerr << "unexpected native initialization result: " << initialized << ", " << elapsed << " s\n";
    return 1;
  }
  return 0;
}
