#include "io/reconnecting_serial.hpp"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pty.h>
#include <string>
#include <thread>
#include <unistd.h>

struct Pty { int master = -1, slave = -1; char path[256]{}; };
static Pty new_terminal()
{
  Pty p;
  assert(openpty(&p.master, &p.slave, p.path, nullptr, nullptr) == 0);
  return p;
}

int main()
{
  char directory[] = "/tmp/vision-serial-XXXXXX";
  assert(mkdtemp(directory) != nullptr);
  const std::string alias = std::string(directory) + "/gimbal";
  Pty old_port = new_terminal();
  assert(symlink(old_port.path, alias.c_str()) == 0);
  {
    io::ReconnectingSerial link(alias, 115200, 50, "test");
    assert(link.ensure_open());
    const auto previous_generation = link.generation();
    uint8_t out[] = {'A', 'B'};
    assert(link.write(out, 2));
    uint8_t rx[2]{};
    assert(::read(old_port.master, rx, 2) == 2);
    assert(rx[0] == 'A' && rx[1] == 'B');

    // Device removed while process still owns an open file descriptor.
    assert(unlink(alias.c_str()) == 0);
    assert(!link.connected());
    assert(!link.write(out, 2));

    close(old_port.master);
    close(old_port.slave);
    Pty newer = new_terminal();
    assert(symlink(newer.path, alias.c_str()) == 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    assert(link.ensure_open());
    assert(link.generation() > previous_generation);
    assert(link.write(out, 2));
    uint8_t rx2[2]{};
    assert(::read(newer.master, rx2, 2) == 2);
    assert(rx2[0] == 'A' && rx2[1] == 'B');

    close(newer.master);
    close(newer.slave);
  }
  assert(unlink(alias.c_str()) == 0);
  assert(rmdir(directory) == 0);
}
