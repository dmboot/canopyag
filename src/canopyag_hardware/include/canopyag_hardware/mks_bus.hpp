// The CAN bus as the MKS drivers see it: a socket plus a reply buffer keyed
// by (can_id, cmd).
//
// Lesson 1 from the Python test bench: a frame that arrives while we wait
// for a different one is STORED, never dropped - with several motors moving,
// dropped "move done" replies made moves time out. Stale replies for one
// (id, cmd) pair are discarded just before that command is sent again.
//
// Not thread-safe: owned by one thread at a time (the plugin's lifecycle
// callbacks, then the CAN thread while active).
#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "canopyag_hardware/can_socket.hpp"
#include "canopyag_hardware/mks_protocol.hpp"

namespace canopyag_hardware
{

class MksBus
{
public:
  using Clock = std::chrono::steady_clock;

  struct Reply
  {
    mks::Frame frame;
    Clock::time_point stamp;
  };

  struct Stats
  {
    uint64_t tx = 0;
    uint64_t rx = 0;
    uint64_t bad_crc = 0;
    uint64_t overflow = 0;     // buffered replies pushed out by newer ones
    uint64_t error_frames = 0;
    uint64_t send_errors = 0;
  };

  bool open(const std::string & ifname, std::string & err) { return sock_.open(ifname, err); }
  void close() { sock_.close(); buffer_.clear(); }
  bool is_open() const { return sock_.is_open(); }

  // Sends one frame. Failures are counted and kept in last_error().
  bool send(const mks::Frame & f);

  // Reads everything the socket has for up to timeout_ms into the buffer.
  // With timeout_ms > 0 it keeps reading until the deadline.
  void pump(int timeout_ms);
  void pump_until(Clock::time_point deadline);

  // Drops buffered replies for (id, cmd) - call right before re-sending cmd.
  void discard(uint32_t id, uint8_t cmd);

  // Oldest buffered reply for (id, cmd), removed from the buffer.
  std::optional<Reply> take(uint32_t id, uint8_t cmd);
  // Every buffered reply for (id, cmd), oldest first.
  std::vector<Reply> take_all(uint32_t id, uint8_t cmd);

  // Waits (pumping the socket) until a reply for (id, cmd) is buffered.
  std::optional<Reply> wait(uint32_t id, uint8_t cmd, int timeout_ms);

  // discard + send + wait.
  std::optional<Reply> request(const mks::Frame & f, int timeout_ms);

  // Controller / bus problems seen by pump() since the last call.
  bool bus_off() const { return bus_off_; }
  std::string take_bus_error();

  const Stats & stats() const { return stats_; }
  const std::string & last_error() const { return last_error_; }

private:
  static constexpr std::size_t kMaxPerKey = 64;

  void store(const mks::Frame & f);

  CanSocket sock_;
  std::map<std::pair<uint32_t, uint8_t>, std::deque<Reply>> buffer_;
  Stats stats_;
  bool bus_off_ = false;
  std::string bus_error_;
  std::string last_error_;
};

}  // namespace canopyag_hardware
