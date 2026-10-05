#include "canopyag_hardware/mks_bus.hpp"

namespace canopyag_hardware
{

bool MksBus::send(const mks::Frame & f)
{
  if (!sock_.send(f, last_error_)) {
    ++stats_.send_errors;
    return false;
  }
  ++stats_.tx;
  return true;
}

void MksBus::store(const mks::Frame & f)
{
  ++stats_.rx;
  if (!mks::crc_ok(f)) {
    ++stats_.bad_crc;
    return;
  }
  auto & q = buffer_[{f.id, f.cmd()}];
  if (q.size() >= kMaxPerKey) {
    q.pop_front();
    ++stats_.overflow;
  }
  q.push_back({f, Clock::now()});
}

void MksBus::pump(int timeout_ms)
{
  pump_until(Clock::now() + std::chrono::milliseconds(timeout_ms));
}

void MksBus::pump_until(Clock::time_point deadline)
{
  if (!sock_.is_open()) {
    return;
  }
  while (true) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - Clock::now()).count();
    mks::Frame f;
    std::string err;
    bool off = false;
    switch (sock_.recv(f, left > 0 ? static_cast<int>(left) : 0, err, off)) {
      case CanSocket::Recv::kFrame:
        store(f);
        continue;   // drain everything that is already there
      case CanSocket::Recv::kErrorFrame:
        ++stats_.error_frames;
        bus_off_ = bus_off_ || off;
        bus_error_ = err;
        continue;
      case CanSocket::Recv::kError:
        last_error_ = err;
        bus_error_ = err;
        return;
      case CanSocket::Recv::kTimeout:
        break;
    }
    if (Clock::now() >= deadline) {
      return;
    }
  }
}

void MksBus::discard(uint32_t id, uint8_t cmd)
{
  pump(0);
  buffer_.erase({id, cmd});
}

std::optional<MksBus::Reply> MksBus::take(uint32_t id, uint8_t cmd)
{
  auto it = buffer_.find({id, cmd});
  if (it == buffer_.end() || it->second.empty()) {
    return std::nullopt;
  }
  Reply r = it->second.front();
  it->second.pop_front();
  return r;
}

std::vector<MksBus::Reply> MksBus::take_all(uint32_t id, uint8_t cmd)
{
  std::vector<Reply> out;
  auto it = buffer_.find({id, cmd});
  if (it != buffer_.end()) {
    out.assign(it->second.begin(), it->second.end());
    buffer_.erase(it);
  }
  return out;
}

std::optional<MksBus::Reply> MksBus::wait(uint32_t id, uint8_t cmd, int timeout_ms)
{
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  while (true) {
    if (auto r = take(id, cmd)) {
      return r;
    }
    if (Clock::now() >= deadline || !sock_.is_open()) {
      return std::nullopt;
    }
    // Short slices, so a reply is picked up as soon as it lands.
    pump_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(1)));
  }
}

std::size_t MksBus::count(uint32_t id, uint8_t cmd) const
{
  auto it = buffer_.find({id, cmd});
  return it == buffer_.end() ? 0 : it->second.size();
}

bool MksBus::await_reply(uint32_t id, uint8_t cmd, std::size_t want, int timeout_ms)
{
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  while (true) {
    if (count(id, cmd) >= want) {
      return true;
    }
    if (Clock::now() >= deadline || !sock_.is_open()) {
      return false;
    }
    pump_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(1)));
  }
}

std::optional<MksBus::Reply> MksBus::request(const mks::Frame & f, int timeout_ms)
{
  discard(f.id, f.cmd());
  if (!send(f)) {
    return std::nullopt;
  }
  return wait(f.id, f.cmd(), timeout_ms);
}

std::string MksBus::take_bus_error()
{
  std::string e;
  e.swap(bus_error_);
  return e;
}

}  // namespace canopyag_hardware
