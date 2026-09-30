#include "canopyag_hardware/mks_protocol.hpp"

#include <algorithm>
#include <cstdio>
#include <stdexcept>

namespace canopyag_hardware
{
namespace mks
{

std::string Frame::hex() const
{
  char buf[64];
  int n = std::snprintf(buf, sizeof(buf), "%03X#", id);
  for (uint8_t i = 0; i < len && n < static_cast<int>(sizeof(buf)) - 4; ++i) {
    n += std::snprintf(buf + n, sizeof(buf) - n, i ? " %02X" : "%02X", data[i]);
  }
  return std::string(buf, n);
}

uint8_t crc(uint32_t can_id, const uint8_t * bytes, std::size_t n)
{
  uint32_t sum = can_id;
  for (std::size_t i = 0; i < n; ++i) {
    sum += bytes[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

bool crc_ok(const Frame & f)
{
  return f.len >= 2 && crc(f.id, f.data.data(), f.len - 1) == f.data[f.len - 1];
}

Frame make(uint32_t can_id, uint8_t cmd, std::initializer_list<uint8_t> params)
{
  if (params.size() > 6) {
    throw std::invalid_argument("MKS frame: at most 6 parameter bytes");
  }
  Frame f;
  f.id = can_id;
  f.data[0] = cmd;
  std::copy(params.begin(), params.end(), f.data.begin() + 1);
  f.len = static_cast<uint8_t>(params.size() + 2);
  f.data[f.len - 1] = crc(can_id, f.data.data(), f.len - 1);
  return f;
}

static Frame axis_move(uint32_t can_id, uint8_t cmd, uint16_t rpm, uint8_t acc, int32_t counts)
{
  if (rpm > kMaxRpm) {
    throw std::invalid_argument("MKS move: rpm above 3000");
  }
  if (counts < kI24Min || counts > kI24Max) {
    throw std::invalid_argument("MKS move: counts outside the i24 range");
  }
  uint8_t c[3];
  put_i24(c, counts);
  return make(can_id, cmd, {
      static_cast<uint8_t>(rpm >> 8), static_cast<uint8_t>(rpm & 0xFF), acc, c[0], c[1], c[2]});
}

Frame enable(uint32_t can_id, bool on) { return make(can_id, kEnable, {on ? uint8_t{1} : uint8_t{0}}); }

Frame abs_move(uint32_t can_id, uint16_t rpm, uint8_t acc, int32_t counts)
{
  return axis_move(can_id, kAbsMove, rpm, acc, counts);
}

Frame rel_move(uint32_t can_id, uint16_t rpm, uint8_t acc, int32_t counts)
{
  return axis_move(can_id, kRelMove, rpm, acc, counts);
}

Frame speed_mode(uint32_t can_id, uint16_t rpm, uint8_t acc, bool reverse)
{
  if (rpm > kMaxRpm) {
    throw std::invalid_argument("MKS speed mode: rpm above 3000");
  }
  const uint8_t b1 = static_cast<uint8_t>((reverse ? 0x80 : 0x00) | ((rpm >> 8) & 0x0F));
  return make(can_id, kSpeedMode, {b1, static_cast<uint8_t>(rpm & 0xFF), acc});
}

Frame estop(uint32_t can_id) { return make(can_id, kEStop); }
Frame read_encoder(uint32_t can_id) { return make(can_id, kReadEncoder); }
Frame read_speed(uint32_t can_id) { return make(can_id, kReadSpeed); }
Frame read_enabled(uint32_t can_id) { return make(can_id, kReadEnabled); }
Frame read_stall(uint32_t can_id) { return make(can_id, kReadStall); }
Frame release_stall(uint32_t can_id) { return make(can_id, kReleaseStall); }
Frame set_zero(uint32_t can_id) { return make(can_id, kSetZero); }

std::optional<int64_t> parse_encoder(const Frame & f)
{
  if (f.len != 8 || f.cmd() != kReadEncoder || !crc_ok(f)) {
    return std::nullopt;
  }
  return get_i48(f.data.data() + 1);
}

std::optional<int16_t> parse_speed(const Frame & f)
{
  if (f.len != 4 || f.cmd() != kReadSpeed || !crc_ok(f)) {
    return std::nullopt;
  }
  return static_cast<int16_t>((f.data[1] << 8) | f.data[2]);
}

std::optional<uint8_t> parse_status(const Frame & f, uint8_t cmd)
{
  if (f.len != 3 || f.cmd() != cmd || !crc_ok(f)) {
    return std::nullopt;
  }
  return f.data[1];
}

void put_i24(uint8_t * out, int32_t v)
{
  const uint32_t u = static_cast<uint32_t>(v) & 0xFFFFFF;
  out[0] = static_cast<uint8_t>(u >> 16);
  out[1] = static_cast<uint8_t>(u >> 8);
  out[2] = static_cast<uint8_t>(u);
}

int32_t get_i24(const uint8_t * in)
{
  int32_t v = (in[0] << 16) | (in[1] << 8) | in[2];
  if (v & 0x800000) {
    v -= 0x1000000;
  }
  return v;
}

int64_t get_i48(const uint8_t * in)
{
  int64_t v = 0;
  for (int i = 0; i < 6; ++i) {
    v = (v << 8) | in[i];
  }
  if (v & (int64_t{1} << 47)) {
    v -= int64_t{1} << 48;
  }
  return v;
}

int32_t clamp_i24(int64_t v)
{
  return static_cast<int32_t>(std::clamp<int64_t>(v, kI24Min, kI24Max));
}

}  // namespace mks
}  // namespace canopyag_hardware
