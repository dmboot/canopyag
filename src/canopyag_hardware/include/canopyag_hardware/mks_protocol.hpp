// MKS SERVO57D CAN protocol: frame builders, reply parsers and CRC.
//
// Pure functions, no I/O and no ROS - covered by test/test_mks_protocol.cpp.
// Reference: ~/nema_test/mks_servo.py, which has been run on the real bus.
//
// Framing: standard 11-bit ID = the driver's CanID; replies use the same ID.
//   data = [cmd, params..., CRC],  CRC = (CAN_ID + sum(cmd, params)) & 0xFF
// Multi-byte values are big-endian two's complement.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>

namespace canopyag_hardware
{
namespace mks
{

constexpr int kCountsPerRev = 16384;       // 14-bit encoder on the motor shaft
constexpr int32_t kI24Max = 0x7FFFFF;      // F4/F5 position range (~ +-512 revs)
constexpr int32_t kI24Min = -0x800000;
constexpr int kMaxRpm = 3000;

enum Cmd : uint8_t
{
  kReadEncoder = 0x31,    // -> [31, i48 counts, CRC]
  kReadSpeed = 0x32,      // -> [32, i16 rpm, CRC]
  kReadEnabled = 0x3A,    // -> [3A, u8, CRC]
  kReleaseStall = 0x3D,   // -> [3D, status, CRC]
  kReadStall = 0x3E,      // -> [3E, u8 1 = tripped, CRC]
  kSetZero = 0x92,        // -> [92, status, CRC]
  kEnable = 0xF3,         // u8 0/1 -> [F3, status, CRC]
  kRelMove = 0xF4,        // u16 rpm, u8 acc, i24 counts
  kAbsMove = 0xF5,        // u16 rpm, u8 acc, i24 counts
  kSpeedMode = 0xF6,      // dir|rpm_hi, rpm_lo, acc
  kEStop = 0xF7,
};

// Move statuses in F4/F5 replies.
enum MoveStatus : uint8_t
{
  kMoveFailed = 0,
  kMoveStarted = 1,
  kMoveDone = 2,
  kMoveLimit = 3,
};

struct Frame
{
  uint32_t id = 0;
  uint8_t len = 0;
  std::array<uint8_t, 8> data{};

  uint8_t cmd() const { return len > 0 ? data[0] : 0; }
  std::string hex() const;   // "001#F5 00 C8 ..." for logs
};

uint8_t crc(uint32_t can_id, const uint8_t * bytes, std::size_t n);
bool crc_ok(const Frame & f);

// [cmd, params..., CRC]
Frame make(uint32_t can_id, uint8_t cmd, std::initializer_list<uint8_t> params = {});

// ---- builders ----
Frame enable(uint32_t can_id, bool on);
Frame abs_move(uint32_t can_id, uint16_t rpm, uint8_t acc, int32_t counts);
Frame rel_move(uint32_t can_id, uint16_t rpm, uint8_t acc, int32_t counts);
Frame speed_mode(uint32_t can_id, uint16_t rpm, uint8_t acc, bool reverse);
Frame estop(uint32_t can_id);
Frame read_encoder(uint32_t can_id);
Frame read_speed(uint32_t can_id);
Frame read_enabled(uint32_t can_id);
Frame read_stall(uint32_t can_id);
Frame release_stall(uint32_t can_id);
Frame set_zero(uint32_t can_id);

// ---- parsers: nullopt on wrong command, wrong length or bad CRC ----
std::optional<int64_t> parse_encoder(const Frame & f);                 // 0x31
std::optional<int16_t> parse_speed(const Frame & f);                   // 0x32
std::optional<uint8_t> parse_status(const Frame & f, uint8_t cmd);     // [cmd, u8, CRC]

// ---- big-endian signed helpers ----
void put_i24(uint8_t * out, int32_t v);       // v must already be in i24 range
int32_t get_i24(const uint8_t * in);
int64_t get_i48(const uint8_t * in);
int32_t clamp_i24(int64_t v);

}  // namespace mks
}  // namespace canopyag_hardware
