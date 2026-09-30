// Frame builders, parsers and CRC against the vectors in the task spec,
// which match the MKS manual and the Python test bench.
#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "canopyag_hardware/mks_protocol.hpp"

using namespace canopyag_hardware;

static std::vector<uint8_t> bytes(const mks::Frame & f)
{
  return {f.data.begin(), f.data.begin() + f.len};
}

TEST(Crc, IdPlusSumOfBytes)
{
  const uint8_t d[] = {0xF6, 0x01, 0x40, 0x02};
  EXPECT_EQ(mks::crc(1, d, 4), 0x3A);
  EXPECT_EQ(mks::crc(0x100, d, 0), 0x00);   // only the low byte of the ID counts
}

TEST(Builders, SpeedModeMatchesManual)
{
  // 01#F6 01 40 02 3A : 320 rpm, acc 2, dir 0
  EXPECT_EQ(bytes(mks::speed_mode(1, 320, 2, false)), (std::vector<uint8_t>{0xF6, 0x01, 0x40, 0x02, 0x3A}));
  EXPECT_EQ(mks::speed_mode(1, 320, 2, true).data[1], 0x81);
}

TEST(Builders, RelativeMovePositive)
{
  // 01#F4 00 C8 32 00 40 00 2F
  EXPECT_EQ(bytes(mks::rel_move(1, 200, 50, 16384)),
    (std::vector<uint8_t>{0xF4, 0x00, 0xC8, 0x32, 0x00, 0x40, 0x00, 0x2F}));
}

TEST(Builders, RelativeMoveNegative)
{
  // 01#F4 00 C8 32 FF F0 00 DE
  EXPECT_EQ(bytes(mks::rel_move(1, 200, 50, -4096)),
    (std::vector<uint8_t>{0xF4, 0x00, 0xC8, 0x32, 0xFF, 0xF0, 0x00, 0xDE}));
}

TEST(Builders, Enable)
{
  // 01#F3 01 F5
  EXPECT_EQ(bytes(mks::enable(1, true)), (std::vector<uint8_t>{0xF3, 0x01, 0xF5}));
  EXPECT_EQ(bytes(mks::enable(1, false)), (std::vector<uint8_t>{0xF3, 0x00, 0xF4}));
}

TEST(Builders, AbsoluteMoveSameLayoutAsRelative)
{
  auto f = mks::abs_move(3, 200, 50, -4096);
  EXPECT_EQ(f.id, 3u);
  EXPECT_EQ(f.len, 8);
  EXPECT_EQ(f.data[0], 0xF5);
  EXPECT_EQ(mks::get_i24(f.data.data() + 4), -4096);
  EXPECT_TRUE(mks::crc_ok(f));
}

TEST(Builders, NoParamCommands)
{
  EXPECT_EQ(bytes(mks::estop(1)), (std::vector<uint8_t>{0xF7, 0xF8}));
  EXPECT_EQ(bytes(mks::read_encoder(1)), (std::vector<uint8_t>{0x31, 0x32}));
  EXPECT_EQ(bytes(mks::read_stall(2)), (std::vector<uint8_t>{0x3E, 0x40}));
}

TEST(Builders, RejectOutOfRange)
{
  EXPECT_THROW(mks::abs_move(1, 3001, 0, 0), std::invalid_argument);
  EXPECT_THROW(mks::abs_move(1, 100, 0, mks::kI24Max + 1), std::invalid_argument);
  EXPECT_THROW(mks::abs_move(1, 100, 0, mks::kI24Min - 1), std::invalid_argument);
  EXPECT_NO_THROW(mks::abs_move(1, 3000, 255, mks::kI24Min));
}

TEST(SignedFields, I24RoundTrip)
{
  for (int32_t v : {0, 1, -1, 16384, -4096, mks::kI24Max, mks::kI24Min}) {
    uint8_t b[3];
    mks::put_i24(b, v);
    EXPECT_EQ(mks::get_i24(b), v) << v;
  }
  EXPECT_EQ(mks::clamp_i24(int64_t{1} << 40), mks::kI24Max);
  EXPECT_EQ(mks::clamp_i24(-(int64_t{1} << 40)), mks::kI24Min);
  EXPECT_EQ(mks::clamp_i24(-5), -5);
}

TEST(SignedFields, I48)
{
  const uint8_t pos[6] = {0x00, 0x00, 0x00, 0x00, 0x40, 0x00};
  const uint8_t neg[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00};
  const uint8_t min[6] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(mks::get_i48(pos), 16384);
  EXPECT_EQ(mks::get_i48(neg), -4096);
  EXPECT_EQ(mks::get_i48(min), -(int64_t{1} << 47));
}

static mks::Frame reply(uint32_t id, std::vector<uint8_t> body)
{
  mks::Frame f;
  f.id = id;
  f.len = static_cast<uint8_t>(body.size() + 1);
  std::copy(body.begin(), body.end(), f.data.begin());
  f.data[body.size()] = mks::crc(id, body.data(), body.size());
  return f;
}

TEST(Parsers, Encoder)
{
  EXPECT_EQ(mks::parse_encoder(reply(1, {0x31, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00})), -4096);
  EXPECT_EQ(mks::parse_encoder(reply(1, {0x31, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00})), 65536);
  auto bad = reply(1, {0x31, 0, 0, 0, 0, 0, 1});
  bad.data[7] ^= 1;
  EXPECT_FALSE(mks::parse_encoder(bad));                          // CRC
  EXPECT_FALSE(mks::parse_encoder(reply(1, {0x32, 0, 0, 0, 0, 0, 0})));  // wrong cmd
  EXPECT_FALSE(mks::parse_encoder(reply(1, {0x31, 0, 0})));       // wrong length
}

TEST(Parsers, SpeedAndStatus)
{
  EXPECT_EQ(mks::parse_speed(reply(1, {0x32, 0xFF, 0xFF})), -1);
  EXPECT_EQ(mks::parse_speed(reply(1, {0x32, 0x01, 0x40})), 320);
  EXPECT_EQ(mks::parse_status(reply(1, {0xF5, 0x02}), 0xF5), 2);
  EXPECT_FALSE(mks::parse_status(reply(1, {0xF5, 0x02}), 0xF4));
  EXPECT_EQ(mks::parse_status(reply(2, {0x3E, 0x01}), 0x3E), 1);
}

TEST(Frame, Hex)
{
  EXPECT_EQ(mks::enable(1, true).hex(), "001#F3 01 F5");
}
