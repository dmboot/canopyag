// Joint <-> motor unit conversion both ways, limits and param validation.
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include "canopyag_hardware/joint_config.hpp"
#include "canopyag_hardware/mks_protocol.hpp"

using namespace canopyag_hardware;

static JointConfig joint_1()
{
  // 16:1 revolute, as in hardware.yaml
  return parse_joint_config(
    "joint_1",
    {{"mode", "can"}, {"can_id", "3"}, {"motor_revs_per_unit", "2.5464790894703255"},   // 16 / (2 pi)
      {"reversed", "False"}, {"max_motor_rpm", "300"}, {"acceleration", "230"},
      {"initial_position", "0.0"}},
    "-1.65", "1.65");
}

TEST(Conversion, SixteenToOneRevolute)
{
  const auto c = joint_1();
  // one link revolution = 16 motor revs
  EXPECT_EQ(c.joint_to_counts(2 * M_PI, 0), 16 * mks::kCountsPerRev);
  // lesson 7: a 360 deg MOTOR move is 22.5 deg at the link
  EXPECT_NEAR(c.counts_to_joint(mks::kCountsPerRev, 0), 22.5 * M_PI / 180.0, 1e-9);
  EXPECT_NEAR(c.counts_to_joint(c.joint_to_counts(M_PI / 2, 0), 0), M_PI / 2, 1e-4);
  // 1 rad/s at the link = 16/(2 pi) rev/s = 152.8 motor rpm
  EXPECT_NEAR(c.joint_vel_to_rpm(1.0), 16.0 / (2 * M_PI) * 60.0, 1e-9);
  EXPECT_NEAR(c.rpm_to_joint_vel(c.joint_vel_to_rpm(-0.7)), 0.7, 1e-12);
}

TEST(Conversion, ZeroOffsetAndInitialPosition)
{
  auto c = joint_1();
  c.initial_position = 0.5;
  const int64_t zero = 123456;
  EXPECT_DOUBLE_EQ(c.counts_to_joint(zero, zero), 0.5);
  EXPECT_EQ(c.joint_to_counts(0.5, zero), zero);
  EXPECT_NEAR(c.counts_to_joint(c.joint_to_counts(-1.0, zero), zero), -1.0, 1e-4);
}

TEST(Conversion, Reversed)
{
  auto c = joint_1();
  c.reversed = true;
  EXPECT_EQ(c.joint_to_counts(2 * M_PI, 1000), 1000 - 16 * mks::kCountsPerRev);
  EXPECT_NEAR(c.counts_to_joint(1000 - mks::kCountsPerRev, 1000), 22.5 * M_PI / 180.0, 1e-9);
}

TEST(Conversion, BeltPrismatic)
{
  // z_carriage: 5:1 into a 20T GT2 pulley = 8 mm per motor rev = 125 rev/m
  const auto c = parse_joint_config(
    "z_carriage",
    {{"mode", "can"}, {"can_id", "1"}, {"motor_revs_per_unit", "125.0"}, {"reversed", "false"},
      {"max_motor_rpm", "600"}, {"acceleration", "200"}},
    "0", "1.22");
  EXPECT_EQ(c.joint_to_counts(0.008, 0), mks::kCountsPerRev);
  EXPECT_NEAR(c.rpm_to_joint_vel(600), 0.08, 1e-12);
  // full stroke fits the i24 F5 range
  EXPECT_LT(c.joint_to_counts(1.22, 0), mks::kI24Max);
}

TEST(Limits, ClampToMinMax)
{
  const auto c = joint_1();
  EXPECT_DOUBLE_EQ(c.clamp(2.0), 1.65);
  EXPECT_DOUBLE_EQ(c.clamp(-2.0), -1.65);
  EXPECT_DOUBLE_EQ(c.clamp(0.3), 0.3);
}

TEST(Limits, ContinuousHasNoLimits)
{
  const auto c = parse_joint_config("joint_2", {{"mode", "virtual"}}, "", "");
  EXPECT_EQ(c.mode, JointConfig::Mode::kVirtual);
  EXPECT_DOUBLE_EQ(c.clamp(1e6), 1e6);
}

TEST(Limits, I24RangeForContinuousGearedJoint)
{
  // 16:1 continuous: +-512 motor revs = +-32 link revs from the driver's zero
  const auto c = joint_1();
  const double top = c.counts_to_joint(mks::kI24Max, 0);
  EXPECT_NEAR(top / (2 * M_PI), 512.0 / 16.0, 1e-3);
  EXPECT_EQ(mks::clamp_i24(c.joint_to_counts(100 * 2 * M_PI, 0)), mks::kI24Max);
}

TEST(Params, MissingAndInvalid)
{
  const std::map<std::string, std::string> ok{
    {"mode", "can"}, {"can_id", "3"}, {"motor_revs_per_unit", "2.5"}, {"reversed", "false"},
    {"max_motor_rpm", "300"}, {"acceleration", "230"}};
  EXPECT_NO_THROW(parse_joint_config("j", ok, "", ""));
  for (const auto & key : {"mode", "can_id", "motor_revs_per_unit", "reversed", "max_motor_rpm",
      "acceleration"})
  {
    auto p = ok;
    p.erase(key);
    EXPECT_THROW(parse_joint_config("j", p, "", ""), std::invalid_argument) << key;
  }
  auto bad = ok;
  bad["max_motor_rpm"] = "3001";
  EXPECT_THROW(parse_joint_config("j", bad, "", ""), std::invalid_argument);
  bad = ok;
  bad["acceleration"] = "256";
  EXPECT_THROW(parse_joint_config("j", bad, "", ""), std::invalid_argument);
  bad = ok;
  bad["motor_revs_per_unit"] = "-1";
  EXPECT_THROW(parse_joint_config("j", bad, "", ""), std::invalid_argument);
  bad = ok;
  bad["mode"] = "sim";
  EXPECT_THROW(parse_joint_config("j", bad, "", ""), std::invalid_argument);
  bad = ok;
  bad["reversed"] = "maybe";
  EXPECT_THROW(parse_joint_config("j", bad, "", ""), std::invalid_argument);
  bad = ok;
  bad["initial_position"] = "5";
  EXPECT_THROW(parse_joint_config("j", bad, "-1", "1"), std::invalid_argument);
}
