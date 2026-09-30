// One joint's motor mapping and EVERY joint <-> motor unit conversion.
//
// Lesson 7: a 360 deg motor move on a 16:1 link is 22.5 deg at the link,
// which once looked like "positioning is broken". All scaling lives here and
// nowhere else; test/test_conversion.cpp covers it both ways.
//
//   counts = zero_counts + sign * (joint - initial_position) * revs_per_unit * 16384
#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <string>

namespace canopyag_hardware
{

struct JointConfig
{
  enum class Mode { kCan, kVirtual };

  std::string name;
  Mode mode = Mode::kVirtual;
  uint32_t can_id = 0;
  double motor_revs_per_unit = 1.0;   // motor revs per rad or per metre
  bool reversed = false;
  int max_motor_rpm = 300;
  int acceleration = 200;
  double initial_position = 0.0;

  // Joint limits from the command interface's min/max (none: continuous).
  double min_position = -std::numeric_limits<double>::infinity();
  double max_position = std::numeric_limits<double>::infinity();

  double sign() const { return reversed ? -1.0 : 1.0; }
  double counts_per_unit() const;

  // Joint units <-> absolute encoder counts, given the zero reference.
  double counts_to_joint(int64_t counts, int64_t zero_counts) const;
  int64_t joint_to_counts(double joint, int64_t zero_counts) const;

  // Joint velocity (units/s) -> motor rpm magnitude (no clamping).
  double joint_vel_to_rpm(double joint_vel) const;
  double rpm_to_joint_vel(double rpm) const;

  // Clamp a target to [min_position, max_position].
  double clamp(double joint) const;
};

// Builds a JointConfig from the <param> tags of one <joint>. Throws
// std::invalid_argument with a message that names the joint and the param.
JointConfig parse_joint_config(
  const std::string & name, const std::map<std::string, std::string> & params,
  const std::string & min, const std::string & max);

bool parse_bool(const std::string & s);

}  // namespace canopyag_hardware
