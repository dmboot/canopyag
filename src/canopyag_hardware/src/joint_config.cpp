#include "canopyag_hardware/joint_config.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "canopyag_hardware/mks_protocol.hpp"

namespace canopyag_hardware
{

double JointConfig::counts_per_unit() const
{
  return motor_revs_per_unit * mks::kCountsPerRev;
}

double JointConfig::counts_to_joint(int64_t counts, int64_t zero_counts) const
{
  return initial_position + sign() * static_cast<double>(counts - zero_counts) / counts_per_unit();
}

int64_t JointConfig::joint_to_counts(double joint, int64_t zero_counts) const
{
  return zero_counts + std::llround(sign() * (joint - initial_position) * counts_per_unit());
}

double JointConfig::joint_vel_to_rpm(double joint_vel) const
{
  return std::abs(joint_vel) * motor_revs_per_unit * 60.0;
}

double JointConfig::rpm_to_joint_vel(double rpm) const
{
  return rpm / 60.0 / motor_revs_per_unit;
}

double JointConfig::clamp(double joint) const
{
  return std::clamp(joint, min_position, max_position);
}

bool parse_bool(const std::string & s)
{
  if (s == "true" || s == "True" || s == "TRUE" || s == "1") {
    return true;
  }
  if (s == "false" || s == "False" || s == "FALSE" || s == "0") {
    return false;
  }
  throw std::invalid_argument("'" + s + "' is not a bool");
}

namespace
{

const std::string & need(
  const std::string & joint, const std::map<std::string, std::string> & p, const std::string & key)
{
  auto it = p.find(key);
  if (it == p.end() || it->second.empty()) {
    throw std::invalid_argument(
            "joint '" + joint + "': missing <param name=\"" + key +
            "\"> (set it in canopyag_description/config/hardware.yaml)");
  }
  return it->second;
}

double to_double(const std::string & joint, const std::string & key, const std::string & v)
{
  try {
    std::size_t used = 0;
    const double d = std::stod(v, &used);
    if (used != v.size() || !std::isfinite(d)) {
      throw std::invalid_argument("");
    }
    return d;
  } catch (const std::exception &) {
    throw std::invalid_argument("joint '" + joint + "': " + key + " = '" + v + "' is not a number");
  }
}

int to_int(const std::string & joint, const std::string & key, const std::string & v)
{
  const double d = to_double(joint, key, v);
  if (d != std::floor(d)) {
    throw std::invalid_argument("joint '" + joint + "': " + key + " = '" + v + "' must be an integer");
  }
  return static_cast<int>(d);
}

}  // namespace

JointConfig parse_joint_config(
  const std::string & name, const std::map<std::string, std::string> & p,
  const std::string & min, const std::string & max)
{
  JointConfig c;
  c.name = name;

  const std::string & mode = need(name, p, "mode");
  if (mode == "can") {
    c.mode = JointConfig::Mode::kCan;
  } else if (mode == "virtual") {
    c.mode = JointConfig::Mode::kVirtual;
  } else {
    throw std::invalid_argument("joint '" + name + "': mode '" + mode + "' must be can or virtual");
  }

  if (auto it = p.find("initial_position"); it != p.end()) {
    c.initial_position = to_double(name, "initial_position", it->second);
  }
  if (!min.empty()) {
    c.min_position = to_double(name, "min", min);
  }
  if (!max.empty()) {
    c.max_position = to_double(name, "max", max);
  }
  if (c.min_position > c.max_position) {
    throw std::invalid_argument("joint '" + name + "': min > max");
  }
  if (c.initial_position < c.min_position || c.initial_position > c.max_position) {
    throw std::invalid_argument("joint '" + name + "': initial_position outside [min, max]");
  }
  if (c.mode == JointConfig::Mode::kVirtual) {
    return c;
  }

  const int id = to_int(name, "can_id", need(name, p, "can_id"));
  if (id < 1 || id > 0x7FF) {
    throw std::invalid_argument("joint '" + name + "': can_id must be 1..2047");
  }
  c.can_id = static_cast<uint32_t>(id);

  c.motor_revs_per_unit = to_double(name, "motor_revs_per_unit", need(name, p, "motor_revs_per_unit"));
  if (c.motor_revs_per_unit <= 0.0) {
    throw std::invalid_argument(
            "joint '" + name + "': motor_revs_per_unit must be > 0 (use reversed: true to flip)");
  }
  try {
    c.reversed = parse_bool(need(name, p, "reversed"));
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument("joint '" + name + "': reversed: " + e.what());
  }

  c.max_motor_rpm = to_int(name, "max_motor_rpm", need(name, p, "max_motor_rpm"));
  if (c.max_motor_rpm < 1 || c.max_motor_rpm > mks::kMaxRpm) {
    throw std::invalid_argument("joint '" + name + "': max_motor_rpm must be 1..3000");
  }
  c.acceleration = to_int(name, "acceleration", need(name, p, "acceleration"));
  if (c.acceleration < 0 || c.acceleration > 255) {
    throw std::invalid_argument("joint '" + name + "': acceleration must be 0..255");
  }
  return c;
}

}  // namespace canopyag_hardware
