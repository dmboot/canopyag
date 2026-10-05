#include "canopyag_hardware/canopyag_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace canopyag_hardware
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Streaming F5 (see stream_target): an F5 is a move that ends at rest, so a
// target that is only one cycle ahead makes the driver brake every cycle -
// it lags and stutters. Instead each F5 aims at where the reference could stop
// (its braking distance at the driver's acceleration, times kLeadGain, at most
// kMaxLeadTime of travel ahead), at the reference speed plus a catch-up term
// that closes any lag within kCatchupTime.
constexpr double kLeadGain = 1.5;
constexpr double kMaxLeadTime = 0.25;   // s
constexpr double kCatchupTime = 0.25;   // s
constexpr double kRpmMargin = 1.05;
// Target velocity: low-pass over the controller's 200 Hz commands.
constexpr double kCmdVelAlpha = 0.3;
// Velocity state: first-order low-pass on the encoder difference.
constexpr double kVelAlpha = 0.3;
// A target that has not been reached while the motor stands still for this
// long gets its F5 sent again (covers a driver that ignored a retarget).
constexpr auto kResendAfter = std::chrono::milliseconds(250);
constexpr auto kStatsEvery = std::chrono::seconds(10);
// ~bits of one 8-byte standard frame incl. stuffing (README: bus budget).
constexpr double kBitsPerFrame = 130.0;

int param_int(const std::unordered_map<std::string, std::string> & p, const std::string & k, int def)
{
  auto it = p.find(k);
  if (it == p.end() || it->second.empty()) {
    return def;
  }
  try {
    return static_cast<int>(std::stod(it->second));
  } catch (const std::exception &) {
    throw std::invalid_argument("hardware param " + k + " = '" + it->second + "' is not a number");
  }
}

double to_ms(std::chrono::steady_clock::duration d)
{
  return std::chrono::duration<double, std::milli>(d).count();
}

}  // namespace

CanopyagSystem::CanopyagSystem()
: logger_(rclcpp::get_logger("CanopyagSystem")) {}

CanopyagSystem::~CanopyagSystem()
{
  if (running_) {
    stop_thread();
    stop_motors(fault_);
  }
  bus_.close();
}

// ===========================================================================
// init: parse and validate every param, no I/O
// ===========================================================================
CanopyagSystem::CallbackReturn CanopyagSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  logger_ = rclcpp::get_logger(info.name + ".CanopyagSystem");

  try {
    const auto & hp = info_.hardware_parameters;
    if (auto it = hp.find("can_interface"); it != hp.end() && !it->second.empty()) {
      can_interface_ = it->second;
    }
    can_bitrate_ = param_int(hp, "can_bitrate", can_bitrate_);
    can_rate_hz_ = param_int(hp, "can_rate_hz", 100);
    reply_timeout_ms_ = param_int(hp, "reply_timeout_ms", reply_timeout_ms_);
    max_missed_replies_ = param_int(hp, "max_missed_replies", max_missed_replies_);
    startup_check_retries_ = param_int(hp, "startup_check_retries", startup_check_retries_);
    min_rpm_ = param_int(hp, "min_rpm", min_rpm_);
    deadband_counts_ = param_int(hp, "position_deadband_counts", deadband_counts_);
    turn_timeout_ms_ = param_int(hp, "turn_timeout_ms", turn_timeout_ms_);
    if (can_rate_hz_ < 1 || can_rate_hz_ > 1000) {
      throw std::invalid_argument("can_rate_hz must be 1..1000");
    }
    if (reply_timeout_ms_ < 1 || max_missed_replies_ < 1 || startup_check_retries_ < 1) {
      throw std::invalid_argument(
              "reply_timeout_ms, max_missed_replies and startup_check_retries must be >= 1");
    }
    if (turn_timeout_ms_ < 1 || turn_timeout_ms_ * 2 > 1000.0 / can_rate_hz_) {
      throw std::invalid_argument("turn_timeout_ms must be >= 1 and well inside one CAN cycle");
    }
    if (min_rpm_ < 1 || min_rpm_ > mks::kMaxRpm) {
      throw std::invalid_argument("min_rpm must be 1..3000");
    }

    std::map<uint32_t, std::string> ids;
    for (const auto & ji : info_.joints) {
      if (ji.command_interfaces.size() != 1 ||
        ji.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::invalid_argument("joint '" + ji.name + "': needs exactly one 'position' command interface");
      }
      std::set<std::string> states;
      for (const auto & s : ji.state_interfaces) {
        states.insert(s.name);
      }
      if (states != std::set<std::string>{hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY})
      {
        throw std::invalid_argument("joint '" + ji.name + "': needs 'position' and 'velocity' state interfaces");
      }

      std::map<std::string, std::string> params(ji.parameters.begin(), ji.parameters.end());
      Joint j;
      j.cfg = parse_joint_config(
        ji.name, params, ji.command_interfaces[0].min, ji.command_interfaces[0].max);
      if (j.cfg.mode == JointConfig::Mode::kCan) {
        if (j.cfg.max_motor_rpm < min_rpm_) {
          throw std::invalid_argument("joint '" + ji.name + "': max_motor_rpm below min_rpm");
        }
        auto [it, fresh] = ids.emplace(j.cfg.can_id, ji.name);
        if (!fresh) {
          throw std::invalid_argument(
                  "CAN ID " + std::to_string(j.cfg.can_id) + " is used by both '" + it->second +
                  "' and '" + ji.name + "' in hardware.yaml");
        }
      }
      j.position = j.command = j.cfg.initial_position;
      joints_.push_back(j);
    }
  } catch (const std::invalid_argument & e) {
    RCLCPP_FATAL(logger_, "%s", e.what());
    return CallbackReturn::ERROR;
  }

  shared_.target.assign(joints_.size(), kNaN);
  shared_.target_vel.assign(joints_.size(), 0.0);
  shared_.position.assign(joints_.size(), 0.0);
  shared_.velocity.assign(joints_.size(), 0.0);

  for (const auto & j : joints_) {
    if (j.cfg.mode == JointConfig::Mode::kCan) {
      RCLCPP_INFO(
        logger_, "  %-11s CAN ID %u  %.6g motor rev/unit%s  max %d rpm (= %.4g units/s)  acc %d",
        j.cfg.name.c_str(), j.cfg.can_id, j.cfg.motor_revs_per_unit,
        j.cfg.reversed ? " reversed" : "", j.cfg.max_motor_rpm,
        j.cfg.rpm_to_joint_vel(j.cfg.max_motor_rpm), j.cfg.acceleration);
    } else {
      RCLCPP_INFO(logger_, "  %-11s VIRTUAL (no motor: state follows command)", j.cfg.name.c_str());
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<CanopyagSystem::Joint *> CanopyagSystem::can_joints()
{
  std::vector<Joint *> out;
  for (auto & j : joints_) {
    if (j.cfg.mode == JointConfig::Mode::kCan) {
      out.push_back(&j);
    }
  }
  return out;
}

double CanopyagSystem::expected_bus_load() const
{
  std::size_t n = 0;
  for (const auto & j : joints_) {
    n += j.cfg.mode == JointConfig::Mode::kCan;
  }
  // per cycle: F5 + reply and 0x31 + reply per motor, plus one 0x3E + reply
  const double frames_per_s = can_rate_hz_ * (4.0 * n + (n ? 2.0 : 0.0));
  return frames_per_s * kBitsPerFrame / std::max(can_bitrate_, 1);
}

// ===========================================================================
// configure: open the socket, every motor must answer, no stall latched
// ===========================================================================
CanopyagSystem::CallbackReturn CanopyagSystem::on_configure(const rclcpp_lifecycle::State &)
{
  fault_ = false;
  fault_logged_ = false;
  zeroed_ = false;
  const auto motors = can_joints();
  if (motors.empty()) {
    RCLCPP_WARN(logger_, "every joint is virtual: no CAN traffic at all");
    return CallbackReturn::SUCCESS;
  }

  std::string err;
  if (!bus_.open(can_interface_, err)) {
    RCLCPP_FATAL(logger_, "%s", err.c_str());
    return CallbackReturn::ERROR;
  }

  const double load = expected_bus_load();
  RCLCPP_INFO(
    logger_, "CAN %s at %d bit/s, %zu motor(s), %.0f Hz cycle: expected bus load %.0f%%",
    can_interface_.c_str(), can_bitrate_, motors.size(), can_rate_hz_, 100.0 * load);
  if (load > 0.5) {
    RCLCPP_WARN(
      logger_, "expected bus load %.0f%% is over 50%%: lower can_rate_hz", 100.0 * load);
  }

  if (!check_motors(err)) {
    RCLCPP_FATAL(logger_, "%s", err.c_str());
    fault_reason_ = err;
    stop_motors(true);
    bus_.close();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool CanopyagSystem::check_motors(std::string & err)
{
  for (Joint * j : can_joints()) {
    const uint32_t id = j->cfg.can_id;
    std::vector<MksBus::Reply> replies;
    for (int attempt = 0; attempt < startup_check_retries_ && replies.empty(); ++attempt) {
      bus_.discard(id, mks::kReadEncoder);
      if (!bus_.send(mks::read_encoder(id))) {
        err = "joint '" + j->cfg.name + "' (CAN ID " + std::to_string(id) + "): " + bus_.last_error();
        return false;
      }
      if (auto first = bus_.wait(id, mks::kReadEncoder, reply_timeout_ms_)) {
        // Keep listening: a second driver with the same CanID answers too
        // (lesson 3), and only an extra listen window catches it.
        bus_.pump(reply_timeout_ms_);
        replies.push_back(*first);
        auto more = bus_.take_all(id, mks::kReadEncoder);
        replies.insert(replies.end(), more.begin(), more.end());
      }
    }
    if (const auto e = bus_.take_bus_error(); !e.empty()) {
      RCLCPP_WARN(logger_, "CAN controller: %s", e.c_str());
    }
    if (replies.empty()) {
      err = "joint '" + j->cfg.name + "' (CAN ID " + std::to_string(id) +
        ") does not reply to 0x31 after " + std::to_string(startup_check_retries_) +
        " tries. Check power, the driver's CanID / CanRate menu and the wiring "
        "(python3 ~/nema_test/status.py)";
      if (bus_.bus_off()) {
        err += ". The adapter is BUS-OFF: check 60 ohm termination, then "
          "sudo ip link set " + can_interface_ + " down && sudo ip link set " + can_interface_ + " up";
      }
      return false;
    }
    if (replies.size() > 1) {
      err = "CAN ID " + std::to_string(id) + " (joint '" + j->cfg.name + "') answered " +
        std::to_string(replies.size()) + " times to one 0x31 request: two drivers share this "
        "CanID. Give each driver a unique CanID in its menu.";
      return false;
    }

    auto stall = bus_.request(mks::read_stall(id), reply_timeout_ms_);
    const auto tripped = stall ? mks::parse_status(stall->frame, mks::kReadStall) : std::nullopt;
    if (!tripped) {
      err = "joint '" + j->cfg.name + "' (CAN ID " + std::to_string(id) + ") does not reply to 0x3E";
      return false;
    }
    if (*tripped == 1) {
      err = "joint '" + j->cfg.name + "' (CAN ID " + std::to_string(id) + "): stall protection is "
        "LATCHED. Check the mechanics, then clear it by hand: cd ~/nema_test && python3 -c "
        "\"from mks_servo import *; b=CanBus(); MKSServo(b," + std::to_string(id) +
        ").release_protection()\"";
      return false;
    }
    RCLCPP_INFO(logger_, "  %s (CAN ID %u): replies, no stall", j->cfg.name.c_str(), id);
  }
  return true;
}

CanopyagSystem::CallbackReturn CanopyagSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_thread();
  bus_.close();
  zeroed_ = false;
  return CallbackReturn::SUCCESS;
}

// ===========================================================================
// activate: enable, zero at startup, command = state, start the CAN thread
// ===========================================================================
CanopyagSystem::CallbackReturn CanopyagSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (fault_) {
    RCLCPP_ERROR(logger_, "refusing to activate after a fault: %s", fault_reason_.c_str());
    return CallbackReturn::ERROR;
  }
  const auto now = Clock::now();

  for (Joint * j : can_joints()) {
    const uint32_t id = j->cfg.can_id;
    std::optional<uint8_t> ok;
    for (int a = 0; a < startup_check_retries_ && !ok; ++a) {
      auto r = bus_.request(mks::enable(id, true), reply_timeout_ms_);
      ok = r ? mks::parse_status(r->frame, mks::kEnable) : std::nullopt;
    }
    if (!ok || *ok != 1) {
      RCLCPP_FATAL(
        logger_, "joint '%s' (CAN ID %u): enable (0xF3) %s", j->cfg.name.c_str(), id,
        ok ? "rejected" : "not answered");
      stop_motors(true);
      return CallbackReturn::ERROR;
    }

    std::optional<int64_t> counts;
    for (int a = 0; a < startup_check_retries_ && !counts; ++a) {
      auto r = bus_.request(mks::read_encoder(id), reply_timeout_ms_);
      counts = r ? mks::parse_encoder(r->frame) : std::nullopt;
    }
    if (!counts) {
      RCLCPP_FATAL(logger_, "joint '%s' (CAN ID %u): no 0x31 reply", j->cfg.name.c_str(), id);
      stop_motors(true);
      return CallbackReturn::ERROR;
    }

    if (!zeroed_) {
      j->zero_counts = *counts;
    }
    j->counts = *counts;
    j->counts_stamp = j->last_reply = j->sent_stamp = j->still_since = now;
    j->position = j->cfg.counts_to_joint(*counts, j->zero_counts);
    j->velocity = j->filtered_vel = 0.0;
    j->command = j->prev_target = j->position;
    j->sent_counts = *counts;      // "already there": no F5 until the command moves
    j->sent_rpm = static_cast<uint16_t>(min_rpm_);

    RCLCPP_INFO(
      logger_, "  %s (CAN ID %u): encoder %ld counts = %.5f %s, F5 range %.4g .. %.4g",
      j->cfg.name.c_str(), id, static_cast<long>(*counts), j->position,
      std::isfinite(j->cfg.max_position - j->cfg.min_position) ? "(limited)" : "(continuous)",
      std::min(j->cfg.counts_to_joint(mks::kI24Min, j->zero_counts),
      j->cfg.counts_to_joint(mks::kI24Max, j->zero_counts)),
      std::max(j->cfg.counts_to_joint(mks::kI24Min, j->zero_counts),
      j->cfg.counts_to_joint(mks::kI24Max, j->zero_counts)));
  }

  for (auto & j : joints_) {
    if (j.cfg.mode == JointConfig::Mode::kVirtual) {
      if (!zeroed_) {
        j.position = j.cfg.initial_position;
      }
      j.command = j.position;
      j.velocity = 0.0;
    }
  }

  if (!zeroed_ && !can_joints().empty()) {
    RCLCPP_WARN(logger_, "==============================================================");
    RCLCPP_WARN(logger_, "ZERO AT STARTUP: each joint's CURRENT position is taken as its");
    RCLCPP_WARN(logger_, "initial_position (0.0 = URDF home). The robot must have been");
    RCLCPP_WARN(logger_, "physically at home when the drivers were powered on. After a");
    RCLCPP_WARN(logger_, "driver power cycle, move it home and restart this node.");
    RCLCPP_WARN(logger_, "==============================================================");
  }
  zeroed_ = true;

  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      shared_.target[i] = kNaN;
      shared_.target_vel[i] = 0.0;
      joints_[i].prev_command = kNoCommand;
      joints_[i].command_vel = 0.0;
      shared_.position[i] = joints_[i].position;
      shared_.velocity[i] = 0.0;
    }
  }
  start_thread();
  return CallbackReturn::SUCCESS;
}

CanopyagSystem::CallbackReturn CanopyagSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_thread();
  stop_motors(fault_);
  return CallbackReturn::SUCCESS;
}

CanopyagSystem::CallbackReturn CanopyagSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  const bool was_running = running_;
  stop_thread();
  if (was_running || fault_) {
    stop_motors(fault_);
  }
  bus_.close();
  return CallbackReturn::SUCCESS;
}

CanopyagSystem::CallbackReturn CanopyagSystem::on_error(const rclcpp_lifecycle::State &)
{
  stop_thread();
  stop_motors(true);
  RCLCPP_ERROR(
    logger_, "hardware in ERROR (%s). Every motor got F7. Fix the cause, put the robot at "
    "home and restart the launch (zero at startup).",
    fault_reason_.empty() ? "lifecycle error" : fault_reason_.c_str());
  bus_.close();
  // FAILURE -> FINALIZED: no way back to active without a restart, because a
  // re-configure would re-zero wherever the robot happens to be now.
  return CallbackReturn::FAILURE;
}

void CanopyagSystem::stop_motors(bool emergency)
{
  if (!bus_.is_open()) {
    return;
  }
  for (Joint * j : can_joints()) {
    // One at a time like everything else; a missing reply gets one retry.
    const mks::Frame f = emergency ?
      mks::estop(j->cfg.can_id) :
      // ramped stop that keeps holding torque
      mks::speed_mode(j->cfg.can_id, 0, static_cast<uint8_t>(j->cfg.acceleration), false);
    if (!turn(f)) {
      turn(f);
    }
  }
  bus_.pump(5);
  RCLCPP_WARN(
    logger_, "%s sent to every motor", emergency ? "EMERGENCY STOP (F7)" : "ramped stop (F6 0 rpm)");
}

void CanopyagSystem::start_thread()
{
  if (can_joints().empty() || running_) {
    return;
  }
  running_ = true;
  thread_ = std::thread(&CanopyagSystem::can_loop, this);
}

void CanopyagSystem::stop_thread()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

// ===========================================================================
// read / write: no socket I/O, only the snapshot
// ===========================================================================
std::vector<hardware_interface::StateInterface> CanopyagSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.cfg.name, hardware_interface::HW_IF_POSITION, &j.position);
    out.emplace_back(j.cfg.name, hardware_interface::HW_IF_VELOCITY, &j.velocity);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> CanopyagSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.cfg.name, hardware_interface::HW_IF_POSITION, &j.command);
  }
  return out;
}

hardware_interface::return_type CanopyagSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (fault_) {
    if (!fault_logged_) {
      std::lock_guard<std::mutex> lk(mutex_);
      RCLCPP_ERROR(logger_, "FAULT: %s", fault_reason_.c_str());
      fault_logged_ = true;
    }
    return hardware_interface::return_type::ERROR;
  }

  const double dt = period.seconds();
  std::lock_guard<std::mutex> lk(mutex_);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    Joint & j = joints_[i];
    if (j.cfg.mode == JointConfig::Mode::kCan) {
      j.position = shared_.position[i];
      j.velocity = shared_.velocity[i];
    } else if (std::isfinite(j.command)) {
      const double next = j.cfg.clamp(j.command);
      j.velocity = dt > 0.0 ? (next - j.position) / dt : 0.0;
      j.position = next;
    } else {
      j.velocity = 0.0;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanopyagSystem::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (fault_) {
    return hardware_interface::return_type::ERROR;
  }
  // Velocity of the command, here and not in the CAN thread: this runs on the
  // controller's own clock, so successive commands are exactly one period
  // apart. (Sampled from the 100 Hz CAN thread, one or three controller steps
  // can fall into a cycle and the speed jitters by +-50%.)
  const double dt = period.seconds();
  for (auto & j : joints_) {
    if (!std::isfinite(j.command)) {
      continue;
    }
    if (j.prev_command != kNoCommand && dt > 0.0) {
      const double v = (j.command - j.prev_command) / dt;
      j.command_vel += kCmdVelAlpha * (v - j.command_vel);
    }
    j.prev_command = j.command;
  }
  std::lock_guard<std::mutex> lk(mutex_);
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    shared_.target[i] = joints_[i].command;
    shared_.target_vel[i] = joints_[i].command_vel;
  }
  return hardware_interface::return_type::OK;
}

// ===========================================================================
// CAN thread
// ===========================================================================
void CanopyagSystem::raise_fault(const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (fault_) {
      return;
    }
    fault_reason_ = reason;
    fault_ = true;
  }
  RCLCPP_ERROR(logger_, "FAULT: %s -> F7 to every motor", reason.c_str());
  stop_motors(true);
  running_ = false;
}

void CanopyagSystem::can_loop()
{
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / can_rate_hz_));
  const auto watchdog = std::chrono::milliseconds(reply_timeout_ms_ * max_missed_replies_);
  auto motors = can_joints();
  std::vector<std::size_t> index;
  for (Joint * j : motors) {
    index.push_back(static_cast<std::size_t>(j - joints_.data()));
  }

  const auto t_start = Clock::now();
  for (Joint * j : motors) {
    j->last_reply = t_start;
  }
  stats_since_ = t_start;
  stats_at_ = bus_.stats();
  reply_timeouts_ = 0;

  std::vector<double> targets(joints_.size(), kNaN);
  std::vector<double> target_vels(joints_.size(), 0.0);
  while (running_) {
    const auto t0 = Clock::now();

    {
      std::lock_guard<std::mutex> lk(mutex_);
      targets = shared_.target;
      target_vels = shared_.target_vel;
    }

    // One request on the bus at a time: the MKS drivers produce bit errors
    // (error-warning, error-passive) when two of them answer at once, so
    // every request waits for its reply before the next one goes out.
    // Per motor: its F5 if the target moved, then its encoder; then one
    // motor's stall flag.
    for (std::size_t k = 0; k < motors.size(); ++k) {
      const double t = targets[index[k]];
      stream_target(
        *motors[k], std::isfinite(t) ? t : motors[k]->prev_target,
        std::isfinite(t) ? target_vels[index[k]] : 0.0, t0);
      // No discard: the buffer was emptied at the end of the last cycle,
      // and a reply that lands late is still fresh data, not stale.
      turn(mks::read_encoder(motors[k]->cfg.can_id));
    }
    Joint * poll = motors[stall_poll_next_++ % motors.size()];
    bus_.discard(poll->cfg.can_id, mks::kReadStall);
    turn(mks::read_stall(poll->cfg.can_id));

    // Listen for the rest of the cycle (late replies, F5 "done"), then act
    // on everything that came back.
    bus_.pump_until(t0 + period);
    const auto now = Clock::now();
    collect_replies(now);
    if (!running_) {
      break;
    }

    for (Joint * j : motors) {
      if (now - j->last_reply > watchdog) {
        raise_fault(
          "joint '" + j->cfg.name + "' (CAN ID " + std::to_string(j->cfg.can_id) +
          ") has not answered 0x31 for " + std::to_string(static_cast<int>(to_ms(now - j->last_reply))) +
          " ms (limit " + std::to_string(watchdog.count()) + " ms): power, wiring or CanID?");
        break;
      }
    }
    if (bus_.bus_off()) {
      raise_fault("CAN adapter is BUS-OFF: " + bus_.take_bus_error());
    } else if (const auto e = bus_.take_bus_error(); !e.empty()) {
      RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 2000, "CAN controller: %s", e.c_str());
    }
    if (!running_) {
      break;
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      for (std::size_t k = 0; k < motors.size(); ++k) {
        const Joint & j = *motors[k];
        shared_.position[index[k]] = j.cfg.counts_to_joint(j.counts, j.zero_counts);
        shared_.velocity[index[k]] = j.filtered_vel;
      }
    }
    log_stats(now);
  }
}

bool CanopyagSystem::turn(const mks::Frame & f)
{
  const std::size_t had = bus_.count(f.id, f.cmd());
  if (!bus_.send(f)) {
    return false;
  }
  if (bus_.await_reply(f.id, f.cmd(), had + 1, turn_timeout_ms_)) {
    return true;
  }
  ++turn_timeouts_;
  return false;
}

void CanopyagSystem::stream_target(
  Joint & j, double target, double target_vel, Clock::time_point now)
{
  const JointConfig & c = j.cfg;
  target = c.clamp(target);
  j.prev_target = target;

  // The reference can never be faster than the joint's speed cap; a step
  // command (forward_position_controller) would otherwise look like a huge
  // velocity for one cycle.
  const double vmax = c.rpm_to_joint_vel(c.max_motor_rpm);
  const double v = std::clamp(target_vel, -vmax, vmax);

  // Aim ahead by the driver's braking distance at this speed, so it never
  // decides to stop short while the reference keeps moving. Joint units.
  double lead = 0.0;
  if (c.acceleration > 0 && std::abs(v) > 0.0) {
    const double accel = c.rpm_to_joint_vel(20000.0 / (256 - c.acceleration));  // units/s^2
    lead = std::min(kLeadGain * v * v / (2.0 * accel), std::abs(v) * kMaxLeadTime);
  }
  const double aim = c.clamp(target + std::copysign(lead, v));

  const int64_t wanted = c.joint_to_counts(aim, j.zero_counts);
  const int32_t counts = mks::clamp_i24(wanted);
  if (counts != wanted && !j.range_warned) {
    RCLCPP_WARN(
      logger_, "joint '%s': target %.4f is outside the F5 range (i24 = +-512 motor revs); "
      "clamped to %.4f. See the README joint table for the usable range.",
      c.name.c_str(), target, c.counts_to_joint(counts, j.zero_counts));
    j.range_warned = true;
  }

  const int32_t ref_counts = mks::clamp_i24(c.joint_to_counts(target, j.zero_counts));
  const bool target_moved = std::llabs(counts - j.sent_counts) > deadband_counts_;
  const bool not_there = std::llabs(ref_counts - j.counts) > 4 * deadband_counts_;
  const bool stuck = not_there && now - j.still_since > kResendAfter && now - j.sent_stamp > kResendAfter;
  if (!target_moved && !stuck) {
    return;
  }

  // Speed cap: the reference speed, plus whatever closes the lag in
  // kCatchupTime (minus, when the motor is ahead of the reference).
  const double measured = c.counts_to_joint(j.counts, j.zero_counts);
  const double lag = target - measured;   // joint units, signed
  const double along = std::abs(v) > 1e-9 ? std::copysign(1.0, v) * lag : std::abs(lag);
  const double r = c.joint_vel_to_rpm(v) * kRpmMargin + c.joint_vel_to_rpm(along / kCatchupTime) *
    (along >= 0.0 ? 1.0 : -1.0);
  const uint16_t rpm = static_cast<uint16_t>(
    std::clamp<double>(std::ceil(r), min_rpm_, c.max_motor_rpm));

  turn(mks::abs_move(c.can_id, rpm, static_cast<uint8_t>(c.acceleration), counts));
  RCLCPP_DEBUG(
    logger_, "%s F5 -> %.5f units (ref %.5f, %+.4f units/s, lag %+.5f) = %d counts "
    "(%+.3f motor revs from zero) @ %u rpm%s",
    c.name.c_str(), aim, target, v, lag, counts,
    static_cast<double>(counts - j.zero_counts) / mks::kCountsPerRev, rpm,
    target_moved ? "" : " (resend: motor stopped short)");
  j.sent_counts = counts;
  j.sent_rpm = rpm;
  j.sent_stamp = now;
}

void CanopyagSystem::collect_replies(Clock::time_point now)
{
  for (Joint * j : can_joints()) {
    const uint32_t id = j->cfg.can_id;

    auto enc = bus_.take_all(id, mks::kReadEncoder);
    bool got = false;
    for (const auto & r : enc) {
      const auto counts = mks::parse_encoder(r.frame);
      if (!counts) {
        continue;
      }
      const double dt = std::chrono::duration<double>(r.stamp - j->counts_stamp).count();
      if (dt > 1e-4) {
        const double raw = j->cfg.sign() * static_cast<double>(*counts - j->counts) /
          j->cfg.counts_per_unit() / dt;
        j->filtered_vel += kVelAlpha * (raw - j->filtered_vel);
      }
      if (std::llabs(*counts - j->counts) > deadband_counts_) {
        j->still_since = r.stamp;
      }
      j->counts = *counts;
      j->counts_stamp = r.stamp;
      got = true;
    }
    if (got) {
      j->last_reply = now;
    } else {
      ++reply_timeouts_;
    }

    for (const auto & r : bus_.take_all(id, mks::kAbsMove)) {
      const auto st = mks::parse_status(r.frame, mks::kAbsMove);
      if (st && (*st == mks::kMoveFailed || *st == mks::kMoveLimit)) {
        raise_fault(
          "joint '" + j->cfg.name + "' (CAN ID " + std::to_string(id) + "): F5 " +
          (*st == mks::kMoveFailed ? "REJECTED (status 0): not enabled, wrong Mode (needs SR_vFOC), "
          "stall protection, or the driver refuses a new target mid-move" :
          "stopped by end limit (status 3)"));
        return;
      }
    }

    for (const auto & r : bus_.take_all(id, mks::kReadStall)) {
      const auto st = mks::parse_status(r.frame, mks::kReadStall);
      if (st && *st == 1) {
        raise_fault(
          "joint '" + j->cfg.name + "' (CAN ID " + std::to_string(id) + "): STALL PROTECTION "
          "tripped. Check the mechanics; it stays latched until cleared by hand (0x3D, see README)");
        return;
      }
    }
  }
}

void CanopyagSystem::log_stats(Clock::time_point now)
{
  if (now - stats_since_ < kStatsEvery) {
    return;
  }
  const auto & s = bus_.stats();
  const double secs = std::chrono::duration<double>(now - stats_since_).count();
  RCLCPP_DEBUG(
    logger_, "CAN %.0f tx/s, %.0f rx/s, %lu missed 0x31 replies, %lu late replies (> %d ms), "
    "%lu bad CRC, %lu error frames in the last %.0f s",
    (s.tx - stats_at_.tx) / secs, (s.rx - stats_at_.rx) / secs,
    static_cast<unsigned long>(reply_timeouts_), static_cast<unsigned long>(turn_timeouts_),
    turn_timeout_ms_,
    static_cast<unsigned long>(s.bad_crc - stats_at_.bad_crc),
    static_cast<unsigned long>(s.error_frames - stats_at_.error_frames), secs);
  for (Joint * j : can_joints()) {
    RCLCPP_DEBUG(
      logger_, "  %s: %ld counts = %+.3f motor revs from zero = %.5f joint units, %.4f units/s",
      j->cfg.name.c_str(), static_cast<long>(j->counts),
      static_cast<double>(j->counts - j->zero_counts) / mks::kCountsPerRev,
      j->cfg.counts_to_joint(j->counts, j->zero_counts), j->filtered_vel);
  }
  stats_since_ = now;
  stats_at_ = s;
  reply_timeouts_ = 0;
  turn_timeouts_ = 0;
}

}  // namespace canopyag_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(canopyag_hardware::CanopyagSystem, hardware_interface::SystemInterface)
