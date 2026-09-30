// canopyag_hardware/CanopyagSystem: ros2_control SystemInterface for the MKS
// SERVO57D drivers on SocketCAN (ROS 2 Humble API).
//
// Joints come in two modes (config/hardware.yaml in canopyag_description):
//   can     - a real driver. The CAN thread streams its target as an absolute
//             encoder position (0xF5) and reads the encoder back (0x31).
//   virtual - no motor yet. State = command, like mock hardware, so the full
//             arm controller and MoveIt keep working while only some axes exist.
//
// Threads: read()/write() run in the controller manager's loop and never
// touch the socket; they swap targets and states with the CAN thread through
// a mutex-protected snapshot. The CAN thread runs at can_rate_hz on its own
// clock. The lifecycle callbacks use the bus only while the thread is stopped.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "canopyag_hardware/joint_config.hpp"
#include "canopyag_hardware/mks_bus.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace canopyag_hardware
{

class CanopyagSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CanopyagSystem)

  CanopyagSystem();
  ~CanopyagSystem() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;

  struct Joint
  {
    JointConfig cfg;

    // Controller-loop side: what the exported interfaces point at.
    double command = 0.0;
    double position = 0.0;
    double velocity = 0.0;

    // CAN-thread side (and lifecycle callbacks while the thread is stopped).
    int64_t zero_counts = 0;
    int64_t counts = 0;
    Clock::time_point counts_stamp{};
    Clock::time_point last_reply{};
    double filtered_vel = 0.0;
    double prev_target = 0.0;
    int64_t sent_counts = 0;        // last F5 target
    uint16_t sent_rpm = 0;
    Clock::time_point sent_stamp{};
    Clock::time_point still_since{};
    bool range_warned = false;
  };

  // Snapshot shared between read()/write() and the CAN thread.
  struct Shared
  {
    std::vector<double> target;     // joint units, NaN = keep the last one
    std::vector<double> position;
    std::vector<double> velocity;
  };

  // ---- lifecycle helpers (thread stopped) ----
  bool check_motors(std::string & err);
  void start_thread();
  void stop_thread();
  void stop_motors(bool emergency);

  // ---- CAN thread ----
  void can_loop();
  void stream_target(Joint & j, double target, double dt, Clock::time_point now);
  void collect_replies(Clock::time_point now);
  void raise_fault(const std::string & reason);
  void log_stats(Clock::time_point now);

  double expected_bus_load() const;
  std::vector<Joint *> can_joints();

  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::vector<Joint> joints_;

  // hardware params
  std::string can_interface_ = "can0";
  int can_bitrate_ = 500000;
  double can_rate_hz_ = 100.0;
  int reply_timeout_ms_ = 50;
  int max_missed_replies_ = 5;
  int startup_check_retries_ = 3;
  int min_rpm_ = 10;
  int deadband_counts_ = 4;

  MksBus bus_;
  bool zeroed_ = false;

  std::thread thread_;
  std::atomic<bool> running_{false};

  std::mutex mutex_;
  Shared shared_;
  std::atomic<bool> fault_{false};
  std::string fault_reason_;        // under mutex_
  bool fault_logged_ = false;

  // stats, CAN thread only
  Clock::time_point stats_since_{};
  MksBus::Stats stats_at_{};
  uint64_t reply_timeouts_ = 0;
  std::size_t stall_poll_next_ = 0;
};

}  // namespace canopyag_hardware
