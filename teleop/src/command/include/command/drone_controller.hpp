#pragma once

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

// Handles low-level MAVROS flight operations (mode switching, arming, takeoff).
// CommandNode uses this class to issue high-level drone commands without
// dealing directly with MAVROS service calls.
//
// Internally a 100 ms wall timer drives an explicit state machine.
// Service response callbacks only update state; all transitions are
// initiated by the timer, making the control flow easy to follow and extend.
class DroneController {
 public:
  explicit DroneController(rclcpp::Node &node);

  // Arms the vehicle, switches to GUIDED mode, and commands a takeoff to the
  // given altitude (metres).  No-ops if a takeoff sequence is already running.
  void Takeoff(float altitude);

 private:
  enum class State {
    kIdle,
    kSettingGuidedMode,
    kArming,
    kTakingOff,
  };

  // Called every kTimerUpdateIntervalMs; drives the takeoff state machine.
  void TimerUpdate();

  static constexpr int kTimerUpdateIntervalMs = 100;

  rclcpp::Node &node_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_ = State::kIdle;
  bool request_pending_ = false;
  float pending_altitude_ = 0.0f;
};
