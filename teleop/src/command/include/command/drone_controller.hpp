#pragma once

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

// Handles low-level MAVROS flight operations (mode switching, arming, takeoff).
// CommandNode uses this class to issue high-level drone commands without
// dealing directly with MAVROS service calls.
class DroneController {
 public:
  explicit DroneController(rclcpp::Node &node);

  // Arms the vehicle, switches to GUIDED mode, and commands a takeoff to the
  // given altitude (metres).  No-ops if a takeoff is already in progress.
  void Takeoff(float altitude);

 private:
  void SetGuidedMode(float altitude);
  void ArmVehicle(float altitude);
  void SendTakeoffRequest(float altitude);

  rclcpp::Node &node_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

  bool takeoff_in_progress_ = false;
};
