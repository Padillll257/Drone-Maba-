#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

class DroneController {
 public:
  static constexpr float kTakeoffAltitudeTolerance = 0.2f;

  enum class TakeOffState {
    kIdle,
    kSettingGuided,
    kArming,
    kTakingOff,
    kHovering,
  };

  explicit DroneController(rclcpp::Node& node);

  void StartTakeoffSequence(float takeoff_altitude);
  void SendVelocityCommand(geometry_msgs::msg::Twist cmd_vel);

 private:
  rclcpp::Node& node_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped pose_;

  TakeOffState takeoff_state_;
  bool waiting_for_response_;
  float takeoff_altitude_;

  void ProceedTakeoffSequence();
  void UpdateCurrentState(const mavros_msgs::msg::State::ConstSharedPtr msg);
};