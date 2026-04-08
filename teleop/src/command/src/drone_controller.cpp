#include "command/drone_controller.hpp"

DroneController::DroneController(rclcpp::Node& node)
    : node_(node),
      takeoff_state_(TakeOffState::kIdle),
      waiting_for_response_(false),
      takeoff_altitude_(-1.0f) {
  auto qos_best_effort = rclcpp::QoS(1).best_effort();

  state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10, [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
        UpdateCurrentState(msg);
      });
  pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", qos_best_effort,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        pose_ = *msg;
      });

  cmd_vel_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 10);

  set_mode_client_ =
      node_.create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
  arming_client_ =
      node_.create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
  takeoff_client_ =
      node_.create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

  timer_ = node_.create_wall_timer(std::chrono::milliseconds(100),
                                   [this]() { ProceedTakeoffSequence(); });
}

void DroneController::StartTakeoffSequence(float takeoff_altitude) {
  if (takeoff_altitude <= 0.0f) {
    RCLCPP_ERROR(node_.get_logger(),
                 "Invalid takeoff altitude %.2f, must be greater than 0",
                 takeoff_altitude);
    return;
  }

  takeoff_state_ = TakeOffState::kSettingGuided;
  takeoff_altitude_ = takeoff_altitude;
}

void DroneController::SendVelocityCommand(geometry_msgs::msg::Twist cmd_vel) {
  if (takeoff_state_ != TakeOffState::kHovering &&
      takeoff_state_ != TakeOffState::kIdle) {
    RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "Cannot send velocity command, takeoff sequence in progress");
    return;
  }

  auto cmd_vel_stamped = geometry_msgs::msg::TwistStamped();
  cmd_vel_stamped.header.frame_id = "base_link";
  cmd_vel_stamped.header.stamp = node_.now();
  cmd_vel_stamped.twist = cmd_vel;
  cmd_vel_pub_->publish(cmd_vel_stamped);
}

void DroneController::UpdateCurrentState(
    const mavros_msgs::msg::State::ConstSharedPtr msg) {
  current_state_ = *msg;

  if (takeoff_state_ == TakeOffState::kTakingOff) {
    RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "Drone is climbing, waiting to reach takeoff altitude. current "
        "altitude: %.2f, target altitude: %.2f",
        pose_.pose.position.z, takeoff_altitude_);

    if (pose_.pose.position.z >=
        takeoff_altitude_ - kTakeoffAltitudeTolerance) {
      RCLCPP_INFO(node_.get_logger(),
                  "Drone reached takeoff altitude, now hovering");
      takeoff_state_ = TakeOffState::kHovering;
      waiting_for_response_ = false;
    }
  }

  if (current_state_.mode != "GUIDED" && takeoff_state_ > TakeOffState::kIdle) {
    RCLCPP_ERROR(node_.get_logger(),
                 "Drone mode changed to %s, resetting takeoff sequence",
                 current_state_.mode.c_str());
    takeoff_state_ = TakeOffState::kIdle;
    waiting_for_response_ = false;
  }

  if (current_state_.mode == "GUIDED" &&
      takeoff_state_ == TakeOffState::kSettingGuided) {
    RCLCPP_INFO(node_.get_logger(), "Drone mode changed to %s, arming drone",
                current_state_.mode.c_str());
    takeoff_state_ = TakeOffState::kArming;
    waiting_for_response_ = false;
  }

  if (!current_state_.armed && takeoff_state_ > TakeOffState::kArming) {
    RCLCPP_ERROR(node_.get_logger(),
                 "Drone disarmed, resetting takeoff sequence");
    takeoff_state_ = TakeOffState::kIdle;
    waiting_for_response_ = false;
  }

  if (current_state_.armed && takeoff_state_ == TakeOffState::kArming) {
    takeoff_state_ = TakeOffState::kTakingOff;
    waiting_for_response_ = false;
  }
}

void DroneController::ProceedTakeoffSequence() {
  if (waiting_for_response_) {
    RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                         "Waiting for drone response...");
    return;
  }

  switch (takeoff_state_) {
    case TakeOffState::kIdle:
      break;

    case TakeOffState::kSettingGuided: {
      RCLCPP_INFO(node_.get_logger(), "Setting mode to GUIDED");

      if (!set_mode_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                             "SetMode service not available, waiting...");
        return;
      }

      auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      request->custom_mode = "GUIDED";
      set_mode_client_->async_send_request(request);

      waiting_for_response_ = true;
      break;
    }

    case TakeOffState::kArming: {
      RCLCPP_INFO(node_.get_logger(), "Arming the drone");

      if (!arming_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                             "Arming service not available, waiting...");
        return;
      }

      auto arm_request =
          std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      arm_request->value = true;
      arming_client_->async_send_request(arm_request);

      waiting_for_response_ = true;
      break;
    }

    case TakeOffState::kTakingOff: {
      RCLCPP_INFO(node_.get_logger(), "Taking off");

      if (!takeoff_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                             "Takeoff service not available, waiting...");
        return;
      }

      auto takeoff_request =
          std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
      takeoff_request->altitude =
          takeoff_altitude_;  // Takeoff to specified altitude
      takeoff_client_->async_send_request(
          takeoff_request,
          [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture
                     future) {
            auto response = future.get();
            if (response->success) {
              RCLCPP_INFO(node_.get_logger(), "Takeoff command accepted");
            } else {
              RCLCPP_ERROR(node_.get_logger(), "Takeoff command failed");
              takeoff_state_ = TakeOffState::kIdle;
              waiting_for_response_ = false;
            }
          });

      waiting_for_response_ = true;
      break;
    }

    case TakeOffState::kHovering: {
      RCLCPP_INFO_ONCE(node_.get_logger(), "Drone is hovering");
      break;
    }

    default: {
      RCLCPP_ERROR(node_.get_logger(),
                   "Unknown state %d, setting drone state back to IDLE",
                   static_cast<int>(takeoff_state_));
      takeoff_state_ = TakeOffState::kIdle;
      break;
    }
  }
}