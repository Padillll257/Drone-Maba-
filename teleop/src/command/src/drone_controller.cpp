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
    RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                         "Drone is climbing to target altitude: %.2f, current "
                         "altitude: %.2f",
                         takeoff_altitude_, pose_.pose.position.z);

    if (pose_.pose.position.z >=
        takeoff_altitude_ - kTakeoffAltitudeTolerance) {
      RCLCPP_INFO(node_.get_logger(),
                  "Drone reached takeoff altitude, now hovering");
      takeoff_state_ = TakeOffState::kHovering;
      waiting_for_response_ = false;
    }
  }

  if (current_state_.mode != "GUIDED" &&
      static_cast<int>(takeoff_state_) >
          static_cast<int>(TakeOffState::kIdle)) {
    RCLCPP_ERROR(node_.get_logger(),
                 "Drone mode changed to %s, resetting takeoff sequence",
                 current_state_.mode.c_str());
    takeoff_state_ = TakeOffState::kIdle;
    waiting_for_response_ = false;
  }

  if (current_state_.mode == "GUIDED" &&
      takeoff_state_ == TakeOffState::kSettingGuided) {
    RCLCPP_INFO(node_.get_logger(), "Drone mode changed to %s",
                current_state_.mode.c_str());
    takeoff_state_ = TakeOffState::kArming;
    waiting_for_response_ = false;
  }

  if (!current_state_.armed && static_cast<int>(takeoff_state_) >
                                   static_cast<int>(TakeOffState::kArming)) {
    RCLCPP_ERROR(node_.get_logger(),
                 "Drone disarmed, resetting takeoff sequence");
    takeoff_state_ = TakeOffState::kIdle;
    waiting_for_response_ = false;
  }

  if (current_state_.armed && takeoff_state_ == TakeOffState::kArming) {
    RCLCPP_INFO(node_.get_logger(), "Drone armed");
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

      SendServiceRequest(set_mode_client_, "SetMode", []() {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "GUIDED";
        return request;
      });
      break;
    }

    case TakeOffState::kArming: {
      RCLCPP_INFO(node_.get_logger(), "Arming the drone");

      SendServiceRequest(arming_client_, "Arming", []() {
        auto request =
            std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        return request;
      });
      break;
    }

    case TakeOffState::kTakingOff: {
      RCLCPP_INFO(node_.get_logger(), "Taking off");

      SendServiceRequest(takeoff_client_, "Takeoff", [this]() {
        auto request =
            std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = takeoff_altitude_;
        return request;
      });
      break;
    }

    case TakeOffState::kHovering:
      break;

    default: {
      RCLCPP_ERROR(node_.get_logger(),
                   "Unknown state %d, setting drone state back to IDLE",
                   static_cast<int>(takeoff_state_));
      takeoff_state_ = TakeOffState::kIdle;
      break;
    }
  }
}

bool DroneController::Land() {
  if (!set_mode_client_->service_is_ready()) {
    RCLCPP_ERROR(node_.get_logger(), "SetMode service not ready!");
    return false;
  }

  auto LandCallback =
      [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        if (!future.get()->mode_sent) {
          RCLCPP_INFO(node_.get_logger(), "Failed to send land request!");
        }
        RCLCPP_INFO(node_.get_logger(), "Land request has been sent");
      };

  RCLCPP_INFO(node_.get_logger(), "Sending land request...");
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "LAND";
  set_mode_client_->async_send_request(request, LandCallback);
  return true;
}