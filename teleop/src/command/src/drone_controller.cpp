#include "command/drone_controller.hpp"

DroneController::DroneController(rclcpp::Node &node) : node_(node) {
  set_mode_client_ =
      node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  arming_client_ =
      node_.create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  takeoff_client_ =
      node_.create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(kTimerUpdateIntervalMs),
      [this]() { TimerUpdate(); });
}

void DroneController::Takeoff(float altitude) {
  if (state_ != State::kIdle) {
    RCLCPP_WARN(node_.get_logger(), "Takeoff already in progress, ignoring.");
    return;
  }
  pending_altitude_ = altitude;
  state_ = State::kSettingGuidedMode;
  RCLCPP_INFO(node_.get_logger(), "Takeoff requested: setting GUIDED mode.");
}

void DroneController::TimerUpdate() {
  if (request_pending_) {
    return;
  }

  switch (state_) {
    case State::kIdle:
      break;

    case State::kSettingGuidedMode: {
      if (!set_mode_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                             "Waiting for SetMode service...");
        break;
      }
      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = "GUIDED";
      request_pending_ = true;
      set_mode_client_->async_send_request(
          req,
          [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            request_pending_ = false;
            if (!future.get()->mode_sent) {
              RCLCPP_ERROR(node_.get_logger(), "Failed to set GUIDED mode.");
              state_ = State::kIdle;
              return;
            }
            RCLCPP_INFO(node_.get_logger(), "GUIDED mode set. Arming...");
            state_ = State::kArming;
          });
      break;
    }

    case State::kArming: {
      if (!arming_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                             "Waiting for Arming service...");
        break;
      }
      auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      req->value = true;
      request_pending_ = true;
      arming_client_->async_send_request(
          req,
          [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            request_pending_ = false;
            if (!future.get()->success) {
              RCLCPP_ERROR(node_.get_logger(), "Arming failed.");
              state_ = State::kIdle;
              return;
            }
            RCLCPP_INFO(node_.get_logger(), "Armed. Sending takeoff command...");
            state_ = State::kTakingOff;
          });
      break;
    }

    case State::kTakingOff: {
      if (!takeoff_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                             "Waiting for Takeoff service...");
        break;
      }
      auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
      req->altitude = pending_altitude_;
      // min_pitch, yaw, latitude, and longitude are set to 0 to let the
      // flight controller use its own defaults. With ArduPilot via MAVROS,
      // lat/lon values of 0 are treated as "use current GPS position".
      req->min_pitch = 0.0f;
      req->yaw = 0.0f;
      req->latitude = 0.0f;
      req->longitude = 0.0f;
      request_pending_ = true;
      takeoff_client_->async_send_request(
          req,
          [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
            request_pending_ = false;
            if (!future.get()->success) {
              RCLCPP_ERROR(node_.get_logger(), "Takeoff command failed.");
            } else {
              RCLCPP_INFO(node_.get_logger(),
                          "Takeoff command accepted. Target altitude: %.1f m",
                          pending_altitude_);
            }
            state_ = State::kIdle;
          });
      break;
    }
  }
}
