#include "command/drone_controller.hpp"

DroneController::DroneController(rclcpp::Node &node) : node_(node) {
  set_mode_client_ =
      node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  arming_client_ =
      node_.create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  takeoff_client_ =
      node_.create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
}

void DroneController::Takeoff(float altitude) {
  if (takeoff_in_progress_) {
    RCLCPP_WARN(node_.get_logger(), "Takeoff already in progress, ignoring.");
    return;
  }
  if (!set_mode_client_->service_is_ready()) {
    RCLCPP_ERROR(node_.get_logger(), "SetMode service not available.");
    return;
  }
  takeoff_in_progress_ = true;
  RCLCPP_INFO(node_.get_logger(),
              "Takeoff requested: setting GUIDED mode.");
  SetGuidedMode(altitude);
}

void DroneController::SetGuidedMode(float altitude) {
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = "GUIDED";
  set_mode_client_->async_send_request(
      req,
      [this, altitude](
          rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        auto result = future.get();
        if (!result->mode_sent) {
          RCLCPP_ERROR(node_.get_logger(), "Failed to set GUIDED mode.");
          takeoff_in_progress_ = false;
          return;
        }
        RCLCPP_INFO(node_.get_logger(), "GUIDED mode set. Arming...");
        ArmVehicle(altitude);
      });
}

void DroneController::ArmVehicle(float altitude) {
  if (!arming_client_->service_is_ready()) {
    RCLCPP_ERROR(node_.get_logger(), "Arming service not available.");
    takeoff_in_progress_ = false;
    return;
  }
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = true;
  arming_client_->async_send_request(
      req,
      [this, altitude](
          rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        auto result = future.get();
        if (!result->success) {
          RCLCPP_ERROR(node_.get_logger(), "Arming failed.");
          takeoff_in_progress_ = false;
          return;
        }
        RCLCPP_INFO(node_.get_logger(),
                    "Armed. Sending takeoff command...");
        SendTakeoffRequest(altitude);
      });
}

void DroneController::SendTakeoffRequest(float altitude) {
  if (!takeoff_client_->service_is_ready()) {
    RCLCPP_ERROR(node_.get_logger(), "Takeoff service not available.");
    takeoff_in_progress_ = false;
    return;
  }
  auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  req->altitude = altitude;
  // min_pitch, yaw, latitude, and longitude are set to 0 to let the
  // flight controller use its own defaults. With ArduPilot via MAVROS,
  // lat/lon values of 0 are treated as "use current GPS position".
  req->min_pitch = 0.0f;
  req->yaw = 0.0f;
  req->latitude = 0.0f;
  req->longitude = 0.0f;
  takeoff_client_->async_send_request(
      req,
      [this, altitude](
          rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
        auto result = future.get();
        if (!result->success) {
          RCLCPP_ERROR(node_.get_logger(), "Takeoff command failed.");
        } else {
          RCLCPP_INFO(node_.get_logger(),
                      "Takeoff command accepted. Target altitude: %.1f m",
                      altitude);
        }
        takeoff_in_progress_ = false;
      });
}
