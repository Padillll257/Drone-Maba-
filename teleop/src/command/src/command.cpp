#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "command/config_manager.hpp"

class CommandNode : public rclcpp::Node {
 public:
  CommandNode() : Node("command"),
                  prev_takeoff_button_(false),
                  takeoff_in_progress_(false) {
    config_manager_ = std::make_shared<ConfigManager>(*this);
    config_manager_->DeclareAllParameters();
    config_ = config_manager_->LoadMappingConfig();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
          JoyCallback(msg);
        });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);

    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode");
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
        "/mavros/cmd/takeoff");
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  std::shared_ptr<ConfigManager> config_manager_;
  ConfigManager::MappingConfig config_;

  bool prev_takeoff_button_;
  bool takeoff_in_progress_;

  static constexpr float kMaxLinearSpeed = 1.0f;
  static constexpr float kMaxAngularSpeed = 1.0f;

  float AxisOrZero(const std::vector<float> &axes, int index) {
    if (index >= 0 && index < static_cast<int>(axes.size())) {
      return axes[index];
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Axis index %d is out of range for Joy axes size %zu",
                         index, axes.size());
    return 0.0f;
  }

  bool ButtonPressed(const std::vector<int> &buttons, int index) {
    if (index >= 0 && index < static_cast<int>(buttons.size())) {
      return buttons[index] != 0;
    }
    return false;
  }

  void ExecuteTakeoff() {
    if (takeoff_in_progress_) {
      RCLCPP_WARN(this->get_logger(), "Takeoff already in progress, ignoring.");
      return;
    }
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "SetMode service not available.");
      return;
    }
    takeoff_in_progress_ = true;
    RCLCPP_INFO(this->get_logger(), "Takeoff button pressed: setting GUIDED mode.");

    auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    mode_req->custom_mode = "GUIDED";
    set_mode_client_->async_send_request(
        mode_req,
        [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
          auto result = future.get();
          if (!result->mode_sent) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set GUIDED mode.");
            takeoff_in_progress_ = false;
            return;
          }
          RCLCPP_INFO(this->get_logger(), "GUIDED mode set. Arming...");
          SendArmRequest();
        });
  }

  void SendArmRequest() {
    if (!arming_client_->service_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "Arming service not available.");
      takeoff_in_progress_ = false;
      return;
    }
    auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req->value = true;
    arming_client_->async_send_request(
        arm_req,
        [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
          auto result = future.get();
          if (!result->success) {
            RCLCPP_ERROR(this->get_logger(), "Arming failed.");
            takeoff_in_progress_ = false;
            return;
          }
          RCLCPP_INFO(this->get_logger(), "Armed. Sending takeoff command...");
          SendTakeoffRequest();
        });
  }

  void SendTakeoffRequest() {
    if (!takeoff_client_->service_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "Takeoff service not available.");
      takeoff_in_progress_ = false;
      return;
    }
    auto tol_req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    tol_req->altitude = config_.takeoff.altitude;
    // min_pitch, yaw, latitude, and longitude are set to 0 to let the
    // flight controller use its own defaults; MAVROS ignores lat/lon
    // when they are both 0 and uses the current GPS position instead.
    tol_req->min_pitch = 0.0f;
    tol_req->yaw = 0.0f;
    tol_req->latitude = 0.0f;
    tol_req->longitude = 0.0f;
    takeoff_client_->async_send_request(
        tol_req,
        [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
          auto result = future.get();
          if (!result->success) {
            RCLCPP_ERROR(this->get_logger(), "Takeoff command failed.");
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "Takeoff command accepted. Target altitude: %.1f m",
                        config_.takeoff.altitude);
          }
          takeoff_in_progress_ = false;
        });
  }

  void JoyCallback(sensor_msgs::msg::Joy::ConstSharedPtr msg) {
    auto out = geometry_msgs::msg::TwistStamped();
    out.header.frame_id = "base_link";
    out.header.stamp = this->now();

    out.twist.linear.x =
        AxisOrZero(msg->axes, config_.joy_mappings.forward) * kMaxLinearSpeed;
    out.twist.linear.y =
        AxisOrZero(msg->axes, config_.joy_mappings.side) * kMaxLinearSpeed;
    out.twist.linear.z =
        AxisOrZero(msg->axes, config_.joy_mappings.altitude) * kMaxLinearSpeed;

    out.twist.angular.x = 0.0f;
    out.twist.angular.y = 0.0f;
    out.twist.angular.z =
        AxisOrZero(msg->axes, config_.joy_mappings.yaw) * kMaxAngularSpeed;

    cmd_vel_pub_->publish(out);

    bool takeoff_button =
        ButtonPressed(msg->buttons, config_.button_mappings.take_off);
    if (takeoff_button && !prev_takeoff_button_) {
      ExecuteTakeoff();
    }
    prev_takeoff_button_ = takeoff_button;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CommandNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("command"), "Fatal error: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
