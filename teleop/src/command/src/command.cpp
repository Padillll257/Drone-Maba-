#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "command/config_manager.hpp"
#include "command/drone_controller.hpp"

class CommandNode : public rclcpp::Node {
 public:
  CommandNode() : Node("command"),
                  prev_takeoff_button_(false) {
    config_manager_ = std::make_shared<ConfigManager>(*this);
    config_manager_->DeclareAllParameters();
    config_ = config_manager_->LoadMappingConfig();

    drone_controller_ = std::make_shared<DroneController>(*this);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
          JoyCallback(msg);
        });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<ConfigManager> config_manager_;
  std::shared_ptr<DroneController> drone_controller_;
  ConfigManager::MappingConfig config_;

  bool prev_takeoff_button_;

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
      drone_controller_->Takeoff(config_.takeoff.altitude);
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
