#include <chrono>
#include <command/config_manager.hpp>
#include <functional>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "command/drone_controller.hpp"

class CommandNode : public rclcpp::Node {
 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::shared_ptr<ConfigManager> config_manager_;
  ConfigManager::Config config_;
  DroneController drone_controller_;

  static constexpr float MAX_LINEAR_SPEED = 1.0f;
  static constexpr float MAX_ANGULAR_SPEED = 1.0f;

 public:
  CommandNode() : Node("command"), drone_controller_(*this) {
    config_manager_ = std::make_shared<ConfigManager>(*this);
    config_manager_->DeclareAllParameters();
    config_ = config_manager_->LoadConfig();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
          JoyCallback(msg);
        });
  }

 private:
  template <typename T>
  T GetOrZero(const std::vector<T> &values, int index) {
    if (index >= 0 && index < static_cast<int>(values.size())) {
      return values[index];
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Values index %d is out of range for values size %zu",
                         index, values.size());
    return static_cast<T>(0);
  }

  void JoyCallback(sensor_msgs::msg::Joy::ConstSharedPtr msg) {
    auto twist = geometry_msgs::msg::Twist();

    twist.linear.x =
        GetOrZero(msg->axes, config_.joy_mappings.forward) * MAX_LINEAR_SPEED;
    twist.linear.y =
        GetOrZero(msg->axes, config_.joy_mappings.side) * MAX_LINEAR_SPEED;
    twist.linear.z =
        GetOrZero(msg->axes, config_.joy_mappings.altitude) * MAX_LINEAR_SPEED;

    twist.angular.x = 0.0f;
    twist.angular.y = 0.0f;
    twist.angular.z =
        GetOrZero(msg->axes, config_.joy_mappings.yaw) * MAX_ANGULAR_SPEED;

    drone_controller_.SendVelocityCommand(twist);

    if (GetOrZero(msg->buttons, config_.button_mappings.takeoff)) {
      RCLCPP_INFO(this->get_logger(), "Takeoff button pressed");
      drone_controller_.StartTakeoffSequence(config_.takeoff_altitude);
    }
    if (GetOrZero(msg->buttons, config_.button_mappings.land)) {
      RCLCPP_INFO(this->get_logger(), "Land button pressed");
      drone_controller_.Land();
    }
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
