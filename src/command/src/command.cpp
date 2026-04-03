#include <chrono>
#include <command/config_manager.hpp>
#include <functional>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class CommandNode : public rclcpp::Node {
 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<ConfigManager> config_manager_;
  ConfigManager::MappingConfig config_;

  static constexpr float MAX_LINEAR_SPEED = 1.0f;
  static constexpr float MAX_ANGULAR_SPEED = 1.0f;

 public:
  CommandNode() : Node("command") {
    config_manager_ = std::make_shared<ConfigManager>(*this);
    config_manager_->DeclareAllParameters();
    config_ = config_manager_->LoadMappingConfig();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        [this](const sensor_msgs::msg::Joy &msg) { JoyCallback(msg); });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);
  }

 private:
  float axisOrZero(const std::vector<float> &axes, int index) {
    if (index >= 0 && index < static_cast<int>(axes.size())) {
      return axes[index];
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Axis index %d is out of range for Joy axes size %zu",
                         index, axes.size());
    return 0.0f;
  }

  void JoyCallback(const sensor_msgs::msg::Joy &msg) {
    auto out = geometry_msgs::msg::TwistStamped();
    out.header.frame_id = "base_link";
    out.header.stamp = this->now();

    out.twist.linear.x =
        axisOrZero(msg.axes, config_.joy_mappings.forward) * MAX_LINEAR_SPEED;
    out.twist.linear.y =
        axisOrZero(msg.axes, config_.joy_mappings.side) * MAX_LINEAR_SPEED;
    out.twist.linear.z =
        axisOrZero(msg.axes, config_.joy_mappings.altitude) * MAX_LINEAR_SPEED;

    out.twist.angular.x = 0.0f;
    out.twist.angular.y = 0.0f;
    out.twist.angular.z =
        axisOrZero(msg.axes, config_.joy_mappings.yaw) * MAX_ANGULAR_SPEED;

    cmd_vel_pub_->publish(out);
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
