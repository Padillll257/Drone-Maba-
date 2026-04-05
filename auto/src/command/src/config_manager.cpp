#include <command/config_manager.hpp>

ConfigManager::ConfigManager(rclcpp::Node &node) : node_(node) {}

void ConfigManager::DeclareAllParameters() {
  node_.declare_parameter("yaw", 0);
  node_.declare_parameter("altitude", 1);
  node_.declare_parameter("side", 2);
  node_.declare_parameter("forward", 3);
  node_.declare_parameter("arm_throttle", 8);
  node_.declare_parameter("mode_guided", 9);
}

ConfigManager::MappingConfig ConfigManager::LoadMappingConfig() {
  MappingConfig config;

  config.joy_mappings.altitude = node_.get_parameter("altitude").as_int();
  config.joy_mappings.yaw = node_.get_parameter("yaw").as_int();
  config.joy_mappings.forward = node_.get_parameter("forward").as_int();
  config.joy_mappings.side = node_.get_parameter("side").as_int();

  config.button_mappings.mode_guided =
      node_.get_parameter("mode_guided").as_int();
  config.button_mappings.arm_throttle =
      node_.get_parameter("arm_throttle").as_int();

  return config;
}