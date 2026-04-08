#include <command/config_manager.hpp>

ConfigManager::ConfigManager(rclcpp::Node &node) : node_(node) {}

void ConfigManager::DeclareAllParameters() {
  node_.declare_parameter("yaw", 0);
  node_.declare_parameter("altitude", 1);
  node_.declare_parameter("side", 2);
  node_.declare_parameter("forward", 3);

  node_.declare_parameter("takeoff", 1);
}

ConfigManager::MappingConfig ConfigManager::LoadMappingConfig() {
  MappingConfig config;

  config.joy_mappings.altitude = node_.get_parameter("altitude").as_int();
  config.joy_mappings.yaw = node_.get_parameter("yaw").as_int();
  config.joy_mappings.forward = node_.get_parameter("forward").as_int();
  config.joy_mappings.side = node_.get_parameter("side").as_int();

  config.button_mappings.takeoff = node_.get_parameter("takeoff").as_int();

  return config;
}