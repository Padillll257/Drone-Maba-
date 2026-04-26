#include <command/config_manager.hpp>

ConfigManager::ConfigManager(rclcpp::Node &node) : node_(node) {}

void ConfigManager::DeclareAllParameters() {
  node_.declare_parameter("yaw", 0);
  node_.declare_parameter("altitude", 1);
  node_.declare_parameter("side", 2);
  node_.declare_parameter("forward", 3);

  node_.declare_parameter("takeoff", 1);
  node_.declare_parameter("land", 2);

  node_.declare_parameter("takeoff_altitude", 1.0);
}

ConfigManager::Config ConfigManager::LoadConfig() {
  Config config;

  config.joy_mappings.altitude = node_.get_parameter("altitude").as_int();
  config.joy_mappings.yaw = node_.get_parameter("yaw").as_int();
  config.joy_mappings.forward = node_.get_parameter("forward").as_int();
  config.joy_mappings.side = node_.get_parameter("side").as_int();

  config.button_mappings.takeoff = node_.get_parameter("takeoff").as_int();
  config.button_mappings.land = node_.get_parameter("land").as_int();

  config.takeoff_altitude = node_.get_parameter("takeoff_altitude").as_double();

  return config;
}