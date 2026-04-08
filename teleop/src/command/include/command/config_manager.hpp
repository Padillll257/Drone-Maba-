#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

class ConfigManager {
 public:
  struct JoyMapping {
    int altitude;
    int yaw;
    int forward;
    int side;
  };

  struct ButtonMapping {
    int takeoff;
  };

  struct TakeoffConfig {
    float altitude;
  };

  struct MappingConfig {
    JoyMapping joy_mappings;
    ButtonMapping button_mappings;
  };

  explicit ConfigManager(rclcpp::Node &node);

  void DeclareAllParameters();

  MappingConfig LoadMappingConfig();
  TakeoffConfig LoadTakeoffConfig();

 private:
  rclcpp::Node &node_;
};