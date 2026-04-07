#pragma once

#include <rclcpp/rclcpp.hpp>

class ConfigManager {
 public:
  struct JoyMapping {
    int altitude = 0;
    int yaw = 0;
    int forward = 0;
    int side = 0;
  };

  struct ButtonMapping {
    int arm_throttle = 0;
    int mode_guided = 0;
    int take_off = 0;
  };

  struct TakeoffConfig {
    float altitude = 0.0f;
  };

  struct MappingConfig {
    JoyMapping joy_mappings;
    ButtonMapping button_mappings;
    TakeoffConfig takeoff;
  };

  explicit ConfigManager(rclcpp::Node &node);

  void DeclareAllParameters();

  MappingConfig LoadMappingConfig();

 private:
  rclcpp::Node &node_;
};