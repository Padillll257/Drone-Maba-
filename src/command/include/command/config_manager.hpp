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
    int arm_throttle;
    int mode_guided;
  };

  struct MappingConfig {
    JoyMapping joy_mappings;
    ButtonMapping button_mappings;
  };

  explicit ConfigManager(rclcpp::Node &node);

  void DeclareAllParameters();

  MappingConfig LoadMappingConfig();

 private:
  rclcpp::Node &node_;
};