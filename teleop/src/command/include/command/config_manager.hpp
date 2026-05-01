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
    int land;
  };

  struct Config {
    JoyMapping joy_mappings;
    ButtonMapping button_mappings;

    double takeoff_altitude;
    double max_linear_speed;
    double max_angular_speed;
  };

  explicit ConfigManager(rclcpp::Node &node);

  void DeclareAllParameters();

  Config LoadConfig();

 private:
  rclcpp::Node &node_;
};