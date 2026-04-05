cmake_minimum_required(VERSION 3.8)
project(autonomous)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED)    
find_package(mavros_msgs REQUIRED)    
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(auto_mission src/autonomous.cpp)  

ament_target_dependencies(auto_mission
  rclcpp
  mavros_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  auto_mission
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()