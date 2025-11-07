cmake_minimum_required(VERSION 3.16)
project(sphero_rvr_controllers)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Build sensor broadcaster controllers
add_library(
  ${PROJECT_NAME}
  SHARED
  src/ambient_light_sensor_broadcaster.cpp
  src/color_sensor_broadcaster.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface sphero_rvr_controllers.xml)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install config and launch files
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
)

ament_package()
