cmake_minimum_required(VERSION 3.16)
project(sphero_rvr_control)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sphero_rvr_msgs REQUIRED)
find_package(sphero_rvr_driver_cpp REQUIRED)

# Hardware interface library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/sphero_rvr_hardware_interface.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sphero_rvr_msgs
  sphero_rvr_driver_cpp
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface sphero_rvr_control.xml)

# Install hardware interface library
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(
  DIRECTORY
    launch
    config
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
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sphero_rvr_msgs
  sphero_rvr_driver_cpp
)

ament_package()
