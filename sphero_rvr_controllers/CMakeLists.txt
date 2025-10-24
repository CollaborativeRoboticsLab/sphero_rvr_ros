cmake_minimum_required(VERSION 3.16)
project(sphero_rvr_controllers)

find_package(ament_cmake REQUIRED)

# Install config and launch files
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
