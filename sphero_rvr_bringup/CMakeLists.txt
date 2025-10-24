cmake_minimum_required(VERSION 3.16)
project(sphero_rvr_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
