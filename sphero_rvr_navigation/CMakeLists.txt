cmake_minimum_required(VERSION 3.16)
project(sphero_rvr_navigation)

find_package(ament_cmake REQUIRED COMPONENTS)

# install config and launch files
install(
    DIRECTORY
        config
        launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
