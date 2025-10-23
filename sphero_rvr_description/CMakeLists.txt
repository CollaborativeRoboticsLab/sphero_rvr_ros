cmake_minimum_required(VERSION 3.16)
project(sphero_rvr_description)

find_package(ament_cmake REQUIRED)

# Install launch files, URDFs, and meshes
install(
	DIRECTORY
		launch
		urdf
		meshes
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
