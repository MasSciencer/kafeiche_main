cmake_minimum_required(VERSION 3.8)
project(kafeiche_description)

# Find the required packages
find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)

# Install the config, launch, meshes, and urdf directories
install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Ament package macros
ament_package()
