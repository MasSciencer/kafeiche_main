cmake_minimum_required(VERSION 3.8)
project(kafeiche_drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  std_msgs
  rclcpp
  rclcpp_lifecycle
  control_toolbox
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Specify the required version of ros2_control
find_package(controller_manager 2.43.1)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 2.43.1 or higher is required. ")
endif()

# Find WiringPi library
find_library(WIRINGPI_LIBRARIES wiringPi)
if(NOT WIRINGPI_LIBRARIES)
    message(FATAL_ERROR "WiringPi library not found!")
endif()



# Add executable
add_executable(encoder hardware/encoder.cpp)
add_executable(motor hardware/motor.cpp)

target_link_libraries(encoder ${WIRINGPI_LIBRARIES})
target_link_libraries(motor ${WIRINGPI_LIBRARIES})

ament_target_dependencies(encoder std_msgs rclcpp)
ament_target_dependencies(motor std_msgs rclcpp)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

# Install the executable
# INSTALL

install(
  TARGETS encoder
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS motor
  DESTINATION lib/${PROJECT_NAME}
)

message(STATUS "CMAKE_INCLUDE_PATH: ${CMAKE_INCLUDE_PATH}")
message(STATUS "rclcpp_INCLUDE_DIRS: ${rclcpp_INCLUDE_DIRS}")

## EXPORTS
ament_package()
