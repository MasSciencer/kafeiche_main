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
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# Find WiringPi library
find_library(WIRINGPI_LIBRARIES wiringPi)
if(NOT WIRINGPI_LIBRARIES)
    message(FATAL_ERROR "WiringPi library not found!")
endif()

# Add executable
add_executable(encoders src/encoders.cpp)
add_executable(motor src/motor.cpp)
add_executable(diffriver src/diffriver/diffriver_system.cpp)

target_link_libraries(encoders ${WIRINGPI_LIBRARIES})
target_link_libraries(motor ${WIRINGPI_LIBRARIES})

ament_target_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Install the executable
install(TARGETS encoders motor diffriver
  DESTINATION lib/${PROJECT_NAME})

## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
