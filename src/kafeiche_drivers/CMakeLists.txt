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
add_executable(encoder src/encoder.cpp)
add_executable(motor src/motor.cpp)

#Here we implement cpp as library and next we we put it into plugin
add_library(
  kafeiche_drivers
  SHARED
  src/diffdriver_system.cpp
)

target_link_libraries(encoder ${WIRINGPI_LIBRARIES})
target_link_libraries(motor ${WIRINGPI_LIBRARIES})
target_link_libraries(kafeiche_drivers PUBLIC serial)

target_compile_features(kafeiche_drivers PUBLIC cxx_std_17)
target_include_directories(kafeiche_drivers PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/kafeiche_drivers>
)

#That is very importaint part which matches #includ and own libraries 
ament_target_dependencies(encoder std_msgs rclcpp) #this executable file depends only on 2 ROS2 packages
ament_target_dependencies(motor std_msgs rclcpp) #this executable file depends only on 2 ROS2 packages
ament_target_dependencies(kafeiche_drivers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}) #this executable file depends on many ROS2 packages
 
# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface kafeiche_drivers.xml)

# Install the executable
install(
  TARGETS encoder
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS motor
  DESTINATION lib/${PROJECT_NAME}
)

# Install library
install(TARGETS kafeiche_drivers
  EXPORT export_kafeiche_drivers
  DESTINATION lib/${PROJECT_NAME}
)

# Install yaml
install(
  DIRECTORY config
  DESTINATION share/kafeiche_drivers
)

# EXPORTS
ament_export_targets(export_kafeiche_drivers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
