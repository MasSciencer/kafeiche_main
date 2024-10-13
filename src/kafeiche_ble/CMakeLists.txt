cmake_minimum_required(VERSION 3.8)
project(kafeiche_ble)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Find bluez library and headers
find_package(PkgConfig REQUIRED)
pkg_check_modules(BLUEZ REQUIRED bluez)

if(NOT BLUEZ_FOUND)
    message(FATAL_ERROR "bluez library not found!")
endif()

# Add executable
add_executable(ble_command_controller src/ble_command_controller.cpp)

# Link the bluez library and ROS2 dependencies
ament_target_dependencies(ble_command_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ble_command_controller PRIVATE ${BLUEZ_LIBRARIES})
target_include_directories(ble_command_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/ble_command_controller>
)

# Set compile features and include directories
target_compile_features(ble_command_controller PUBLIC cxx_std_17)
target_include_directories(ble_command_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/kafeiche_ble>
)

# Install the executable
install(
  TARGETS ble_command_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

