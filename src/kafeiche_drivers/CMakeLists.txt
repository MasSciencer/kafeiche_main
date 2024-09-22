cmake_minimum_required(VERSION 3.8)
project(kafeiche_drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find WiringPi library
find_library(WIRINGPI_LIBRARIES wiringPi)
if(NOT WIRINGPI_LIBRARIES)
    message(FATAL_ERROR "WiringPi library not found!")
endif()

# Add executable
add_executable(encoders src/encoders.cpp)
add_executable(motor src/motor.cpp)

# Link the libraries
ament_target_dependencies(encoders
  rclcpp
  std_msgs
)

ament_target_dependencies(motor
  rclcpp
  std_msgs
)

target_link_libraries(encoders ${WIRINGPI_LIBRARIES})
target_link_libraries(motor ${WIRINGPI_LIBRARIES})

# Install the executable
install(TARGETS encoders motor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
