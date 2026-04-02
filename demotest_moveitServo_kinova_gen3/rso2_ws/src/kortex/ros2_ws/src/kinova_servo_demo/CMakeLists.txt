cmake_minimum_required(VERSION 3.8)
project(kinova_servo_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning REQUIRED) 
find_package(sensor_msgs REQUIRED)

# Add the executable
add_executable(kinova_integrated_servo src/kinova_integrated_servo.cpp)

# Use ONLY ament_target_dependencies to avoid the "keyword signature" conflict
ament_target_dependencies(kinova_integrated_servo
  rclcpp
  moveit_servo
  moveit_msgs
  control_msgs
  geometry_msgs
  tf2_ros
  moveit_ros_planning 
  sensor_msgs
)

# Install the executable
install(TARGETS kinova_integrated_servo
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
