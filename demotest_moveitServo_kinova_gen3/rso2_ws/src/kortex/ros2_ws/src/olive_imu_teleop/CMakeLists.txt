cmake_minimum_required(VERSION 3.8)
project(olive_imu_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Create executable
add_executable(olive_imu_teleop_node
  src/olive_imu_teleop_node.cpp
)

# Include directories
target_include_directories(olive_imu_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(olive_imu_teleop_node
  rclcpp
  geometry_msgs
  tf2_msgs
  moveit_msgs
)

# Install executable
install(TARGETS
  olive_imu_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
