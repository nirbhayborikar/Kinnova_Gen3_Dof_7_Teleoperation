
cmake_minimum_required(VERSION 3.8)
project(sensagram_teleop)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) # action
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)                    # ADDED THIS
find_package(tf2_ros REQUIRED)  
              # ADD THIS
find_package(tf2_geometry_msgs REQUIRED)      # ADD THIS

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(sensagram_teleop_node
  src/sensagram_teleop_node.cpp
)

target_include_directories(sensagram_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(sensagram_teleop_node
  rclcpp
  rclcpp_action
  geometry_msgs
  control_msgs
  moveit_msgs
  trajectory_msgs
  tf2                         # ADD THIS
  tf2_ros                     # ADD THIS
  tf2_geometry_msgs           # ADD THIS
  std_msgs
  std_srvs
)

target_link_libraries(sensagram_teleop_node
  nlohmann_json::nlohmann_json
)

install(TARGETS sensagram_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  geometry_msgs
  control_msgs
  moveit_msgs
  std_msgs
  std_srvs
)

ament_package()