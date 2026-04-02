cmake_minimum_required(VERSION 3.8)
project(camera_republisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

# Add the camera republisher component
add_library(camera_republisher_component SHARED
  src/camera_republisher_component.cpp)

target_include_directories(camera_republisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(camera_republisher_component PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(camera_republisher_component
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport)

# Register the component
rclcpp_components_register_nodes(camera_republisher_component "camera_republisher::CameraRepublisherComponent")

# Install the library
install(TARGETS camera_republisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install headers
install(DIRECTORY include/
  DESTINATION include)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()