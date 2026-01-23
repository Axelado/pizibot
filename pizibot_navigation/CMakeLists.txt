cmake_minimum_required(VERSION 3.8)
project(pizibot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build initial_pose_publisher node
add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs)

# Install executables
install(TARGETS
  initial_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, params, rviz configs, and maps
install(
  DIRECTORY launch params rviz map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
