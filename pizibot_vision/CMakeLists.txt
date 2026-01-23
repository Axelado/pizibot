# ==============================================================================
# Pizibot Vision Package - CMake Build Configuration
# ==============================================================================
# ROS 2 Jazzy compatible camera image capture and publishing
#
# Targets:
#   - camera_publisher_node: Main executable for camera image publishing
#
# Install locations:
#   - camera_publisher_node -> lib/pizibot_vision/
#   - launch files -> share/pizibot_vision/launch/
#   - config files -> share/pizibot_vision/config/
# ==============================================================================

cmake_minimum_required(VERSION 3.8)
project(pizibot_vision)

# ==============================================================================
# C++ Standard Configuration
# ==============================================================================
# Require C++17 for ROS 2 Jazzy compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# ==============================================================================
# Compiler Options
# ==============================================================================
# Enable strict compiler warnings to catch potential issues
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# Dependency Discovery
# ==============================================================================
# ROS 2 core packages
find_package(ament_cmake REQUIRED)        # CMake build system
find_package(ament_cmake_python REQUIRED) # Python support
find_package(rclcpp REQUIRED)              # ROS 2 C++ client library
find_package(rclpy REQUIRED)               # ROS 2 Python client library

# ROS 2 message definitions
find_package(sensor_msgs REQUIRED)        # Image and sensor messages
find_package(std_msgs REQUIRED)            # Standard message types

# Image processing and conversion
find_package(cv_bridge REQUIRED)           # OpenCV ↔ ROS image conversion
find_package(image_transport REQUIRED)     # Efficient image transport
find_package(OpenCV REQUIRED)              # OpenCV library

# ==============================================================================
# Include Directories
# ==============================================================================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ==============================================================================
# Executable Target: camera_publisher_node
# ==============================================================================
# C++ executable for capturing and publishing camera images
add_executable(camera_publisher_node src/camera_publisher_node.cpp)

# Link ROS 2 dependencies
ament_target_dependencies(camera_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

# Link OpenCV libraries
target_link_libraries(camera_publisher_node ${OpenCV_LIBS})

# ==============================================================================
# Installation Targets
# ==============================================================================
# Install C++ executables
install(TARGETS
  camera_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Python Package Setup
# ==============================================================================
ament_python_install_package(${PROJECT_NAME})

# ==============================================================================
# Install Configuration and Launch Files
# ==============================================================================
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ==============================================================================
# Testing Configuration
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ==============================================================================
# Package Finalization
# ==============================================================================
ament_package()
