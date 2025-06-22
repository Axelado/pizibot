cmake_minimum_required(VERSION 3.8)
project(pizibot_vision)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# C++ executable
add_executable(camera_publisher_node src/camera_publisher_node.cpp)
ament_target_dependencies(camera_publisher_node
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
)
install(TARGETS camera_publisher_node
    DESTINATION lib/${PROJECT_NAME}
)

include_directories(include)

# Install include directory if you have headers (optional)
# install(DIRECTORY include/
#     DESTINATION include/${PROJECT_NAME}
# )

# Install param and launch directories if they exist
# install(DIRECTORY param launch
#     DESTINATION share/${PROJECT_NAME}
# )

# Python support (if needed)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python scripts (add your scripts here if any)
# install(PROGRAMS
#     # scripts/my_python_node.py
#     DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
