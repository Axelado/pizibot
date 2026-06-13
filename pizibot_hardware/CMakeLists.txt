cmake_minimum_required(VERSION 3.8)
project(pizibot_hardware)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY param launch rviz
        DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        scripts/esp32cam_stream_getter.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
