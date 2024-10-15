cmake_minimum_required(VERSION 3.8)
project(pizibot_hardware)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(battery_hardware_interface REQUIRED)

# Ajouter les chemins d'inclusion manuels pour pigpio
include_directories(/usr/local/include)

add_executable(battery_management_node src/battery_management_node.cpp)

ament_target_dependencies(battery_management_node
                          rclcpp
                          sensor_msgs)

target_link_libraries(battery_management_node /usr/local/lib/libpigpio.so)


install(TARGETS battery_management_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME})

ament_package()
