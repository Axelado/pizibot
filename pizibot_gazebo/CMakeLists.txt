cmake_minimum_required(VERSION 3.8)
project(pizibot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pizibot_description REQUIRED)

find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(yaml_modifier_node src/yaml_modifier_node.cpp)

ament_target_dependencies(yaml_modifier_node rclcpp)

target_link_libraries(yaml_modifier_node yaml-cpp)

install(TARGETS yaml_modifier_node
        DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY param launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
