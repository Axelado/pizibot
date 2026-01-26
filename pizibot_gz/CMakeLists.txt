cmake_minimum_required(VERSION 3.8)
project(pizibot_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pizibot_description REQUIRED)

find_package(rclcpp REQUIRED)

install(
  DIRECTORY params launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs)
find_package(tf2)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(dataset_saver src/dataset_saver.cpp)
ament_target_dependencies(dataset_saver 
  rclcpp 
  sensor_msgs
  tf2_msgs
  tf2
  tf2_geometry_msgs
  message_filters
  cv_bridge
) 
target_link_libraries(dataset_saver ${OpenCV_LIBS})
install(TARGETS dataset_saver
        DESTINATION lib/${PROJECT_NAME})

ament_package()
