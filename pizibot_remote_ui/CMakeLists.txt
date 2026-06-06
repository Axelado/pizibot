cmake_minimum_required(VERSION 3.8)
project(pizibot_remote_ui)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)

# Install the built frontend (run `npm run build` in web/ first)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/web/dist")
  install(DIRECTORY web/dist/ DESTINATION share/${PROJECT_NAME}/web)
endif()

ament_package()
