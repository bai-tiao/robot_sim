cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  params
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
