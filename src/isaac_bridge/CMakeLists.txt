cmake_minimum_required(VERSION 3.8)
project(isaac_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
