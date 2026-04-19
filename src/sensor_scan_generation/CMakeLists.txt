cmake_minimum_required(VERSION 3.8)
project(sensor_scan_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

# sensorScanGeneration: map 系点云 → sensor_at_scan 系局部点云
add_executable(sensorScanGeneration src/sensorScanGeneration.cpp)
ament_target_dependencies(sensorScanGeneration
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  message_filters pcl_ros pcl_conversions)
target_link_libraries(sensorScanGeneration ${PCL_LIBRARIES})

install(TARGETS sensorScanGeneration
  DESTINATION lib/${PROJECT_NAME})

ament_package()
