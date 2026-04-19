cmake_minimum_required(VERSION 3.8)
project(vehicle_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# opencv_imgcodecs 间接链 libgdal → libcurl，存在 CURL_OPENSSL 版本符号冲突
# --allow-shlib-undefined 允许运行时动态解析，避免链接阶段报错
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")

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
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

# vehicleSimulator: 虚拟里程计 + Unity 位置同步
add_executable(vehicleSimulator src/vehicleSimulator.cpp)
ament_target_dependencies(vehicleSimulator
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  message_filters pcl_ros pcl_conversions)
target_link_libraries(vehicleSimulator ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# sim_image_repub: Unity 压缩图 → raw sensor_msgs/Image
add_executable(sim_image_repub src/sim_image_repub.cpp)
ament_target_dependencies(sim_image_repub rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(sim_image_repub ${OpenCV_LIBRARIES})

install(TARGETS vehicleSimulator sim_image_repub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
