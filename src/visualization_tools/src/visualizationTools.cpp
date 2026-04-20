/**
 * @file visualizationTools.cpp
 * @brief 可视化与指标统计节点
 *
 * 该节点不参与任何规划决策，职责是：
 *   1. 【可视化】将 SLAM 数据转化为 RViz 可显示的话题：
 *      - 已探索区域点云（/explored_areas）
 *      - 机器人运动轨迹（/trajectory）
 *      - 预加载的全局地图（/overall_map，从 .ply 文件读取）
 *   2. 【指标统计】实时计算并发布探索效果量化指标：
 *      - 已探索体积 /explored_volume（m³）
 *      - 累计行驶距离 /traveling_distance（m）
 *      - 运行时长 /time_duration（s）
 *   3. 【数据记录】可选地将以上指标、轨迹、点云保存到文件（txt 格式）
 *
 * 主要用于 TARE 探索规划场景的效果评估，也可与 FAR Planner 配合使用。
 *
 * 订阅：
 *   /state_estimation   (nav_msgs/Odometry)      — 机器人位姿，用于轨迹记录
 *   /registered_scan    (sensor_msgs/PointCloud2) — 配准点云，用于探索区域/体积统计
 *   /runtime            (std_msgs/Float32)        — 规划器运行时间（由 TARE 发布）
 *
 * 发布：
 *   /overall_map        (PointCloud2) — 预加载的全局参考地图
 *   /explored_areas     (PointCloud2) — 累积已探索点云（体素降采样后）
 *   /trajectory         (PointCloud2) — 机器人历史轨迹（intensity=行驶距离）
 *   /explored_volume    (Float32)     — 已探索体积（m³）
 *   /traveling_distance (Float32)     — 累计行驶距离（m）
 *   /time_duration      (Float32)     — 自系统初始化以来的运行时长（s）
 */

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/polygon_stamped.h>
#include <geometry_msgs/msg/point_stamped.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

const double PI = 3.1415926;

// ---- 文件路径参数（由 launch 文件注入，路径在 main 中会被修正） ----
string metricFile;   // 探索指标保存路径（exploredVolume, travelingDis, runtime, timeDuration）
string trajFile;     // 轨迹保存路径（x, y, z, roll, pitch, yaw, timeDuration）
string pcdFile;      // 点云保存路径（x, y, z, intensity, timeDuration）
string mapFile;      // 预加载全局地图的 .ply 文件路径

// ---- 体素滤波器叶子大小（控制点云精度与内存占用的权衡） ----
double overallMapVoxelSize = 0.5;       // 全局地图降采样分辨率（m）
double exploredAreaVoxelSize = 0.3;     // 已探索区域降采样分辨率（m）
double exploredVolumeVoxelSize = 0.5;   // 已探索体积计算用降采样分辨率（m）

// ---- 轨迹采样阈值：连续两个轨迹点之间的最小位移和偏航角变化 ----
double transInterval = 0.2;   // 位移阈值（m），小于此值不添加新轨迹点
double yawInterval = 10.0;    // 偏航角阈值（度），小于此值不添加新轨迹点

// ---- 显示频率控制（避免每帧都发布大型点云） ----
int overallMapDisplayInterval = 2;    // 全局地图每隔多少秒发布一次
int overallMapDisplayCount = 0;
int exploredAreaDisplayInterval = 1;  // 已探索区域每隔多少秒发布一次
int exploredAreaDisplayCount = 0;

// ---- 数据保存开关（默认关闭，由 launch 参数控制） ----
bool saveMetric = false;  // 是否保存探索指标到文件
bool saveTraj = false;    // 是否保存轨迹到文件
bool savePcd = false;     // 是否保存点云到文件

// ---- 点云缓冲区 ----
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());          // 当前帧点云
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());        // 从 .ply 读取的全局地图（原始）
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());     // 全局地图降采样后（用于发布）
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZI>());    // 已探索区域累积点云
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());   // 已探索区域降采样缓冲
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 用于体积计算的累积点云
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZI>());// 体积计算降采样缓冲
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());           // 历史轨迹点云（intensity=行驶距离）

// ---- 系统初始化延迟（等待 SLAM 稳定后再开始统计） ----
const int systemDelay = 5;          // 延迟帧数（等待 5 帧点云后才开始工作）
int systemDelayCount = 0;
bool systemDelayInited = false;     // 延迟等待是否完成

double systemTime = 0;              // 当前系统时间（秒）
double systemInitTime = 0;          // 系统正式初始化的时刻（第一个有效轨迹点时刻）
bool systemInited = false;          // 系统是否已正式初始化（开始统计指标）

// ---- 机器人当前位置和姿态 ----
float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

// ---- 实时统计量 ----
float exploredVolume = 0;   // 已探索体积（体素数 × 体素体积，m³）
float travelingDis = 0;     // 累计行驶距离（m）
float runtime = 0;          // 规划器运行时间（由 /runtime 话题提供）
float timeDuration = 0;     // 系统运行时长（s）

// ---- 体素滤波器 ----
pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;

// 预先转好格式的全局地图（只转换一次，在 main 循环中周期性发布）
sensor_msgs::msg::PointCloud2 overallMap2;

// ---- 发布器指针（全局，供回调函数使用） ----
shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubExploredAreaPtr;   // 已探索区域点云
shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubTrajectoryPtr;     // 轨迹点云
shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubExploredVolumePtr;        // 已探索体积
shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubTravelingDisPtr;          // 累计行驶距离
shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubTimeDurationPtr;          // 运行时长

// ---- 文件指针 ----
FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;
FILE *pcdFilePtr = NULL;

/**
 * @brief 里程计回调：更新机器人轨迹、行驶距离和运行时长
 *
 * 采样策略：并非每帧都记录，只有当位移超过 transInterval 或
 * 偏航角变化超过 yawInterval 时，才添加新轨迹点。
 * 这样可以有效控制轨迹点云的大小。
 */
void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  systemTime = rclcpp::Time(odom->header.stamp).seconds();

  // 从四元数提取欧拉角
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  // 计算与上一个轨迹点的偏航角差（处理跨 ±π 的情况）
  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI) dYaw = 2 * PI  - dYaw;

  // 计算与上一个轨迹点的欧氏距离
  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  // 系统延迟期间：只更新位置缓存，不做任何统计（等待 SLAM 稳定）
  if (!systemDelayInited) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  // 系统已初始化后：持续发布运行时长
  if (systemInited) {
    timeDuration = systemTime - systemInitTime;
    
    std_msgs::msg::Float32 timeDurationMsg;
    timeDurationMsg.data = timeDuration;
    pubTimeDurationPtr->publish(timeDurationMsg);
  }

  // 采样过滤：位移和偏航角变化都未超过阈值，跳过本帧（不记录轨迹点）
  if (dis < transInterval && dYaw < yawInterval) {
    return;
  }

  // 第一个满足条件的轨迹点：记录系统初始化时刻，重置距离累计
  if (!systemInited) {
    dis = 0;
    systemInitTime = systemTime;
    systemInited = true;
  }

  // 累加行驶距离
  travelingDis += dis;

  // 更新当前位置缓存
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  // 可选：将轨迹点写入文件（格式：x y z roll pitch yaw timeDuration）
  if (saveTraj) {
    fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);
    fflush(trajFilePtr);
  }

  // 将当前位置添加到轨迹点云（intensity 编码累计行驶距离，可在 RViz 中着色）
  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  // 发布轨迹点云（每次有新轨迹点时立即发布）
  sensor_msgs::msg::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "map";
  pubTrajectoryPtr->publish(trajectory2);
}

/**
 * @brief 点云回调：统计已探索体积和已探索区域，并可选保存点云
 *
 * 已探索体积的计算方法：
 *   将所有历史点云合并后做体素降采样，体素数量 × 体素体积 = 已探索体积
 *   这是一种高效的近似方法，避免了精确的凸包计算。
 *
 * 已探索区域（/explored_areas）以更低的频率发布（每 exploredAreaDisplayInterval 秒一次），
 * 避免大型点云频繁发布占用带宽。
 */
void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudIn)
{
  // 系统延迟计数：等待前 systemDelay 帧点云后再开始工作
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  // 系统未正式初始化（轨迹还没有第一个点），暂不统计
  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  // 可选：将当前帧点云写入文件（格式：x y z intensity timeDuration）
  if (savePcd) {
    float timeDuration2 = rclcpp::Time(laserCloudIn->header.stamp).seconds() - systemInitTime;
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      fprintf(pcdFilePtr, "%f %f %f %f %f\n", laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z, laserCloud->points[i].intensity, timeDuration2);
    }
    fflush(pcdFilePtr);
  }

  // ---- 已探索体积统计 ----
  // 将当前帧点云累加进历史体积点云
  *exploredVolumeCloud += *laserCloud;

  // 对累积点云做体素降采样（in-place 双缓冲交换）
  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  // 双缓冲交换（避免内存重分配）
  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  // 体积 = 体素数 × 单个体素体积（边长^3）
  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  // ---- 已探索区域点云（用于 RViz 显示） ----
  // 精度比体积统计更高（exploredAreaVoxelSize 更小），但发布频率低于体积统计
  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 10 * exploredAreaDisplayInterval) {
    // 降采样后发布（10Hz × exploredAreaDisplayInterval 秒 = 每隔 N 秒发布一次）
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::msg::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  // 可选：将探索指标写入文件（格式：exploredVolume travelingDis runtime timeDuration）
  if (saveMetric) {
    fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);
    fflush(metricFilePtr);
  }

  // 发布探索体积和行驶距离指标（每帧都发布，供 RViz Panel 或评估脚本订阅）
  std_msgs::msg::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);

  std_msgs::msg::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}

/**
 * @brief 规划器运行时间回调（由 TARE 等规划器发布 /runtime）
 * 仅做缓存，在保存 metric 文件时一并写入
 */
void runtimeHandler(const std_msgs::msg::Float32::ConstSharedPtr runtimeIn)
{
  runtime = runtimeIn->data;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("visualizationTools");

  // ---- 参数声明与读取（由 launch 文件注入） ----
  nh->declare_parameter<std::string>("metricFile", metricFile);
  nh->declare_parameter<std::string>("trajFile", trajFile);
  nh->declare_parameter<std::string>("pcdFile", pcdFile);
  nh->declare_parameter<std::string>("mapFile", mapFile);
  nh->declare_parameter<double>("overallMapVoxelSize", overallMapVoxelSize);
  nh->declare_parameter<double>("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nh->declare_parameter<double>("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nh->declare_parameter<double>("transInterval", transInterval);
  nh->declare_parameter<double>("yawInterval", yawInterval);
  nh->declare_parameter<int>("overallMapDisplayInterval", overallMapDisplayInterval);
  nh->declare_parameter<int>("exploredAreaDisplayInterval", exploredAreaDisplayInterval);
  nh->declare_parameter<bool>("saveMetric", saveMetric);
  nh->declare_parameter<bool>("saveTraj", saveTraj);
  nh->declare_parameter<bool>("savePcd", savePcd);

  nh->get_parameter("metricFile", metricFile);
  nh->get_parameter("trajFile", trajFile);
  nh->get_parameter("pcdFile", pcdFile);
  nh->get_parameter("mapFile", mapFile);
  nh->get_parameter("overallMapVoxelSize", overallMapVoxelSize);
  nh->get_parameter("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nh->get_parameter("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nh->get_parameter("transInterval", transInterval);
  nh->get_parameter("yawInterval", yawInterval);
  nh->get_parameter("overallMapDisplayInterval", overallMapDisplayInterval);
  nh->get_parameter("exploredAreaDisplayInterval", exploredAreaDisplayInterval);
  nh->get_parameter("saveMetric", saveMetric);
  nh->get_parameter("saveTraj", saveTraj);
  nh->get_parameter("savePcd", savePcd);

  // 仅当路径包含 /install/ 时才做路径替换（兼容直接传入绝对路径的情况）
  auto safe_replace = [](std::string& s, const std::string& from, const std::string& to) {
    auto pos = s.find(from);
    if (pos != std::string::npos) s.replace(pos, from.size(), to);
  };
  safe_replace(mapFile,    "/install/", "/src/base_autonomy/");
  safe_replace(metricFile, "/install/", "/src/base_autonomy/");
  safe_replace(trajFile,   "/install/", "/src/base_autonomy/");
  safe_replace(pcdFile,    "/install/", "/src/base_autonomy/");

  // ---- 订阅 ----
  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odometryHandler);
  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan", 5, laserCloudHandler);
  auto subRuntime = nh->create_subscription<std_msgs::msg::Float32>("/runtime", 5, runtimeHandler);

  // ---- 发布 ----
  auto pubOverallMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/overall_map", 5);
  pubExploredAreaPtr = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/explored_areas", 5);
  pubTrajectoryPtr = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/trajectory", 5);
  pubExploredVolumePtr = nh->create_publisher<std_msgs::msg::Float32>("/explored_volume", 5);
  pubTravelingDisPtr = nh->create_publisher<std_msgs::msg::Float32>("/traveling_distance", 5);
  pubTimeDurationPtr = nh->create_publisher<std_msgs::msg::Float32>("/time_duration", 5);

  // ---- 初始化体素滤波器 ----
  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

  // ---- 预加载全局参考地图（.ply 格式） ----
  // 该地图仅用于 RViz 显示，不参与规划计算
  // 若文件不存在，节点继续运行，只是不发布 /overall_map
  pcl::PLYReader ply_reader;
  if (ply_reader.read(mapFile, *overallMapCloud) == -1) {
    RCLCPP_INFO(nh->get_logger(), "Couldn't read pointcloud.ply file.");
  }

  // 降采样后转为 ROS 消息（只转换一次，之后在主循环中周期性重发）
  overallMapCloudDwz->clear();
  overallMapDwzFilter.setInputCloud(overallMapCloud);
  overallMapDwzFilter.filter(*overallMapCloudDwz);
  overallMapCloud->clear();  // 释放原始点云内存

  int overallMapCloudDwzSize = overallMapCloudDwz->points.size();
  pcl::toROSMsg(*overallMapCloudDwz, overallMap2);

  // ---- 生成带时间戳的日志文件名（格式：原始路径_年-月-日-时-分-秒.txt） ----
  time_t logTime = time(0);
  tm *ltm = localtime(&logTime);
  string timeString = to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" + to_string(ltm->tm_mday) + "-" +
                      to_string(ltm->tm_hour) + "-" + to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);

  metricFile += "_" + timeString + ".txt";
  trajFile += "_" + timeString + ".txt";
  pcdFile += "_" + timeString + ".txt";

  // 按开关决定是否创建文件
  if (saveMetric) metricFilePtr = fopen(metricFile.c_str(), "w");
  if (saveTraj) trajFilePtr = fopen(trajFile.c_str(), "w");
  if (savePcd) pcdFilePtr = fopen(pcdFile.c_str(), "w");

  // ---- 主循环（100Hz）：周期性发布全局地图 ----
  // 其他发布（轨迹、探索区域、指标）在各自的回调函数中完成
  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    // 每隔 overallMapDisplayInterval 秒发布一次全局地图
    overallMapDisplayCount++;
    if (overallMapDisplayCount >= 100 * overallMapDisplayInterval) {
      if (overallMapCloudDwzSize > 0) {
        overallMap2.header.stamp = rclcpp::Time(static_cast<uint64_t>(systemTime * 1e9));
        overallMap2.header.frame_id = "map";
        pubOverallMap->publish(overallMap2);
      }

      overallMapDisplayCount = 0;
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  // ---- 节点退出时关闭所有文件 ----
  if (saveMetric) fclose(metricFilePtr);
  if (saveTraj) fclose(trajFilePtr);
  if (savePcd) fclose(pcdFilePtr);

  RCLCPP_INFO(nh->get_logger(), "Exploration metrics and vehicle trajectory are saved in 'src/vehicle_simulator/log'.");

  return 0;
}
