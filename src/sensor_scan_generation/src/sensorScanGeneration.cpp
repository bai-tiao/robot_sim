/**
 * @file sensorScanGeneration.cpp
 * @brief 传感器扫描生成节点
 *
 * 该节点的核心职责是解决一个时间对齐问题：
 *   SLAM 输出的里程计（/state_estimation）和配准点云（/registered_scan）
 *   来自不同时刻，而下游的地形分析和规划节点需要"同一时刻"的位姿与点云。
 *
 * 具体做法：
 *   1. 用 message_filters 对里程计和点云做时间近似同步；
 *   2. 将点云从世界坐标系（map）逆变换回传感器坐标系（sensor_at_scan），
 *      即重建出"扫描时刻传感器看到的局部点云"；
 *   3. 发布与点云时间戳对齐的里程计（/state_estimation_at_scan）
 *      以及对应的 TF（map → sensor_at_scan）；
 *   4. 发布传感器坐标系下的点云（/sensor_scan）。
 *
 * 订阅：
 *   /state_estimation   (nav_msgs/Odometry)    — SLAM 输出的位姿
 *   /registered_scan    (sensor_msgs/PointCloud2) — SLAM 输出的配准点云（世界系）
 *
 * 发布：
 *   /state_estimation_at_scan  (nav_msgs/Odometry)    — 与点云时间戳对齐的位姿
 *   /sensor_scan               (sensor_msgs/PointCloud2) — 传感器坐标系下的点云
 *   TF: map → sensor_at_scan
 */

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

// 世界坐标系下的配准点云（SLAM 输出，map 系）
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
// 逆变换后的点云（传感器坐标系 sensor_at_scan 下的局部点云）
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZ>());

// 机器人在世界系中的位置和姿态（当前暂未在逆变换中直接使用，由 transformToMap 承载）
double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

bool newTransformToMap = false;

// 缓存最新收到的里程计消息
nav_msgs::msg::Odometry odometryIn;
// 发布与点云对齐的里程计
shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pubOdometryPointer;
// 从里程计构建的 map→sensor 变换（用于点云逆变换）
tf2::Stamped<tf2::Transform> transformToMap;
geometry_msgs::msg::TransformStamped transformTfGeom;

// TF 广播器，用于发布 map → sensor_at_scan 变换
unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPointer;
// 发布传感器坐标系下的点云
shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubLaserCloud;

/**
 * @brief 里程计与点云时间同步回调函数
 *
 * 由 message_filters 保证两个话题时间戳近似对齐后才触发此回调。
 * 主要完成以下三件事：
 *   A. 将世界系点云逆变换回传感器坐标系
 *   B. 发布时间戳对齐的里程计 + TF
 *   C. 发布传感器坐标系下的点云
 *
 * @param odometry  与点云近似同步的里程计消息（SLAM 输出）
 * @param laserCloud2  SLAM 配准后的点云（世界坐标系 map 下）
 */
void laserCloudAndOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odometry,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
  laserCloudIn->clear();
  laserCLoudInSensorFrame->clear();

  // 将 ROS PointCloud2 消息转为 PCL 格式
  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  odometryIn = *odometry;

  // 从里程计中提取 map→sensor 的平移和旋转，构建 tf2 变换
  // 该变换表示"扫描时刻传感器在世界系中的位姿"
  transformToMap.setOrigin(
      tf2::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf2::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  int laserCloudInNum = laserCloudIn->points.size();

  pcl::PointXYZ p1;
  tf2::Vector3 vec;

  // ---- A. 将点云从世界系（map）逆变换到传感器系（sensor_at_scan）----
  // transformToMap 是 map→sensor 的正变换，其逆即 sensor→map 的逆 = map→sensor 坐标变换
  // 对每个点：p_sensor = T_map_sensor^{-1} * p_map
  //
  // 距离与角度裁剪参数（模拟真实雷达视野）
  const float MAX_RANGE = 20.0f;       // 最大量程 (m)
  const float MIN_RANGE = 0.1f;        // 最小量程 (m), 过滤自身遮挡
  const float H_FOV    = M_PI;         // 水平视野: 前方 ±90° = 180° total (rad)
  const float V_FOV_UP  =  M_PI_2;     // 垂直向上 +90° (半球形)
  const float V_FOV_DOWN= -M_PI_2;     // 垂直向下 -90°

  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);

    // 用变换的逆将点从世界系映射回传感器本体系
    vec = transformToMap.inverse() * vec;

    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();

    // ---- 距离裁剪 ----
    float dist = sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z);
    if (dist < MIN_RANGE || dist > MAX_RANGE)
      continue;

    // ---- 水平角度裁剪: 只保留前方 ±(H_FOV/2) ----
    // sensor 系: X 轴朝前，水平角 = atan2(y, x)
    float h_angle = atan2(p1.y, p1.x);
    if (h_angle < -H_FOV / 2.0f || h_angle > H_FOV / 2.0f)
      continue;

    // ---- 垂直角度裁剪 ----
    float v_angle = atan2(p1.z, sqrt(p1.x * p1.x + p1.y * p1.y));
    if (v_angle < V_FOV_DOWN || v_angle > V_FOV_UP)
      continue;

    laserCLoudInSensorFrame->points.push_back(p1);
  }

  // ---- B. 发布与点云时间戳对齐的里程计 + TF ----
  // 将里程计时间戳替换为点云时间戳，确保下游节点的时间一致性
  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "map";
  odometryIn.child_frame_id = "sensor_at_scan";
  pubOdometryPointer->publish(odometryIn);

  // 广播 TF：map → sensor_at_scan（时间戳与点云对齐）
  transformToMap.frame_id_ = "map";
  transformTfGeom = tf2::toMsg(transformToMap);
  transformTfGeom.header.stamp = laserCloud2->header.stamp;
  transformTfGeom.child_frame_id = "sensor_at_scan";
  tfBroadcasterPointer->sendTransform(transformTfGeom);

  // ---- C. 发布传感器坐标系下的点云 ----
  sensor_msgs::msg::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "sensor_at_scan";  // 声明点云所在坐标系
  pubLaserCloud->publish(scan_data);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("sensor_scan");

  // ---- 消息过滤器：对里程计和点云做时间近似同步 ----
  // 目的：里程计频率（100Hz）远高于点云频率（10Hz），需要找到时间上最接近的一对消息
  message_filters::Subscriber<nav_msgs::msg::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subLaserCloud;

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  
  // 自定义 QoS：深度为 1，BEST_EFFORT 可靠性（匹配传感器驱动的发布 QoS）
  // 使用 BEST_EFFORT 可以降低延迟，避免因可靠性协商失败导致收不到数据
  rmw_qos_profile_t qos_profile=
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,                                         // 队列深度为 1，只保留最新消息
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,    // 尽力传输，不重传
    RMW_QOS_POLICY_DURABILITY_VOLATILE,        // 不持久化
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };

  // 订阅 SLAM 输出：里程计 + 配准点云，队列大小 100（给同步器足够的缓冲窗口）
  subOdometry.subscribe(nh, "/state_estimation", qos_profile);
  subLaserCloud.subscribe(nh, "/registered_scan", qos_profile);
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(std::bind(laserCloudAndOdometryHandler, placeholders::_1, placeholders::_2));

  // 发布时间对齐后的里程计（供 terrain_analysis 等节点使用）
  pubOdometryPointer = nh->create_publisher<nav_msgs::msg::Odometry>("/state_estimation_at_scan", 5);

  tfBroadcasterPointer = std::make_unique<tf2_ros::TransformBroadcaster>(*nh);

  // 发布传感器坐标系下的点云（供 terrain_analysis / local_planner 使用）
  pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor_scan", 2);

  rclcpp::spin(nh);

  return 0;
}
