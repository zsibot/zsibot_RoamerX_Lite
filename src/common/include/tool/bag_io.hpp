#pragma once

#include <any>
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "robots_dog_msgs/msg/custom_msg.hpp"

namespace zsibot::common {

class RosbagIO {
public:
  explicit RosbagIO(std::string bag_file) : bag_file_(std::move(bag_file)) {}
  ~RosbagIO() = default;

  using MsgType = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;
  using MessageProcessFunction = std::function<bool(const MsgType &m)>;

  /// 一些方便直接使用的topics, messages
  using Scan2DHandle =
      std::function<bool(sensor_msgs::msg::LaserScan::SharedPtr)>;
  using OdomHandle = std::function<bool(nav_msgs::msg::Odometry::SharedPtr)>;
  using ImuHandle = std::function<bool(sensor_msgs::msg::Imu::SharedPtr)>;
  using PointCloudHandle =
      std::function<bool(sensor_msgs::msg::PointCloud2::SharedPtr)>;
  using CustomMsgHandle =
      std::function<bool(robots_dog_msgs::msg::CustomMsg::SharedPtr)>;

  void Go(int sleep_usec = 0);

  void Stop() { exit_.store(true); }

  /// 通用处理函数
  RosbagIO &AddHandle(const std::string &topic_name,
                      MessageProcessFunction func) {
    process_func_.emplace(topic_name, func);
    return *this;
  }

  /// 2D激光处理
  RosbagIO &AddScan2DHandle(const std::string &topic_name, Scan2DHandle f) {

    return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      rclcpp::SerializedMessage data(*m->serialized_data);
      seri_scan_.deserialize_message(&data, scan_msg.get());
      return f(scan_msg);
    });
  }

  /// odom 处理
  RosbagIO &AddOdomHandle(const std::string &topic_name, OdomHandle f) {
    return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
      auto scan_msg = std::make_shared<nav_msgs::msg::Odometry>();
      rclcpp::SerializedMessage data(*m->serialized_data);
      seri_odom_.deserialize_message(&data, scan_msg.get());
      return f(scan_msg);
    });
  }

  /// IMU
  RosbagIO &AddIMUHandle(const std::string &topic_name, ImuHandle f) {
    return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
      auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
      rclcpp::SerializedMessage data(*m->serialized_data);
      seri_imu_.deserialize_message(&data, imu_msg.get());
      return f(imu_msg);
    });
  }

  /// point cloud2
  RosbagIO &AddCloudHandle(const std::string &topic_name, PointCloudHandle f) {
    return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
      auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      rclcpp::SerializedMessage data(*m->serialized_data);
      seri_cloud_.deserialize_message(&data, cloud_msg.get());
      return f(cloud_msg);
    });
  }

  /// custom msg
  RosbagIO &AddCustomMsgHandle(const std::string &topic_name, CustomMsgHandle f) {
    return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
      auto cloud_msg = std::make_shared<robots_dog_msgs::msg::CustomMsg>();
      rclcpp::SerializedMessage data(*m->serialized_data);
      seri_custommsg_.deserialize_message(&data, cloud_msg.get());
      return f(cloud_msg);
    });
  }

  /// 清除现有的处理函数
  void CleanProcessFunc() { process_func_.clear(); }

private:
  std::atomic<bool> exit_ = false;
  std::atomic<bool> flg_next_ = false;
  std::atomic<float> play_speed_ = 10.0;

  std::map<std::string, MessageProcessFunction> process_func_;

  /// 序列化
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> seri_scan_;
  rclcpp::Serialization<nav_msgs::msg::Odometry> seri_odom_;
  rclcpp::Serialization<sensor_msgs::msg::Imu> seri_imu_;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> seri_cloud_;
  rclcpp::Serialization<robots_dog_msgs::msg::CustomMsg> seri_custommsg_;

  std::string bag_file_;
};
} // namespace zsibot::common
