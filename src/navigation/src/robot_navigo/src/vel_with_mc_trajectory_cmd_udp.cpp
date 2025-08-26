#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

#include "highlevel_connector.h"
#include "robots_dog_msgs/msg/cmd_vel_with_trajectory.hpp"

class VelWithMcTrajecotryCmdUdpPublisher : public rclcpp::Node {
 public:
  VelWithMcTrajecotryCmdUdpPublisher()
      : Node("vel_with_mc_trajectory_cmd_udp_publisher") {
    planner_vel_with_mc_traj_cmd_subscriber =
        this->create_subscription<robots_dog_msgs::msg::CmdVelWithTrajectory>(
            "/cmd_vel_with_mc_trajectory", 10,
            std::bind(&VelWithMcTrajecotryCmdUdpPublisher::
                          HandlPlannerVelWithMcTrajectoryCallback,
                      this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "vel_with_mc_trajectory_cmd_udp_publisher started");

    // Read platform params
    this->declare_parameter("platform",
                            rclcpp::ParameterValue(std::string("")));
    this->get_parameter("platform", platform_);

    // Platform reflection table
    const std::unordered_map<std::string, std::pair<std::string, std::string>>
        platform_map = {{"NX_XG3588", {"192.168.3.120", "192.168.3.100"}},
                        {"XG3588", {"127.0.0.1", "127.0.0.1"}}};

    auto it = platform_map.find(platform_);
    if (it != platform_map.end()) {
      SERVER_IP = it->second.first;
      CLIENT_IP = it->second.second;
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31mError: Unknown PLATFORM type\033[0m");
      rclcpp::shutdown();
      return;
    }

    // Port configuration
    CLIENT_PORT = 43909;
    SERVER_PORT = 43998;

    // Connector initialization
    connector_.creat_connector(SERVER_IP, CLIENT_IP, SERVER_PORT, CLIENT_PORT);
    if (not connector_.reg_state_recive()) {
      RCLCPP_ERROR(this->get_logger(), "==> Init  HighLevelConnector error!");
      rclcpp::shutdown();
      return;
    }

    // UDP thread
    udp_process_thread_ = std::thread([&]() {
      while (rclcpp::ok()) {
        std::unique_lock lk(planner_vel_mutex_);
        cmd_.len = sizeof(navigo_sdk::highLevelCmd);
        cmd_.head = (uint16_t)0x5AA5;  // 获取控制权
        cmd_.checksum =
            connector_.checksum(reinterpret_cast<const unsigned char*>(&cmd_));
        //    		    RCLCPP_INFO(this->get_logger(), "==> 最终发送了:
        //    %f",cmd_.vx);
        connector_.SendCmd(&cmd_);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });
  }

  ~VelWithMcTrajecotryCmdUdpPublisher() {
    if (udp_process_thread_.joinable()) {
      udp_process_thread_.join();
    }
  }

 private:
  void HandlPlannerVelWithMcTrajectoryCallback(
      const robots_dog_msgs::msg::CmdVelWithTrajectory::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(planner_vel_mutex_);

    cmd_.vx = std::fabs(msg->cmd_vel.twist.linear.x) < 0.085
                  ? 0.0
                  : msg->cmd_vel.twist.linear.x;
    cmd_.vy = std::fabs(msg->cmd_vel.twist.linear.y) < 0.085
                  ? 0.0
                  : msg->cmd_vel.twist.linear.y;
    cmd_.yaw_rate = msg->cmd_vel.twist.angular.z;

    if (msg->trajectory.poses.size() != std::size(cmd_.trajectory)) {
      RCLCPP_ERROR(this->get_logger(),
                   "==> Received  traj size is not equal to mc reuqired "
                   "trajectory size");
      RCLCPP_ERROR(this->get_logger(),
                   "Received traj size : %ld | Mc trajectory : %ld",
                   msg->trajectory.poses.size(), std::size(cmd_.trajectory));
      return;
    }

    for (size_t i = 0; i < std::size(cmd_.trajectory); ++i) {
      cmd_.trajectory[i].x = msg->trajectory.poses[i].pose.position.x;
      cmd_.trajectory[i].y = msg->trajectory.poses[i].pose.position.y;

      tf2::Quaternion q(msg->trajectory.poses[i].pose.orientation.x,
                        msg->trajectory.poses[i].pose.orientation.y,
                        msg->trajectory.poses[i].pose.orientation.z,
                        msg->trajectory.poses[i].pose.orientation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      cmd_.trajectory[i].theta = yaw;
    }
  }

 private:
  int CLIENT_PORT;
  int SERVER_PORT;
  std::string SERVER_IP;
  std::string CLIENT_IP;
  std::string platform_;
  std::mutex planner_vel_mutex_;
  rclcpp::Subscription<robots_dog_msgs::msg::CmdVelWithTrajectory>::SharedPtr
      planner_vel_with_mc_traj_cmd_subscriber;

  navigo_sdk::HighLevelConnector connector_;
  navigo_sdk::highLevelCmd cmd_;

  std::thread udp_process_thread_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<VelWithMcTrajecotryCmdUdpPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
