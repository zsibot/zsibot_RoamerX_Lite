/**
 * @file mc_tf.h
 * @brief
 * @author 
 * @version 1.0
 * @date 
 * @copyright 
 */
#pragma once

#include "base_tf.h"
namespace zsibot_tf
{
    class MCTF : public BaseTF
    {
    public:
        explicit MCTF(const rclcpp::Node::WeakPtr& node);

        ~MCTF() = default;

    private:
        bool InitMember();

        bool InitParams();

        void PublishTF(const nav_msgs::msg::Odometry& msg);

    private:
        geometry_msgs::msg::TransformStamped map_odom_tf_;
        geometry_msgs::msg::TransformStamped odom_base_tf_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;
    };
}  // namespace zsibot_tf