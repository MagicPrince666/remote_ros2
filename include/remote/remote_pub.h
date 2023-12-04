/**
 * @file remote_pub.h
 * @author 黄李全 (846863428@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-07-17
 * @copyright 个人版权所有 Copyright (c) 2023
 */

#ifndef __REMOTE_PUB_H__
#define __REMOTE_PUB_H__

#include <memory>
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#include "RemoteFactory.h"

class RemotePub
{
public:
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    RemotePub(std::shared_ptr<ros::NodeHandle> node);
#else
    RemotePub(std::shared_ptr<rclcpp::Node> node);
#endif
    ~RemotePub();

private:
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    using TwistMsg = geometry_msgs::Twist;
    std::shared_ptr<ros::Publisher> remote_pub_;
    std::shared_ptr<ros::NodeHandle> ros_node_;
#else
    using TwistMsg = geometry_msgs::msg::Twist;
    rclcpp::Publisher<TwistMsg>::SharedPtr remote_pub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    std::shared_ptr<rclcpp::Node> ros_node_;
#endif

    RemoteConfig_t config_;
    std::shared_ptr<RemoteProduct> remote_;

    void LoopCallback();
};

#endif
