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

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include "RemoteFactory.h"

class RemotePub : public rclcpp::Node
{
public:
    RemotePub();
    ~RemotePub();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_pub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;

    RemoteConfig_t config_;
    std::shared_ptr<RemoteProduct> remote_;

    void LoopCallback();
};

#endif
