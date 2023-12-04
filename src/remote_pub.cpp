#include <assert.h>
#include <atomic>
#include <csignal>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "remote/remote_pub.h"

#include "Gamepad.hpp"
#include "keyboard.h"
#include "sbus.h"
#include "socket.h"
#include "sonnyps2.h"
#include "xepoll.h"

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
RemotePub::RemotePub(std::shared_ptr<ros::NodeHandle> node)
#else
RemotePub::RemotePub(std::shared_ptr<rclcpp::Node> node)
#endif
: ros_node_(node)
{
    ros_node_->declare_parameter("type", "");
    ros_node_->get_parameter("type", config_.type);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "type = %s", config_.type.c_str());

    ros_node_->declare_parameter("port", "/dev/ttyUSB0");
    ros_node_->get_parameter("port", config_.port);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "port = %s", config_.port.c_str());

    ros_node_->declare_parameter("baudrate", 100000);
    ros_node_->get_parameter("baudrate", config_.baudrate);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "baudrate = %d", config_.baudrate);

    ros_node_->declare_parameter("data_len", 25);
    ros_node_->get_parameter("data_len", config_.data_len);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "data_len = %d", config_.data_len);

    ros_node_->declare_parameter("joy_var_max", 1800);
    ros_node_->get_parameter("joy_var_max", config_.joy_var_max);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "joy_var_max = %d", config_.joy_var_max);

    ros_node_->declare_parameter("joy_var_min", 200);
    ros_node_->get_parameter("joy_var_min", config_.joy_var_min);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "joy_var_min = %d", config_.joy_var_min);

    ros_node_->declare_parameter("max_x_vel", 1.0);
    ros_node_->get_parameter("max_x_vel", config_.max_x_vel);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "max_x_vel = %f", config_.max_x_vel);

    ros_node_->declare_parameter("max_w_vel", 1.0);
    ros_node_->get_parameter("max_w_vel", config_.max_w_vel);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "max_w_vel = %f", config_.max_w_vel);

    if (config_.type == "sbus") {
        // 创建遥控工厂
        std::unique_ptr<RemoteFactory> factory(new SbusRemote());
        // 通过工厂方法创建sbus遥控产品
        std::shared_ptr<RemoteProduct> sbus(factory->CreateRemoteProduct(config_, false));
        remote_ = sbus;
    } else if (config_.type == "gamepad") {
        std::unique_ptr<RemoteFactory> factory(new GamePadRemote());
        std::shared_ptr<RemoteProduct> gamepad(factory->CreateRemoteProduct(config_, false));
        remote_ = gamepad;
    } else if (config_.type == "keyboard") {
        // 创建遥控工厂
        std::unique_ptr<RemoteFactory> factory(new KeyBoardRemote());
        // 通过工厂方法创建键盘遥控产品
        std::shared_ptr<RemoteProduct> key(factory->CreateRemoteProduct(config_, false));
        remote_ = key;
    } else if (config_.type == "socket") {
        std::unique_ptr<RemoteFactory> factory(new UdpRemote());
        std::shared_ptr<RemoteProduct> udp_server(factory->CreateRemoteProduct(config_, false));
        remote_ = udp_server;
    } else if (config_.type == "sonnyps2") {
        std::unique_ptr<RemoteFactory> factory(new SonnyRemote());
        std::shared_ptr<RemoteProduct> ps2(factory->CreateRemoteProduct(config_, false));
        remote_ = ps2;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(__FUNCTION__), "please use an avlable remote");
    }

    remote_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    loop_timer_ = ros_node_->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&RemotePub::LoopCallback, this));
}

RemotePub::~RemotePub()
{
}

void RemotePub::LoopCallback()
{
    RemoteState rc_data;
    if (remote_) {
        remote_->Request(rc_data);
    } else {
        return;
    }

    if (config_.type == "sbus" && (rc_data.adsrx == 0 && rc_data.adsry == 0)) {
        RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "adsrx = %f\tadsry = %f", rc_data.adsrx, rc_data.adsry);
        return;
    }

    geometry_msgs::msg::Twist msg;
    if (config_.type == "gamepad") {
        msg.linear.x  = (1 - 2.0 * rc_data.adsly) * config_.max_x_vel; // 左摇杆y轴 线速度
        msg.angular.z = (1 - 2.0 * rc_data.adsrx) * config_.max_w_vel; // 左摇杆x轴 角速度
    } else {
        msg.linear.x  = (1 - 2.0 * rc_data.adsry) * config_.max_x_vel; // 左摇杆y轴 线速度
        msg.angular.z = (1 - 2.0 * rc_data.adsrx) * config_.max_w_vel; // 左摇杆x轴 角速度
    }

    if (msg.linear.x > -0.1 && msg.linear.x < 0.1) {
        msg.linear.x = 0.0;
    }

    if (msg.angular.z > -0.1 && msg.angular.z < 0.1) {
        msg.angular.z = 0.0;
    }

    // RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "linear: [%f]\tangular : [%f]", msg.linear.x, msg.angular.z);
    remote_pub_->publish(msg);
}
