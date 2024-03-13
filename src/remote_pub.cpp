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

#include <spdlog/spdlog.h>
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
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros_node_->getParam("remote_node/type", config_.type);
    spdlog::info("type = {}", config_.type.c_str());

    ros_node_->getParam("remote_node/port", config_.port);
    spdlog::info("port = {}", config_.port.c_str());

    ros_node_->getParam("remote_node/baudrate", config_.baudrate);
    spdlog::info("baudrate = {}", config_.baudrate);

    ros_node_->getParam("remote_node/data_len", config_.data_len);
    spdlog::info("data_len = {}", config_.data_len);

    ros_node_->getParam("remote_node/joy_var_max", config_.joy_var_max);
    spdlog::info("joy_var_max = {}", config_.joy_var_max);

    ros_node_->getParam("remote_node/joy_var_min", config_.joy_var_min);
    spdlog::info("joy_var_min = {}", config_.joy_var_min);

    ros_node_->getParam("remote_node/max_x_vel", config_.max_x_vel);
    spdlog::info("max_x_vel = {}", config_.max_x_vel);

    ros_node_->getParam("remote_node/max_w_vel", config_.max_w_vel);
    spdlog::info("max_w_vel = {}", config_.max_w_vel);
#else
    ros_node_->declare_parameter("type", "");
    ros_node_->get_parameter("type", config_.type);
    spdlog::info("type = {}", config_.type.c_str());

    ros_node_->declare_parameter("port", "/dev/ttyUSB0");
    ros_node_->get_parameter("port", config_.port);
    spdlog::info("port = {}", config_.port.c_str());

    ros_node_->declare_parameter("baudrate", 100000);
    ros_node_->get_parameter("baudrate", config_.baudrate);
    spdlog::info("baudrate = {}", config_.baudrate);

    ros_node_->declare_parameter("data_len", 25);
    ros_node_->get_parameter("data_len", config_.data_len);
    spdlog::info("data_len = {}", config_.data_len);

    ros_node_->declare_parameter("joy_var_max", 1800);
    ros_node_->get_parameter("joy_var_max", config_.joy_var_max);
    spdlog::info("joy_var_max = {}", config_.joy_var_max);

    ros_node_->declare_parameter("joy_var_min", 200);
    ros_node_->get_parameter("joy_var_min", config_.joy_var_min);
    spdlog::info("joy_var_min = {}", config_.joy_var_min);

    ros_node_->declare_parameter("max_x_vel", 1.0);
    ros_node_->get_parameter("max_x_vel", config_.max_x_vel);
    spdlog::info("max_x_vel = {}", config_.max_x_vel);

    ros_node_->declare_parameter("max_w_vel", 1.0);
    ros_node_->get_parameter("max_w_vel", config_.max_w_vel);
    spdlog::info("max_w_vel = {}", config_.max_w_vel);
#endif

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
        spdlog::error("please use an avlable remote");
    }
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    remote_pub_ = std::make_shared<ros::Publisher>(ros_node_->advertise<TwistMsg>("/cmd_vel", 10));
#else
    remote_pub_ = ros_node_->create_publisher<TwistMsg>("/cmd_vel", 10);

    loop_timer_ = ros_node_->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&RemotePub::LoopCallback, this));
#endif
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

    if ((rc_data.adsrx == 0 && rc_data.adsry == 0)) {
        spdlog::info("adsrx = {}\tadsry = {}", rc_data.adsrx, rc_data.adsry);
        return;
    }

    TwistMsg msg;
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

    // spdlog::info("linear: [{}]\tangular : [{}]", msg.linear.x, msg.angular.z);
    remote_pub_->publish(msg);
}
