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

#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

#include "Gamepad.hpp"
#include "keyboard.h"
#include "sbus.h"
#include "socket.h"
#include "sonnyps2.h"
#include "xepoll.h"

RemotePub::RemotePub() : rclcpp::Node("remote")
{
    this->declare_parameter("type", "");
    this->get_parameter("type", config_.type);
    spdlog::info("type = {}", config_.type);

    this->declare_parameter("port", "/dev/ttyUSB0");
    this->get_parameter("port", config_.port);
    spdlog::info("port = {}", config_.port);

    this->declare_parameter("baudrate", 100000);
    this->get_parameter("baudrate", config_.baudrate);
    spdlog::info("baudrate = {}", config_.baudrate);

    this->declare_parameter("data_len", 25);
    this->get_parameter("data_len", config_.data_len);
    spdlog::info("data_len = {}", config_.data_len);

    this->declare_parameter("joy_var_max", 1800);
    this->get_parameter("joy_var_max", config_.joy_var_max);
    spdlog::info("joy_var_max = {}", config_.joy_var_max);

    this->declare_parameter("joy_var_min", 200);
    this->get_parameter("joy_var_min", config_.joy_var_min);
    spdlog::info("joy_var_min = {}", config_.joy_var_min);

    this->declare_parameter("max_x_vel", 1.0);
    this->get_parameter("max_x_vel", config_.max_x_vel);
    spdlog::info("max_x_vel = {}", config_.max_x_vel);

    this->declare_parameter("max_w_vel", 1.0);
    this->get_parameter("max_w_vel", config_.max_w_vel);
    spdlog::info("max_w_vel = {}", config_.max_w_vel);

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

    remote_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&RemotePub::LoopCallback, this));
}

RemotePub::~RemotePub()
{
}

void RemotePub::LoopCallback()
{
    RemoteState rc_data;
    if (remote_) {
        remote_->Request(rc_data);
    }
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = (1 - 2.0 * rc_data.adsry) * config_.max_x_vel; // 右摇杆y轴 线速度
    msg.angular.z = (1 - 2.0 * rc_data.adsrx) * config_.max_w_vel; // 右摇杆x轴 角速度
    spdlog::info("linear: [{}]\tangular : [{}]", msg.linear.x, msg.angular.z);
    remote_pub_->publish(msg);
}