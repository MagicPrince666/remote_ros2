/**
 * @file remote.h
 * @author 黄李全 (846863428@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-01-13
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */
#ifndef __REMOTE_H__
#define __REMOTE_H__

#include <iostream>
#include <stdint.h>

struct RemoteState {
    bool lose_signal = false;   // 失控标识

    bool front         = false; // 前进按钮
    bool back          = false; // 后退按钮
    bool left          = false; // 左转按钮
    bool right         = false; // 右转按钮
    bool select        = false; // 选择按钮
    bool start         = false; // 开始按钮
    bool l1            = false; // 左顶部按钮1
    bool l2            = false; // 左顶部按钮2
    bool r1            = false; // 右顶部按钮1
    bool r2            = false; // 右顶部按钮2
    bool adl           = false; // 左摇杆按钮
    bool adr           = false; // 右摇杆按钮
    bool triangle      = false; // 三角按钮
    bool quadrilateral = false; // 四边形按钮
    bool rotundity     = false; // 园形按钮
    bool fork          = false; // 叉按钮
    float adslx        = 0.5;   // 左摇杆x轴
    float adsly        = 0.5;   // 左摇杆y轴
    float adsrx        = 0.5;   // 右摇杆x轴
    float adsry        = 0.5;   // 右摇杆y轴
    float ads[12]    = {0};   // 扩展通道 sbus 16路通道都是模拟量
};

typedef struct {
    std::string type    = "sbus";
    std::string port    = "/dev/ttyUSB0";
    int32_t baudrate    = 100000;
    int32_t data_len    = 25;
    int32_t joy_var_max = 1800;
    int32_t joy_var_min = 200;
    double max_x_vel    = 1.0;
    double max_w_vel    = 1.0;
} RemoteConfig_t;

#endif
