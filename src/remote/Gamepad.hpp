/**
 * @file Gamepad.hpp
 * @author 黄李全 (846863428@qq.com)
 * @brief 通用joystick数据获取
 * @version 0.1
 * @date 2023-01-08
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */

#include "RemoteFactory.h"
#include <string>
#include <thread>
#include <mutex>

class Gamepad : public RemoteProduct
{
public:
    Gamepad(RemoteConfig_t config, bool debug = false);
    ~Gamepad();

    int Init();

    virtual bool Request(struct RemoteState &data);

private:
    int ReadJoystick();

    int js_fd_;
    struct RemoteState rc_data_;
    std::mutex  rc_data_lock_;
};

// 生产game pad 遥控工厂
class GamePadRemote : public RemoteFactory
{
public:
    RemoteProduct *CreateRemoteProduct(RemoteConfig_t config, bool debug)
    {
        return new Gamepad(config, debug);
    }
};
