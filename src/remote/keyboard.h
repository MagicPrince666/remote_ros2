/**
 * @file keyboard.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 蓝牙mini手柄
 * @version 0.1
 * @date 2023-01-14
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */
#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#include "RemoteFactory.h"
#include <iostream>
#include <mutex>

class KeyBoard : public RemoteProduct
{
public:
    KeyBoard(RemoteConfig_t config, bool debug = false);
    ~KeyBoard();

    int Init();
    virtual bool Request(struct RemoteState &data);

private:
    int keyboard_fd_;
    struct RemoteState rc_data_;
    std::mutex  rc_data_lock_;

    int GetKeyBoard();
};

// 生产键盘遥控工厂
class KeyBoardRemote : public RemoteFactory
{
public:
    RemoteProduct *CreateRemoteProduct(RemoteConfig_t config, bool debug)
    {
        return new KeyBoard(config, debug);
    }
};

#endif
