#include "Gamepad.hpp"

#include <assert.h>
#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <string.h>
#include <unistd.h>

#include "utils.h"
#include "xepoll.h"

Gamepad::Gamepad(RemoteConfig_t config, bool debug) : RemoteProduct(config, debug)
{
    js_fd_ = -1;
    rc_data_.lose_signal = true;
    Init();
    assert(js_fd_ > 0);
}

Gamepad::~Gamepad()
{
    if (js_fd_ > 0) {
        MY_EPOLL.EpollDel(js_fd_);
        close(js_fd_);
    }
}

int Gamepad::Init()
{
    if ((access(config_.port.c_str(), F_OK)) == -1) {
        std::cerr << "No such file : " << config_.port << std::endl;
        return -1;
    }
    js_fd_ = open(config_.port.c_str(), O_RDONLY);
    if (js_fd_ == -1) {
        std::cerr << "Could not open js0\n";
        return -1;
    } else {
        char buf[256] = {0};
        ioctl(js_fd_, JSIOCGNAME(sizeof(buf)), buf);
        std::string dev(buf);
        std::cout <<"Device info: " << dev << std::endl;
    }

    if (js_fd_ > 0) {
        std::cout << "Joystick bind epoll" << std::endl;
        MY_EPOLL.EpollAddRead(js_fd_, std::bind(&Gamepad::ReadJoystick, this));
    }
    return 0;
}

int Gamepad::ReadJoystick(void)
{
    struct js_event event;
    int ret = read(js_fd_, &event, sizeof(event));
    if (ret > 0) {
        if (debug_) {
            std::cout << "Type = " << (int32_t)event.type << " Number = " << (int32_t)event.number << " Value = " << event.value << std::endl;
        }
        std::lock_guard<std::mutex> mylock_guard(rc_data_lock_);
        rc_data_.lose_signal = false;
        switch (event.type) {
        case JS_EVENT_BUTTON:
            switch (event.number) {
            case 8:
                rc_data_.select = (event.value) ? true : false; // 选择按钮
                break;
            case 9:
                rc_data_.start = (event.value) ? true : false; // 开始按钮
                break;

            case 10:
                rc_data_.adl = (event.value) ? true : false; // 左摇杆按钮
                break;
            case 11:
                rc_data_.adr = (event.value) ? true : false; // 右摇杆按钮
                break;

            case 4:
                rc_data_.l1 = (event.value) ? true : false; // 左顶部按钮1
                break;
            case 6:
                rc_data_.l2 = (event.value) ? true : false; // 左顶部按钮2
                break;
            case 5:
                rc_data_.r1 = (event.value) ? true : false; // 右顶部按钮1
                break;
            case 7:
                rc_data_.r2 = (event.value) ? true : false; // 右顶部按钮2
                break;

            case 0:
                rc_data_.triangle = (event.value) ? true : false; // 三角按钮 1
                break;
            case 3:
                rc_data_.quadrilateral = (event.value) ? true : false; // 四边形按钮 4
                break;
            case 1:
                rc_data_.rotundity = (event.value) ? true : false; // 圆形按钮 2
                break;
            case 2:
                rc_data_.fork = (event.value) ? true : false; // 叉按钮 3
                break;
            default:
                break;
            }
            break;

        case JS_EVENT_AXIS:
            switch (event.number) {
            case 5:
                if (event.value == -32767) {
                    rc_data_.left = true; // 左转按钮
                } else if (event.value == 32767) {
                    rc_data_.right = true; // 右转按钮
                } else {
                    rc_data_.left  = false;
                    rc_data_.right = false;
                }
                break;

            case 6:
                if (event.value == -32767) {
                    rc_data_.front = true; // 前进按钮
                } else if (event.value == 32767) {
                    rc_data_.back = true; // 后退按钮
                } else {
                    rc_data_.front = false;
                    rc_data_.back  = false;
                }
                break;

            case 2:
                rc_data_.adslx = (float)(event.value + 32767) / 65536.0; // 左摇杆x轴
                break;
            case 1:
                rc_data_.adsly = (float)(event.value + 32767) / 65536.0; // 左摇杆y轴
                break;
            case 3:
                rc_data_.adsrx = (float)(event.value + 32767) / 65536.0; // 右摇杆x轴
                break;
            case 4:
                rc_data_.adsry = (float)(event.value + 32767) / 65536.0; // 右摇杆y轴
                break;
            default:
                break;
            }
            break;

        default:
            break;
        }
    } else {
        std::lock_guard<std::mutex> mylock_guard(rc_data_lock_);
        rc_data_.lose_signal = true;
    }
    return ret;
}

bool Gamepad::Request(struct RemoteState &data)
{
    std::lock_guard<std::mutex> mylock_guard(rc_data_lock_);
    data = rc_data_;
    return false;
}
