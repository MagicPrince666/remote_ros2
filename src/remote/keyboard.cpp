#include <assert.h>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

#include "keyboard.h"
#include "xepoll.h"

KeyBoard::KeyBoard(RemoteConfig_t config, bool debug) : RemoteProduct(config, debug)
{
    keyboard_fd_ = -1;
    Init();
    assert(keyboard_fd_ > 0);
}

KeyBoard::~KeyBoard()
{
    if (keyboard_fd_ > 0) {
        MY_EPOLL.EpollDel(keyboard_fd_);
        close(keyboard_fd_);
    }
}

int KeyBoard::Init()
{
    if ((access(config_.port.c_str(), F_OK)) == -1) {
        std::cerr << "No such file : " << config_.port << std::endl;
        return -1;
    }
    std::cout << "Input device: " << config_.port << std::endl;
    keyboard_fd_ = open(config_.port.c_str(), O_RDONLY);
    if (keyboard_fd_ == -1) {
        std::cerr << "Could not open device\n";
        return -1;
    } else {
        char buf[256] = {0};
        ioctl(keyboard_fd_, EVIOCGNAME(sizeof(buf)), buf);
        std::string dev(buf);
        std::cout << "Device info: " << dev << std::endl;
    }

    if (keyboard_fd_ > 0) {
        std::cout << "Keybord bind epoll" << std::endl;
        MY_EPOLL.EpollAddRead(keyboard_fd_, std::bind(&KeyBoard::GetKeyBoard, this));
    }
    return 0;
}

int KeyBoard::GetKeyBoard()
{
    struct input_event event;
    int ret = read(keyboard_fd_, &event, sizeof(event));
    if (ret > 0) {
        if (debug_) {
            printf("Type = %d\tCode = %d\tValue = %d\n", event.type, event.code, event.value);
        }
        std::lock_guard<std::mutex> mylock_guard(rc_data_lock_);
        rc_data_.lose_signal = false;
        switch (event.type) {
        case 0x03:
            switch (event.code) {
            case 0x01:
                rc_data_.adsrx = (float)(255 - event.value) / 255.0; // 摇杆x轴
                break;
            case 0x00:
                rc_data_.adsry = (float)event.value / 255.0; // 摇杆y轴
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
    }
    return ret;
}

bool KeyBoard::Request(struct RemoteState &data)
{
    if (rc_data_.lose_signal) {
        return false;
    }
    std::lock_guard<std::mutex> mylock_guard(rc_data_lock_);
    data = rc_data_;
    return true;
}