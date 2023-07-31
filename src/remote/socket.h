/**
 * @file socket.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 网络通讯,通过udp接收控制指令
 * @version 0.1
 * @date 2023-01-14
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */
#ifndef __SOCKET_H__
#define __SOCKET_H__

#include "RemoteFactory.h"
#include <iostream>

#define MAXLINE 80
#define SERV_PORT 5555

class UdpSocket : public RemoteProduct
{
public:
    UdpSocket(RemoteConfig_t config, bool debug = false);
    ~UdpSocket();

    int Init();
    virtual bool Request(struct RemoteState &data);

private:
    int GetClientData();
    int socket_fd_;
    RemoteState rc_data_;
};

// 生产udp遥控工厂
class UdpRemote : public RemoteFactory
{
public:
    RemoteProduct *CreateRemoteProduct(RemoteConfig_t config, bool debug)
    {
        return new UdpSocket(config, debug);
    }
};

#endif
