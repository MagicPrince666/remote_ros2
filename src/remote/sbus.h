/**
 * @file sbus.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 遥控器SBUS接收解码程序
 * @version 0.1
 * @date 2023-01-09
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */
#ifndef __SBUS_H__
#define __SBUS_H__

#include "RemoteFactory.h"
#include <iostream>
#include <stdint.h>
#include <mutex>

typedef struct {
    uint16_t rawdata[16];      // 根据sbus协议，一帧包含16通道，每通道11位（0-2047）
    float percent[16];       // 我的天地飞遥控器实际rawdata范围是340-1704，中间值1024，这里还原遥控器的百分比
    bool flag_refresh = false; // 解析代码每次成功解析数据都会将此变量设为1，选择使用
    bool lost_signed = true; // 失控标识
} RcData_t;

#pragma pack(push)
#pragma pack(1)
typedef struct {
    uint8_t head;       // sbus协议头 0x0F
    uint8_t data[22];   // 我的天地飞遥控器实际rawdata范围是340-1704，中间值1024
    uint8_t flags;      // flags=1：控制器与接收器保持连接； flags=0：控制器与接收器断开（失控）
    uint8_t endbyte;    // sbus协议尾 0x00
} sbus_t;
#pragma pack(pop)

class Sbus : public RemoteProduct
{
private:
    int sbus_fd_;
    RcData_t rc_data_;
    std::mutex data_lock_;

    struct {
        uint8_t rx_buffer[256];
        uint32_t size; // buf长度
    } uart_rx_buffer_;

    int SerialSetSpeciBaud(int baud);
    int OpenSerial(std::string SerialName, int Bitrate);
    int GetData();

    void SbusDecoderGetFrame(sbus_t *buf);

    sbus_t *SearchSbusHear(uint8_t *data, uint32_t total_len, int &index);

public:
    Sbus(RemoteConfig_t config, bool debug = false);
    ~Sbus();

    bool Request(struct RemoteState &data);
    bool Init();
};

// 生产sbus 遥控工厂
class SbusRemote : public RemoteFactory
{
public:
    RemoteProduct *CreateRemoteProduct(RemoteConfig_t config, bool debug)
    {
        return new Sbus(config, debug);
    }
};

#endif
