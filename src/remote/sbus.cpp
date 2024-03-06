#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#define termios asmtermios
#include <asm-generic/ioctls.h>
#include <asm-generic/termbits.h>
#undef termios
#include <linux/serial.h>
#include <termios.h>

#include "sbus.h"
#include "xepoll.h"

Sbus::Sbus(RemoteConfig_t config, bool debug) : RemoteProduct(config, debug)
{
    std::cout << "Sbus init with " << config_.port << std::endl;
    sbus_fd_ = -1;
    Init();
}

Sbus::~Sbus()
{
    std::cout << "Sbus deinit" << std::endl;

    if (sbus_fd_ > 0) {
        MY_EPOLL.EpollDel(sbus_fd_);
        close(sbus_fd_);
    }
}

bool Sbus::Init()
{
    sbus_fd_ = OpenSerial(config_.port, config_.baudrate);
    if (sbus_fd_ < 0) {
        std::cout << "can\'t open " << config_.port << " !" << std::endl;
        return false;
    }
    assert(sbus_fd_ > 0);
    memset((int8_t *)&rc_data_, 0, sizeof(rc_data_));
    MY_EPOLL.EpollAddRead(sbus_fd_, std::bind(&Sbus::GetData, this));
    return true;
}

int Sbus::SerialSetSpeciBaud(int baud)
{
    struct termios2 tio;
    memset((int8_t *)&tio, 0, sizeof(tio));
    tio.c_cflag  = BOTHER | CS8 | CLOCAL | CREAD;
    tio.c_iflag  = IGNPAR;
    tio.c_oflag  = 0;
    tio.c_ispeed = baud;
    tio.c_ospeed = baud;
    return ioctl(sbus_fd_, TCSETS2, &tio);
}

int Sbus::OpenSerial(std::string SerialName, int Bitrate)
{
    /**
     * open the serial port
     */
    int fd = open(SerialName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == fd) {
        printf("Open SBUS input %s failed, status %d \n", SerialName.c_str(),
               (int)fd);
        fflush(stdout);
        return -1;
    }
    struct termios2 tio {
    };
    if (0 != ioctl(fd, TCGETS2, &tio)) {
        close(fd);
        return -1;
    }
    /**
     * Setting serial port,8E2, non-blocking.100Kbps
     */
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tio.c_iflag |= (INPCK | IGNPAR);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
    /**
     * use BOTHER to specify speed directly in c_[io]speed member
     */
    tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
    tio.c_ispeed    = Bitrate;
    tio.c_ospeed    = Bitrate;
    tio.c_cc[VMIN]  = config_.data_len;
    tio.c_cc[VTIME] = 0;
    if (0 != ioctl(fd, TCSETS2, &tio)) {
        close(fd);
        return -1;
    }
    sbus_fd_ = fd;
    return sbus_fd_;
}

int Sbus::GetData()
{
    uint8_t sbus_buf[100];
    int len = read(sbus_fd_, sbus_buf, sizeof(sbus_buf));
    if (len > 0) {
        int buf_size = sizeof(uart_rx_buffer_.rx_buffer) - uart_rx_buffer_.size;
        if (buf_size < len) {
            uart_rx_buffer_.size = 0;
        }
        memcpy(uart_rx_buffer_.rx_buffer + uart_rx_buffer_.size, sbus_buf, len);
        uart_rx_buffer_.size += len; // 更新buff长度

        uint32_t loop_times = uart_rx_buffer_.size / sizeof(sbus_t);
        for (uint32_t i = 0; i < loop_times; i++) {
            int index                  = 0;
            uint8_t *sbus_rx_buffer_ptr = uart_rx_buffer_.rx_buffer;
            sbus_t *res_tmp = SearchSbusHear(sbus_rx_buffer_ptr, uart_rx_buffer_.size, index);

            if (res_tmp != nullptr) {
                // 有数据包需要处理
                if (index) {
                    sbus_rx_buffer_ptr += index;
                    uart_rx_buffer_.size -= index;
                }

                // 解码一帧数据
                SbusDecoderGetFrame(res_tmp);
                uart_rx_buffer_.size -= sizeof(sbus_t);
                uint8_t buffer[256];
                // 剩余未处理数据拷贝到临时变量
                memcpy(buffer, sbus_rx_buffer_ptr + sizeof(sbus_t), uart_rx_buffer_.size);
                // 覆盖掉原来的buff
                memcpy(uart_rx_buffer_.rx_buffer, buffer, uart_rx_buffer_.size);

                if (debug_) {
                    for (int i = 0; i < 16; i++) {
                        printf("%02d ", rc_data_.rawdata[i]);
                    }
                    printf("\n");
                }
            } else {
                break;
            }
        }
    }

    return len;
}

sbus_t *Sbus::SearchSbusHear(uint8_t *data, uint32_t total_len, int &index)
{
    sbus_t *res_tmp = nullptr;
    for (uint32_t i = 0; i < total_len; i++) {
        // 剩余长度大于一个包长度，说明还有协议数据包可以处理
        if ((total_len - i) >= 25) {
            // 一个字节一个字节的偏移，直到查找到协议头
            res_tmp = (sbus_t *)(data + i);
            // 找到协议头
            if ((res_tmp->head == 0x0F) && (res_tmp->endbyte == 0x00)) {
                index = i; // 记录偏移地址
                break;
            }
        } else {
            return nullptr;
        }
    }
    return res_tmp;
}

bool Sbus::Request(struct RemoteState &data)
{
    memset((int8_t *)&data, 0, sizeof(data));
    std::lock_guard<std::mutex> mylock_guard(data_lock_);
    if (!rc_data_.flag_refresh) {
        data.adslx = 0.5;
        data.adsly = 0.5;
        data.adsrx = 0.5;
        data.adsry = 0.5;
        return false;
    }
    data.lose_signal = rc_data_.lost_signed; // 失控标识
    data.adslx       = rc_data_.percent[3];   // 左摇杆x轴
    data.adsly       = rc_data_.percent[2];   // 左摇杆y轴
    data.adsrx       = rc_data_.percent[0];   // 右摇杆x轴
    data.adsry       = rc_data_.percent[1];   // 右摇杆y轴
    data.ads[0]      = rc_data_.percent[4];   // 5通道 遥控C拨杆开关
    data.ads[1]      = rc_data_.percent[5];   // 6通道 遥控左旋钮
    data.ads[2]      = rc_data_.percent[6];   // 7通道 左后方拨杆
    data.ads[3]      = rc_data_.percent[7];   // 8通道 遥控右旋钮
    data.ads[4]      = rc_data_.percent[8];   // 9通道 遥控B拨杆开关
    data.ads[5]      = rc_data_.percent[9];   // 10通道 遥控A拨杆开关
    return true;
}

void Sbus::SbusDecoderGetFrame(sbus_t *buf)
{
    std::lock_guard<std::mutex> mylock_guard(data_lock_);
    rc_data_.rawdata[0]  = ((buf->data[0] | buf->data[1] << 8) & 0x07FF);
    rc_data_.rawdata[1]  = ((buf->data[1] >> 3 | buf->data[2] << 5) & 0x07FF);
    rc_data_.rawdata[2]  = ((buf->data[2] >> 6 | buf->data[3] << 2 | buf->data[4] << 10) & 0x07FF);
    rc_data_.rawdata[3]  = ((buf->data[4] >> 1 | buf->data[5] << 7) & 0x07FF);
    rc_data_.rawdata[4]  = ((buf->data[5] >> 4 | buf->data[6] << 4) & 0x07FF);
    rc_data_.rawdata[5]  = ((buf->data[6] >> 7 | buf->data[7] << 1 | buf->data[8] << 9) & 0x07FF);
    rc_data_.rawdata[6]  = ((buf->data[8] >> 2 | buf->data[9] << 6) & 0x07FF);
    rc_data_.rawdata[7]  = ((buf->data[9] >> 5 | buf->data[10] << 3) & 0x07FF);
    rc_data_.rawdata[8]  = ((buf->data[11] | buf->data[12] << 8) & 0x07FF);
    rc_data_.rawdata[9]  = ((buf->data[12] >> 3 | buf->data[13] << 5) & 0x07FF);
    rc_data_.rawdata[10] = ((buf->data[13] >> 6 | buf->data[14] << 2 | buf->data[15] << 10) & 0x07FF);
    rc_data_.rawdata[11] = ((buf->data[15] >> 1 | buf->data[16] << 7) & 0x07FF);
    rc_data_.rawdata[12] = ((buf->data[16] >> 4 | buf->data[17] << 4) & 0x07FF);
    rc_data_.rawdata[13] = ((buf->data[17] >> 7 | buf->data[18] << 1 | buf->data[19] << 9) & 0x07FF);
    rc_data_.rawdata[14] = ((buf->data[19] >> 2 | buf->data[20] << 6) & 0x07FF);
    rc_data_.rawdata[15] = ((buf->data[20] >> 5 | buf->data[21] << 3) & 0x07FF);

    for (uint8_t i = 0; i < 16; i++) {
        rc_data_.percent[i] = (float)(rc_data_.rawdata[i] - config_.joy_var_min) / (config_.joy_var_max - config_.joy_var_min);
    }
    rc_data_.flag_refresh = true;
    rc_data_.lost_signed = (buf->flags & 0x04); // (flags & 0x40)失控标识 (flags & 0x80) 失控保护
}
