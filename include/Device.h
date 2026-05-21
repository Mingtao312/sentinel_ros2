#pragma once

#include <libusb-1.0/libusb.h>
#include <thread>
#include "FrameBuffer.h"
#include <atomic>

const int TRANSFER_QUEUE_SIZE = 1024;

class Device {
public:
    // 无参构造：使用所有默认值
    // Device() = default;  // 显式默认构造（C++11）

    // 带参构造：覆盖部分默认值（如 VID/PID/端点）
    Device(FrameBuffer& frameBuffer, uint16_t vid=0x04b4, uint16_t pid=0x00f1, uint8_t out_ep = 0x01, uint8_t in_ep = 0x81);
    ~Device();

    bool SyncRecvData(unsigned char* buf, int max_len, int& actual_len);
    bool SyncSendData(const unsigned char* data, int len);
    // UI控制：开始采集
    void StartCollect();
    // UI控制：停止采集
    void StopCollect();
    // 退出USB设备（程序退出时调用）
    void ExitDevice();

    std::atomic<bool>& IsCollecting() {return is_collecting_;};
    std::atomic<bool>& IsAppRunning() {return is_app_running_;};

    std::thread* thread_ = nullptr; 
private:
    // 成员变量：带默认值（C++11 直接类内赋值）
    FrameBuffer& frameBuffer_; 
                    // USB事件循环线程
    std::atomic<bool> is_app_running_{false};       // 程序运行标志
    std::atomic<bool> is_collecting_{false};        // 采集使能标志
    libusb_context* ctx_ = nullptr;                 // 指针默认空指针
    libusb_device_handle* dev_handle_ = nullptr;    // 设备句柄默认空指针
    uint16_t vid_ = 0x04b4;                  // CYUSB3014 默认 VID
    uint16_t pid_ = 0x00f1;                  // CYUSB3014 默认 PID
    uint8_t out_endpoint_ = 0x01;            // 默认 OUT 端点
    uint8_t in_endpoint_ = 0x81;             // 默认 IN 端点
    int timeout_ = 10000;                    // 默认超时10ms
    int max_packet_size_ = 1024;                // 默认最大包长1024字节

    std::atomic<uint64_t> total_recv_bytes_{0};  // 累计接收字节数
    std::atomic<uint32_t> transfer_count_{0};    // 累计完成传输次数
    std::atomic<uint32_t> error_count_{0};       // 累计传输错误数
    std::chrono::steady_clock::time_point speed_start_time_;  // 速率统计起始时间
    std::mutex speed_mtx_;                        // 速率统计锁
    const uint32_t SPEED_PRINT_INTERVAL = 1;      // 每1秒打印一次速率

    
    libusb_transfer* transfers_[TRANSFER_QUEUE_SIZE] = {nullptr};
    uint8_t* transfer_bufs_[TRANSFER_QUEUE_SIZE] = {nullptr};
    std::atomic<uint64_t> packet_seq_{0}; // 数据包序列号（验证顺序）

    void reset_speed_stats();

    int interface_num_ = 0;

    bool init_device();
    void parse_config_descriptor();
    bool claim_interface();
    uint16_t get_packet_size(uint16_t le_val);
    // USB事件循环线程函数
    void device_transfer();
    // USB传输回调函数
    static void LIBUSB_CALL transfer_callback(libusb_transfer* tr);
    
    bool need_clear_ = true;

    libusb_transfer* transfer_ = nullptr;     // 异步传输对象
    uint8_t* transferBuf_ = nullptr;          // 1KB传输缓冲区
  
};
