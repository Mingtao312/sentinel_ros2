#pragma once

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <vector>
#include <queue>

const uint8_t HEADER[8] = {0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA};

class FrameBuffer {
public:
    FrameBuffer(size_t frame_total_size);
    ~FrameBuffer() = default;

    // 写入分块数据（USB接收线程调用）
    void writeChunk(const uint8_t* data, size_t len, bool collectEn);
    // 读取完整帧（解包线程调用）
    bool readFrame(uint8_t* outBuf, size_t& outLen, std::atomic<bool>& collectEn, std::atomic<bool>& appRunning);
    // // 更新最新帧缓存（解包线程调用）
    // void updateLatestFrame(const uint8_t* frameData);
    // // 获取最新帧（外部调用接口）
    // bool getLatestFrame(uint8_t* outBuf, size_t& outLen);
    // bool getRGBLeft();
    // bool getRGBRight();
    // bool getTDLeft();
    // bool getTDRight();
    // bool getSDLLeft();
    // bool getSDLRight();
    // bool getSDRLeft();
    // bool getSDRRight();
private:
    std::queue<std::vector<uint8_t>> decode_queue_;
    // std::queue<const uint8_t*> decode_queue_;
    // std::queue<std::vector<uint8_t>> decode_queue_; 

    size_t queue_size_ = 1000;

    size_t frame_total_size_;
    // 帧拼接缓冲区
    std::mutex frameMtx_;
    std::condition_variable frameCv_;
};
