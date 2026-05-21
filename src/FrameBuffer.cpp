#include "FrameBuffer.h"
#include <iostream>
#include <algorithm>  // 确保std::copy/std::fill可用
#include <cassert>    // 可选：用于调试期指针校验
#include <fstream>
#include <cstdint>
#include <iomanip>

// ========== FrameBuffer 实现 ==========
FrameBuffer::FrameBuffer(size_t frame_total_size) 
    : frame_total_size_(frame_total_size)
    //   receivedLen_(0),
    //   frameReady_(false)
    //   latestFrameValid_(false)  // 显式初始化，避免未定义行为
{
    // frameBuf_ = std::vector<uint8_t>(frame_total_size, 0);
    // latestFrameBuf_ = std::vector<uint8_t>(frame_total_size, 0);
    // assert(frameBuf_.size() == frame_total_size_);
    // assert(latestFrameBuf_.size() == frame_total_size_);
}

void FrameBuffer::writeChunk(const uint8_t* data, size_t len, bool collectEn) {
    if (!collectEn || !data || len != frame_total_size_) {
        std::cerr << "[writeChunk] 无效参数：collectEn=" << collectEn 
                  << " len=" << len << std::endl;
        return;
    }

    std::vector<uint8_t> frame_data;
    frame_data.resize(frame_total_size_); // 预分配内存
    memcpy(frame_data.data(), data, frame_total_size_); // 拷贝数据

    // 入队
    {
        std::lock_guard<std::mutex> lock(frameMtx_);
        if (decode_queue_.size() < queue_size_) {
            // decode_queue_.push(data);
            decode_queue_.push(std::move(frame_data));
           // std::cout << "队列NUM: " << decode_queue_.size() <<std::endl;
            
            // // std::ios::app：追加写入；std::ios::out：输出模式；std::ios::binary：避免换行符转换（可选）
            // std::ofstream file("data.txt", std::ios::out | std::ios::app);
            // if (!file.is_open()) {
            //     throw std::runtime_error("无法打开文件!");
            // }

            // // 3. 配置16进制输出格式
            // file << std::hex; // 切换为16进制输出
            // file << std::setfill('0'); // 不足两位时补0（如0x1 → 01）

            // // 4. 逐字节写入16进制数据
            // for (size_t i = 0; i < len; ++i) {
            //     // 每个字节输出两位16进制（如uint8_t(5) → "05"）
            //     file << std::setw(2) << static_cast<int>(data[i]);
                
            //     // 控制每行显示个数，添加空格分隔
            //     if ((i + 1) % 64 == 0) {
            //         file << "\n"; // 换行
            //     } else {
            //         file << " "; // 字节间加空格，便于阅读
            //     }
            // }

            // // 5. 最后一行补换行（避免后续追加粘行）
            // if (len % 128 != 0) {
            //     file << "\n";
            // }

            // // 6. 刷新并关闭文件（ofstream析构时会自动关闭，此处显式关闭更安全）
            // file.flush();
            // file.close();

            frameCv_.notify_one();
        } else {
            // 队列满，丢弃数据
            std::cout << "解码队列已满，解码速度小于接收速度！！！" << std::endl;
            // dropped_packets_++;
        }
    }

}

bool FrameBuffer::readFrame(uint8_t* outBuf, size_t& outLen, std::atomic<bool>& collectEn, std::atomic<bool>& appRunning) {
    // std::cout << "[readFrame] ..." << std::endl;
    // 1. 输出缓冲区合法性校验
    if (!outBuf || outLen < frame_total_size_) {
        std::cerr << "[readFrame] 输出缓冲区无效：outBuf=" << (void*)outBuf 
                  << " outLen=" << outLen << " 需至少：" << frame_total_size_ << std::endl;
        return false;
    }

    {
        std::unique_lock<std::mutex> lock(frameMtx_);
        // std::lock_guard<std::mutex> lock(frameMtx_);

        // 1. 检查条件（通过predicate函数）
        // 2. 如果条件不满足：释放锁并进入睡眠
        // 3. 被唤醒后：重新获取锁并再次检查条件
        // 4. 返回 true：停止等待，线程继续执行 | 返回 false：继续等待，线程保持睡眠
        frameCv_.wait(lock, [this]() {
            return !decode_queue_.empty();
        });
        // std::cout << "[readFrame] frameCv_..." << std::endl;


        // 3. 此时已持有锁，且条件已满足
        // 3. 退出条件判断（原子变量需显式load）
        if (!collectEn.load() || !appRunning.load()) {
            std::cout << "[readFrame] 采集/程序停止，退出读取" << std::endl;
            return false;
        }

        if (!decode_queue_.empty()) {
            // 4. 正确拷贝：frameBuf_ → outBuf
            // std::copy(decode_queue_.front().begin(), decode_queue_.front().end(), outBuf);
            

            const auto& frame_data = decode_queue_.front();
            memcpy(outBuf, frame_data.data(), frame_data.size()); // 拷贝自主管理的副本
            // memcpy(outBuf, decode_queue_.front(), frame_total_size_);
            // auto* ptr_t = decode_queue_.front();
            decode_queue_.pop();
            // delete ptr_t;
            // std::cout << "decode_queue_: " << decode_queue_.size() << std::endl;
            outLen = frame_total_size_;
            // outLen = frame_data.size();
            return true;
        }    

    }
    return true;
}