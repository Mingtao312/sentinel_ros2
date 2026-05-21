#pragma once

#include <atomic>
#include <functional>
#include <type_traits>
#include <iostream>

/**
 * @class PingPongBuffer
 * @brief 无锁双缓冲模板类，用于多线程数据交换
 * 
 * 设计原则：
 * 1. 一个写线程，多个读线程
 * 2. 写操作不阻塞读，读操作不阻塞写
 * 3. 总是有有效数据可供读取
 * 4. 读操作不消耗数据（同一数据可被多次读取）
 * 
 * 工作流程：
 * 写线程更新 → 切换缓冲区 → 读线程读取最新缓冲区
 * 
 * @tparam T 数据类型，必须支持拷贝构造和默认构造
 */
template<typename T>
class PingPongBuffer {
    // 静态断言，确保模板类型满足要求
    static_assert(std::is_copy_constructible<T>::value, 
                  "Type T must be copy constructible");
    static_assert(std::is_default_constructible<T>::value,
                  "Type T must be default constructible");
    
private:
    T buffers_[2];
    std::atomic<T*> latest_buffer_{&buffers_[0]}; 
    std::function<bool(const T&)> validator_{nullptr};
    uint32_t T::* sequence_ptr_{nullptr};
    std::atomic<uint64_t> update_counter_{0};
    
public:
    // ==================== 构造函数 ====================
    // PingPongBuffer() = default;
    PingPongBuffer(std::function<bool(const T&)> validator, 
                   uint32_t T::* seq_ptr = nullptr)
        : validator_(std::move(validator))
        , sequence_ptr_(seq_ptr) {}
    
    // 禁止拷贝和赋值（因为包含原子成员和缓冲区）
    PingPongBuffer(const PingPongBuffer&) = delete;
    PingPongBuffer& operator=(const PingPongBuffer&) = delete;
    
    // 允许移动（如果需要）
    PingPongBuffer(PingPongBuffer&&) = default;
    PingPongBuffer& operator=(PingPongBuffer&&) = default;
    
    // ==================== 生产者接口 ====================
    bool update(const T& new_data) {
        if (validator_ && !validator_(new_data)) {
            return false;  // 数据无效，不更新
        }
        T* write_buffer = getWriteBuffer();
        *write_buffer = new_data;
        markAsLatest(write_buffer);
        update_counter_.fetch_add(1, std::memory_order_relaxed);
        
        return true;
    }
    
    // ==================== 消费者接口 ====================
    T getLatest() const {
        T* latest = latest_buffer_.load(std::memory_order_acquire);
        return *latest;  // 返回拷贝
    }
    
    /**
     * @brief 获取最新数据的引用（线程安全）
     * 
     * 与 getLatest() 类似，但返回引用，避免拷贝开销。
     * 注意：返回的引用只在当前调用瞬间有效。
     * 
     * @return const T& 最新数据的常量引用
     */
    const T& peekLatest() const {
        T* latest = latest_buffer_.load(std::memory_order_acquire);
        return *latest;
    }
    
    bool tryGetLatest(T& out_data, uint32_t min_sequence = 0) const {
        T* latest = latest_buffer_.load(std::memory_order_acquire);
        
        // 检查序列号（如果启用了序列号功能）
        if (sequence_ptr_ && (latest->*sequence_ptr_) < min_sequence) {
            return false;  // 数据太旧
        }
        
        out_data = *latest;
        return true;
    }
    
    uint64_t getUpdateCount() const {
        return update_counter_.load(std::memory_order_acquire);
    }
    
    bool getLatestIfChanged(T& out_data, uint64_t& last_update_count) const {
        uint64_t current_count = update_counter_.load(std::memory_order_acquire);
        
        if (current_count == last_update_count) {
            return false;  // 计数没变，说明没有新数据
        }
        
        // 计数变了，获取最新数据
        out_data = getLatest();
        last_update_count = current_count;
        return true;
    }
    
    bool hasValidData() const {
        T* latest = latest_buffer_.load(std::memory_order_acquire);
        if (!validator_) return true;
        return validator_(*latest);
    }
    
    uint32_t getLatestSequence() const {
        if (!sequence_ptr_) return 0;
        T* latest = latest_buffer_.load(std::memory_order_acquire);
        return latest->*sequence_ptr_;
    }
    
    void clear() {
        buffers_[0] = T();
        buffers_[1] = T();
        latest_buffer_.store(&buffers_[0], std::memory_order_release);
        update_counter_.store(0, std::memory_order_release);
    }
    
private:
    T* getWriteBuffer() {
        T* latest = latest_buffer_.load(std::memory_order_acquire);
        return (latest == &buffers_[0]) ? &buffers_[1] : &buffers_[0];
    }
    
    void markAsLatest(T* buffer) {
        latest_buffer_.store(buffer, std::memory_order_release);
    }
};