#include "Device.h"
#include <iostream>
#include <iomanip> 
// #include "usb_collector.h"
#include <chrono>
#include <thread>


Device::~Device() {
    ExitDevice();
}

void Device::ExitDevice() {
    is_app_running_ = false;
    is_collecting_ = false;

    // 等待USB线程退出
    if (thread_ && thread_->joinable()) {
        thread_->join();
        delete thread_;
        thread_ = nullptr;
    }

    // 清理传输资源
    if (transfer_) {
        libusb_cancel_transfer(transfer_);
        libusb_free_transfer(transfer_);
        transfer_ = nullptr;
    }
    if (transferBuf_) {
        delete[] transferBuf_;
        transferBuf_ = nullptr;
    }

    // 释放USB资源
    if (dev_handle_) {
        libusb_release_interface(dev_handle_, 0);
        libusb_close(dev_handle_);
        dev_handle_ = nullptr;
    }
    if (ctx_) {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }

    std::cout << "USB设备资源已释放" << std::endl;
}

void Device::StartCollect() {
    is_app_running_ = true;
    // 启动USB事件循环线程（原生std::thread）
    thread_ = new std::thread(&Device::device_transfer, this);

    if (is_collecting_) return;
    is_collecting_ = true;
    // frameBuffer_.reset();
    std::cout << "开始采集，启用数据处理" << std::endl;
}

void Device::StopCollect() {
    if (!is_collecting_) return;
    is_collecting_ = false;
    // frameBuffer_.reset();
    std::cout << "停止采集，丢弃所有数据" << std::endl;
}

void Device::reset_speed_stats() {
    std::lock_guard<std::mutex> lock(speed_mtx_);
    total_recv_bytes_ = 0;
    transfer_count_ = 0;
    error_count_ = 0;
    speed_start_time_ = std::chrono::steady_clock::now();
}

void Device::device_transfer() {
    // 初始化速率统计
    reset_speed_stats();

    // // 初始化传输对象
    // transferBuf_ = new uint8_t[16*max_packet_size_];
    // transfer_ = libusb_alloc_transfer(0);
    // libusb_fill_bulk_transfer(
    //     transfer_, dev_handle_, in_endpoint_,
    //     transferBuf_, 16*max_packet_size_,
    //     transfer_callback, this, timeout_
    // );

    // 初始化多transfer
    for (int i = 0; i < TRANSFER_QUEUE_SIZE; i++) {
        transfer_bufs_[i] = new uint8_t[16*max_packet_size_];
        transfers_[i] = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(
            transfers_[i], dev_handle_, in_endpoint_,
            transfer_bufs_[i], 16*max_packet_size_,
            transfer_callback, this, // 传递transfer索引
            timeout_
        );
        libusb_submit_transfer(transfers_[i]); // 同时提交所有transfer
    }

    // 首次提交传输
    // int ret = libusb_submit_transfer(transfer_);
    // if (ret != LIBUSB_SUCCESS) {
    //     std::cerr << "提交传输失败：" << libusb_error_name(ret) << std::endl;
    // }
    std::cout << "Device 提交transfer，等待数据 ..." << std::endl;
    // std::cout << "is_app_running_:" << is_app_running_ << std::endl;
    // 状态重置
    // is_app_running_ = true;
    // is_collecting_ = true;

    // 超时阻塞事件循环
    struct timeval timeout = {0, timeout_};
    int completed;
    while (is_app_running_) {
        int ret = libusb_handle_events_timeout_completed(ctx_, &timeout, &completed);
        if (ret == LIBUSB_ERROR_TIMEOUT) {
            continue;
        }
        if (ret < 0 && ret != LIBUSB_ERROR_INTERRUPTED) {
            std::cerr << "Device 事件处理失败：" << libusb_error_name(ret) << std::endl;
        }
    }
}

void LIBUSB_CALL Device::transfer_callback(libusb_transfer* tr) {
   // std::cout << "Device transfer_callback 获取数据 正在处理..." << std::endl;
    Device* device = static_cast<Device*>(tr->user_data);

    if (!device) return;

    // 处理数据（或丢弃）
    if (tr->status == LIBUSB_TRANSFER_COMPLETED) {
        // 累计传输成功计数
        device->transfer_count_++;
        // 累计接收字节数（原子操作，线程安全）
        device->total_recv_bytes_ += tr->actual_length;

        if (device->is_collecting_) {
            device->frameBuffer_.writeChunk(tr->buffer, tr->actual_length, true); 
        } else {
            std::cout << "停止采集，丢弃16KB分块数据" << std::endl;
            // device->frameBuffer_.reset();
        }


        // ===== 新增：速率统计与打印 =====
        std::lock_guard<std::mutex> lock(device->speed_mtx_);
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - device->speed_start_time_).count();

        // 每 SPEED_PRINT_INTERVAL 秒打印一次速率
        if (elapsed >= device->SPEED_PRINT_INTERVAL) {
            // 计算速率（KB/s / MB/s）
            double speed_kB = static_cast<double>(device->total_recv_bytes_) / 1024.0 / elapsed;
            double speed_mB = speed_kB / 1024.0;
            // 计算平均每次传输字节数、传输成功率
            double avg_bytes_per_trans = device->transfer_count_ > 0 ? 
                static_cast<double>(device->total_recv_bytes_) / device->transfer_count_ : 0;
            double success_rate = device->transfer_count_ + device->error_count_ > 0 ?
                (static_cast<double>(device->transfer_count_) / (device->transfer_count_ + device->error_count_)) * 100 : 100;

            // 打印测速信息
            std::cout << "\n===== USB 传输统计 =====" << std::endl;
            std::cout << "累计时间：" << elapsed << "秒" << std::endl;
            std::cout << "实时速率：" << speed_kB << " KB/s (" << speed_mB << " MB/s)" << std::endl;
            std::cout << "传输次数：成功 " << device->transfer_count_ << " 次 | 失败 " << device->error_count_ << " 次" << std::endl;
            std::cout << "平均每包：" << avg_bytes_per_trans << " 字节 | 成功率：" << success_rate << "%" << std::endl;
            std::cout << "=========================\n" << std::endl;

            // 重置统计（用于下一个时间窗口）
            device->speed_start_time_ = now;
            device->total_recv_bytes_ = 0;
            device->transfer_count_ = 0;
            device->error_count_ = 0;
        }

    } else {
        device->error_count_++;
        std::cerr << "USB传输失败：" << libusb_error_name(tr->status) << std::endl;
        // TODO：传输失败，重置frameBuffer_可能会出现问题。需要将16KB数据全部丢弃然后再重置。后续看是否会出现传输失败的情况
        // device->frameBuffer_.reset();
    }


    // 永久提交传输
    if (device->is_app_running_) {
        int ret = libusb_submit_transfer(tr);
        if (ret != LIBUSB_SUCCESS) {
            std::cerr << "重提交传输失败：" << libusb_error_name(ret) << std::endl;
            device->error_count_++;
        }
    }
    // std::cout << "Device 提交transfer，等待数据 ..." << std::endl;
    
}

Device::Device(FrameBuffer& frameBuffer, uint16_t vid, uint16_t pid, uint8_t out_ep, uint8_t in_ep)
: frameBuffer_(frameBuffer), vid_(vid), pid_(pid), out_endpoint_(out_ep), in_endpoint_(in_ep) {
        // 初始化原类资源（libusb 上下文、设备句柄、接口占用）
        if (!init_device()) {
            std::cerr << "[错误] 基础资源初始化失败，异步传输无法启动" << std::endl;
            return;
        }
    }

bool Device::init_device()
{
    // 1. 初始化 libusb 上下文
    int ret = libusb_init(&ctx_);
    if (ret != LIBUSB_SUCCESS) {
        std::cerr << "[错误] libusb 初始化失败：" << libusb_strerror((libusb_error)ret) << std::endl;
        return false;
    }
    std::cout << "[成功] libusb 初始化完成" << std::endl;

    // 2. 打开设备
    dev_handle_ = libusb_open_device_with_vid_pid(ctx_, vid_, pid_);
    if (dev_handle_ == nullptr) {
        std::cerr << "[错误] 设备打开失败（VID:0x" << std::hex << vid_ << ", PID:0x" << pid_ << "），请确认设备已连接" << std::endl;
        libusb_exit(ctx_);
        ctx_ = nullptr;
        return false;
    }
    std::cout << "[成功] 设备打开完成（句柄：" << (void*)dev_handle_ << "）" << std::endl;

    // 3. 解析配置描述符（获取最大包长）
    parse_config_descriptor();

    // 4. 占用接口（强制解绑内核驱动）
    if (!claim_interface()) {
        libusb_close(dev_handle_);
        libusb_exit(ctx_);
        dev_handle_ = nullptr;
        ctx_ = nullptr;
        return false;
    }
    std::cout << "设备初始化成功" << std::endl;

    return true;
}

uint16_t Device::get_packet_size(uint16_t le_val) {
    // 例如 le_val=0x0400（小端存储：低字节=0x00，高字节=0x04）
    uint8_t low_byte = (uint8_t)(le_val & 0x00FF);  // 取低 8 位：0x00
    uint8_t high_byte = (uint8_t)((le_val >> 8) & 0x00FF);  // 取高 8 位：0x04

    return (uint16_t)((high_byte << 8) | low_byte);  // 组合：0x04 <<8 + 0x00 = 0x0400=1024
}

// 解析配置描述符：获取端点最大包长、接口数量等信息（核心：get_config_descriptor 用法）
void Device::parse_config_descriptor() {
    if (dev_handle_ == nullptr) return;

    // 获取设备对象（从设备句柄中提取设备信息）
    libusb_device* usb_dev = libusb_get_device(dev_handle_);
    if (usb_dev == nullptr) {
        std::cerr << "[警告] 无法获取设备对象，使用默认最大包长 " << std::dec << max_packet_size_ << " 字节" << std::endl;
        return;
    }

    // 1. 获取设备的活动配置描述符（libusb_get_config_descriptor）
    // 参数1：设备对象；参数2：配置编号（0 表示获取当前活动配置）；参数3：输出配置描述符指针
    libusb_config_descriptor* config_desc = nullptr;
    int ret = libusb_get_config_descriptor(usb_dev, 0, &config_desc);
    if (ret != LIBUSB_SUCCESS) {
        std::cerr << "[警告] 获取配置描述符失败：" << libusb_strerror((libusb_error)ret) 
                    << "，使用默认最大包长 " << std::dec << max_packet_size_ << " 字节" << std::endl;
        return;
    }

    // 2. 解析配置描述符的核心信息
    std::cout << "\n[配置描述符信息] " << std::endl;
    std::cout << "  配置编号：" << (int)config_desc->bConfigurationValue << std::endl;
    std::cout << "  接口数量（bNumInterfaces）：" << (int)config_desc->bNumInterfaces << std::endl;
    std::cout << "  最大功耗：" << (int)config_desc->MaxPower * 2 << " mA（USB 协议：MaxPower 单位为 mA）" << std::endl;

    // 3. 遍历接口（当前设备 bNumInterfaces=1，仅接口 0）
    for (int i = 0; i < config_desc->bNumInterfaces; i++) {
        // 获取接口描述符（每个接口可能有多个备用配置，默认取第一个）
        const libusb_interface* iface = &config_desc->interface[i];
        const libusb_interface_descriptor* iface_desc = &iface->altsetting[0];

        std::cout << "  接口 " << (int)iface_desc->bInterfaceNumber << "：" << std::endl;
        std::cout << "    端点数量：" << (int)iface_desc->bNumEndpoints << std::endl;

        // 4. 遍历端点（获取每个端点的最大包长、地址等信息）
        for (int j = 0; j < iface_desc->bNumEndpoints; j++) {
            // interface_num_ = j;
            const libusb_endpoint_descriptor* ep_desc = &iface_desc->endpoint[j];

            // 筛选 Bulk 类型端点（忽略其他类型如 Interrupt）
            if ((ep_desc->bmAttributes & 0x03) != 0x02) continue;

            // 解析端点方向和地址
            std::string ep_dir = (ep_desc->bEndpointAddress & 0x80) ? "IN" : "OUT";
            uint8_t ep_addr = ep_desc->bEndpointAddress;
            // 解析最大包长（USB 描述符为小端字节序，需转为主机字节序）
            uint16_t ep_max_packet = get_packet_size(ep_desc->wMaxPacketSize);

            std::cout << "    端点 " << j+1 << "：" << std::endl;
            std::cout << "      方向：" << ep_dir << "，地址：0x" << std::hex << (int)ep_addr << std::endl;
            std::cout << "      最大包长：" << std::dec << ep_max_packet << " 字节" << std::endl;

            // 更新最大包长（以 IN 端点为准，与设备发送逻辑匹配）
            if (ep_dir == "IN") {
                max_packet_size_ = ep_max_packet;
                std::cout << "      [当前使用] IN 端点最大包长：" << std::dec << max_packet_size_ << " 字节" << std::endl;
            }
        }
    }

    // 5. 释放配置描述符（必须调用，否则内存泄漏）
    libusb_free_config_descriptor(config_desc);
}

// 占用接口：USB 传输前的必需操作
bool Device::claim_interface() {
    std::cout << "interface_num_: " << interface_num_ << std::endl;
    if (dev_handle_== nullptr) {
        std::cerr << "[错误] 设备未初始化，无法占用接口" << std::endl;
        return false;
    }

    // 检查接口是否被内核驱动占用，若占用则强制解绑
    if (libusb_kernel_driver_active(dev_handle_, interface_num_)) {
        std::cout << "[提示] 接口 " << interface_num_ << " 被内核驱动占用，尝试强制解绑" << std::endl;
        int ret = libusb_detach_kernel_driver(dev_handle_, interface_num_);
        if (ret != LIBUSB_SUCCESS) {
            std::cerr << "[错误] 解绑内核驱动失败：" << libusb_strerror((libusb_error)ret) << std::endl;
            return false;
        }
        std::cout << "[成功] 强制解绑内核驱动" << std::endl;
    }

    // 占用接口（libusb_claim_interface：独占接口，防止多程序冲突）
    int ret = libusb_claim_interface(dev_handle_, interface_num_);
    if (ret != LIBUSB_SUCCESS) {
        std::cerr << "[错误] 占用接口 " << interface_num_ << " 失败：" << libusb_strerror((libusb_error)ret) 
                    << "（错误码：" << ret << "）" << std::endl;
        return false;
    }

    std::cout << "[成功] 占用接口 " << interface_num_ << " 完成" << std::endl;
    return true;
}

// 批量发送数据（主机 → 设备，使用 OUT 端点）
bool Device::SyncSendData(const uint8_t* data, int data_len) {
    // 参数合法性检查
    if (dev_handle_ == nullptr || data == nullptr || data_len <= 0) {
        std::cerr << "[错误] 发送参数无效（设备未初始化或数据为空）" << std::endl;
        return false;
    }
    if (data_len > max_packet_size_) {
        std::cerr << "[错误] 发送长度（" << data_len << " 字节）超过最大包长（" 
                    << std::dec << max_packet_size_ << " 字节）" << std::endl;
        return false;
    }

    int actual_sent = 0;  // 实际发送的字节数
    // 调用 libusb 批量传输 API（OUT 端点）
    int ret = libusb_bulk_transfer(
        dev_handle_,    // 设备句柄
        out_endpoint_,   // OUT 端点地址（0x01）
        const_cast<uint8_t*>(data),  // 发送数据缓冲区（const 转非 const，libusb 兼容）
        data_len,           // 要发送的长度
        &actual_sent,       // 输出：实际发送的长度
        timeout_         // 超时时间（毫秒）
    );

    if (ret == LIBUSB_SUCCESS && actual_sent == data_len) {
        std::cout << "[成功] 发送数据：" << actual_sent << " 字节（数据：";
        for (int i = 0; i < actual_sent; i++) {
            std::cout << "0x" << std::hex << (int)data[i] << " ";
        }
        std::cout << "）" << std::endl;
        return true;
    } else {
        std::cerr << "[错误] 发送失败：" << libusb_strerror((libusb_error)ret) 
                    << "（错误码：" << ret << "，实际发送：" << actual_sent << " 字节）" << std::endl;
        return false;
    }
}

// 批量接收数据（设备 → 主机，使用 IN 端点）
bool Device::SyncRecvData(uint8_t* recv_buf, int buf_max_len, int& actual_recv_len) {
    // 参数合法性检查
    if (dev_handle_ == nullptr || recv_buf == nullptr || buf_max_len <= 0) {
        std::cerr << "[错误] 接收参数无效（设备未初始化或缓冲区为空）" << std::endl;
        actual_recv_len = 0;
        return false;
    }
    if (buf_max_len < max_packet_size_) {
        std::cerr << "[警告] 接收缓冲区（" << buf_max_len << " 字节）小于最大包长（" 
                    << std::dec << max_packet_size_ << " 字节），可能接收不完整" << std::endl;
    }

    actual_recv_len = 0;  // 实际接收的字节数
    // 调用 libusb 批量传输 API（IN 端点）
    int ret = libusb_bulk_transfer(
        dev_handle_,    // 设备句柄
        in_endpoint_,    // IN 端点地址（0x81）
        recv_buf,           // 接收数据缓冲区
        buf_max_len,        // 缓冲区最大长度
        &actual_recv_len,   // 输出：实际接收的长度
        timeout_         // 超时时间（毫秒）
    );

    if (ret == LIBUSB_SUCCESS) {
        std::cout << "[成功] 接收数据：" << actual_recv_len << " 字节（数据：";
        for (int i = 0; i < actual_recv_len; i++) {
            // std::cout << recv_buf[i] << " ";
            std::cout << "0x" << std::setw(2) << std::setfill('0') << (int)recv_buf[i] << " "; 
        }
        std::cout << "）" << std::endl;
        return true;
    } else {
        std::cerr << "[错误] 接收失败：" << libusb_strerror((libusb_error)ret) 
                    << "（错误码：" << ret << "，实际接收：" << actual_recv_len << " 字节）" << std::endl;
        return false;
    }
}