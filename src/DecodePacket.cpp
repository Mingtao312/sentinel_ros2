#include "DecodePacket.h"
#include <iostream>
#include <cstdio>
#include <iomanip> 
#include <unistd.h>

// ========== USBDataProcessor 实现 ==========
DecodePacket::DecodePacket(FrameBuffer& frameBuffer, Device& device, size_t frame_total_size)
    : frameBuffer_(frameBuffer), device_(device), frame_total_size_(frame_total_size), 
    imu_buffer_(IMUData::validate, &IMUData::sequence),
    rgb_left_buffer_(RGBData::validate, &RGBData::sequence),
    rgb_right_buffer_(RGBData::validate, &RGBData::sequence),
    sdl_left_buffer_(RODData::validate, &RODData::sequence),
    sdr_left_buffer_(RODData::validate, &RODData::sequence),
    td_left_buffer_(RODData::validate, &RODData::sequence),
    sdl_right_buffer_(RODData::validate, &RODData::sequence),
    sdr_right_buffer_(RODData::validate, &RODData::sequence),
    td_right_buffer_(RODData::validate, &RODData::sequence)
{
    last_imu_count_ = imu_buffer_.getUpdateCount();
    last_rgb_left_count_ = rgb_left_buffer_.getUpdateCount();
    last_rgb_right_count_ = rgb_right_buffer_.getUpdateCount();

    last_td_left_count_ = td_left_buffer_.getUpdateCount();
    last_sdl_left_count_ = sdl_left_buffer_.getUpdateCount();
    last_sdr_left_count_ = sdr_left_buffer_.getUpdateCount();

    last_td_right_count_ = td_right_buffer_.getUpdateCount();
    last_sdl_right_count_ = sdl_right_buffer_.getUpdateCount();
    last_sdr_right_count_ = sdr_right_buffer_.getUpdateCount();
}

DecodePacket::~DecodePacket() {
    stop();
}

void DecodePacket::start() {
    decodeThread_ = new std::thread(&DecodePacket::decodeWorker, this);
    std::cout << "数据处理器启动成功" << std::endl;
}

void DecodePacket::stop() {
    if (decodeThread_ && decodeThread_->joinable()) {
        decodeThread_->join();
        delete decodeThread_;
        decodeThread_ = nullptr;
    }
    std::cout << "数据处理器已停止" << std::endl;
}

bool DecodePacket::getIMUData(IMUData& imu_data) {
    if (imu_buffer_.getLatestIfChanged(imu_data, last_imu_count_)) {
        // std::cout << "[Consumer-方法1] 收到新数据: ";
        // printIMU(imu_data);
        return true;
    } else {
        return false;
    }
}

bool DecodePacket::getRGBLeftData(RGBData& rgb_data) {
    if (rgb_left_buffer_.getLatestIfChanged(rgb_data, last_rgb_left_count_)) {
        // std::cout << "[Consumer-方法1] 收到新数据: ";
        // printIMU(imu_data);
        return true;
    } else {
        return false;
    }
}

bool DecodePacket::getSDLLeftData(RODData& rod_data) {
    if (sdl_left_buffer_.getLatestIfChanged(rod_data, last_sdl_left_count_)) {
        // std::cout << "[Consumer-方法1] 收到新数据: ";
        // printIMU(imu_data);
        return true;
    } else {
        return false;
    }
}

bool DecodePacket::getSDRLeftData(RODData& rod_data) {
    if (sdr_left_buffer_.getLatestIfChanged(rod_data, last_sdr_left_count_)) {
        // std::cout << "[Consumer-方法1] 收到新数据: ";
        // printIMU(imu_data);
        return true;
    } else {
        return false;
    }
}

bool DecodePacket::getTDLeftData(RODData& rod_data) {
    if (td_left_buffer_.getLatestIfChanged(rod_data, last_td_left_count_)) {
        // std::cout << "[Consumer-方法1] 收到新数据: ";
        // printIMU(imu_data);
        return true;
    } else {
        return false;
    }
}

bool DecodePacket::getSDLRightData(RODData& rod_data) {
    if (sdl_right_buffer_.getLatestIfChanged(rod_data, last_sdl_right_count_)) {
        // std::cout << "[Consumer-方法1] 收到新数据: ";
        // printIMU(imu_data);
        return true;
    } else {
        return false;
    }
}

bool DecodePacket::getSDRRightData(RODData& rod_data) {
    if (sdr_right_buffer_.getLatestIfChanged(rod_data, last_sdr_right_count_)) {
        // std::cout << "[Consumer-方法1] 收到新数据: ";
        // printIMU(imu_data);
        return true;
    } else {
        return false;
    }
}

bool DecodePacket::getTDRightData(RODData& rod_data) {
    if (td_right_buffer_.getLatestIfChanged(rod_data, last_td_right_count_)) {
        // std::cout << "[Consumer-方法1] 收到新数据: ";
        // printIMU(imu_data);
        return true;
    } else {
        return false;
    }
}

bool DecodePacket::getRGBRightData(RGBData& rgb_data) {
    if (rgb_right_buffer_.getLatestIfChanged(rgb_data, last_rgb_right_count_)) {
        // std::cout << "[Consumer-方法1] 收到新数据: ";
        // printIMU(imu_data);
        return true;
    } else {
        return false;
    }
}

bool DecodePacket::hasIMUData() const {
    return(imu_buffer_.hasValidData());
}

// TODO：这里while(true)占用cpu，应该也改写成条件变量
void DecodePacket::decodeWorker() {
    uint8_t frameBuf[frame_total_size_];
    size_t frameLen = frame_total_size_;

    while (true) {
        if (!device_.IsCollecting()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // 读取完整帧
        if (!frameBuffer_.readFrame(frameBuf, frameLen, device_.IsCollecting(), device_.IsAppRunning())) {
            continue;
        }

        // // 解包逻辑
        // std::cout << "收到完整16KB帧，开始解包...,数据长度为： " << frameLen << std::endl;
        // for (int i = 0; i < 20; i++) {
        //     // std::cout << " " << (int)frameBuf[i];
        //     std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') 
        //       << static_cast<int>(frameBuf[i]) << " ";
        // }
        // std::cout << std::endl;

        int offset = 0;
        while (offset < frameLen){

            // std::cout << "offset: " << offset << std::endl;
            // std::cout << "offset: 0x" << std::hex << std::setw(2) << std::setfill('0') 
            //     << offset << std::endl;

            if (frameBuf[offset] != 0xFA) {

                // std::cout << "frameBuf[offset]: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                // << static_cast<int>(frameBuf[offset]) << std::endl;

                // std::cout << "offset: " << offset << std::endl;

                // if (access("uint8_hex.txt", F_OK) == 0) {
                //     printf("文件 %s 已存在，跳过写入操作\n", "uint8_hex.txt");
                // } else {
                //     FILE *fp = fopen("uint8_hex.txt", "w");
                //     if (fp == NULL) {
                //         perror("打开文件失败");
                //     }

                //     // 遍历数组，以16进制写入（%02X表示2位大写十六进制，不足补0）
                //     for (size_t i = 0; i < frameLen; i++) {
                //         fprintf(fp, "%02X ", frameBuf[i]); // 每个元素后加空格分隔
                //         // 可选：每16个元素换行，方便阅读
                //         if ((i + 1) % 16 == 0) {
                //             fprintf(fp, "\n");
                //         }
                //     }

                //     fclose(fp);
                //     printf("数组已以16进制保存到uint8_hex.txt\n");
                // }

                std::cout << "数据包格式错误，无法完成后续的解包逻辑！！！" << std::endl;
                break;
            }
            
            offset += 1;
            // 高4位掩码: 0xF0 = 1111 0000
            // imu解码逻辑
            if ((frameBuf[offset] & 0xF0) == 0) {
                // std::cout << "IMU数据" << std::endl;
                offset += 3;
                uint64_t time_step = 0;
                uint32_t time_step_low = 0;
                uint32_t time_step_high = 0;
                IMUData imu_data;

                for (int i = 3; i >= 0; i--) {
                    time_step_low |= static_cast<uint32_t>(frameBuf[offset + i]) << (8 * (3-i));
                }
                for (int i = 7; i >= 4; i--) {
                    time_step_high |= static_cast<uint32_t>(frameBuf[offset + i]) << (8 * (7-i));
                }

                time_step |= static_cast<uint64_t>(time_step_low);
                time_step |= static_cast<uint64_t>(time_step_high) << 32;

                imu_data.system_time= time_step;
                offset += 8;
                
                DecodeErrorCode re = decodeIMU(&frameBuf[offset], imu_data);
                
                if (re == DecodeErrorCode::SUCCESS) {
                    imu_data.valid = true;\
                    imu_data.sequence = imu_sq_;
                    // printIMU(imu_data);
                    // std::cout << "imu_data.sequence: " << imu_data.sequence << "imu_data.valid: " << imu_data.valid << std::endl;
                    imu_sq_ += 1;

                    // 更新到缓冲区
                    if (imu_buffer_.update(imu_data)) {
                        // std::cout << "[Producer] 发送数据 #" << imu_data.sequence;
                        // std::cout << " (更新计数: " << imu_buffer_.getUpdateCount() << ")" << std::endl;
                        // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    } else {
                        std::cout << "[Producer] 数据验证失败，丢弃" << std::endl;
                    }
                } else {
                    std::cerr << "IMU解码失败，错误码为: " << int(re) << " IMU数据包丢弃" << std::endl;
                }
                offset += 76;
                
            }
            // } else if (((frameBuf[offset] & 0x40) == 0x40) && ((frameBuf[offset] & 0x80) == 0x00)) {
            else if ((frameBuf[offset] & 0xC0) == 0x40) {
                // rgb解码逻辑
                // std::cout << "RGB数据" << std::endl;
                if ((frameBuf[offset] | 0xCF) == 0xEF) {
                    // 左目RGB
                    bool flag = bool(frameBuf[offset] & 0x01);
                    if (flag != rgb_left_flag_) {
                        // std::cout << "变帧!!! " << std::endl;
                        rgb_left_change_ = true;
                        rgb_left_flag_ = flag;
                    }

                    offset += 3;
                    uint64_t time_step = 0;
                    uint32_t time_step_low = 0;
                    uint32_t time_step_high = 0;
                    for (int i = 3; i >= 0; i--) {
                        time_step_low |= static_cast<uint32_t>(frameBuf[offset + i]) << (8 * (3-i));
                    }
                    for (int i = 7; i >= 4; i--) {
                        time_step_high |= static_cast<uint32_t>(frameBuf[offset + i]) << (8 * (7-i));
                    }
                    time_step |= static_cast<uint64_t>(time_step_low);
                    time_step |= static_cast<uint64_t>(time_step_high) << 32;
                    offset += 8;

                    // rgb每行进行传输，
                    // if (rgb_left_data_.system_time != time_step) {
                    if (rgb_left_change_) {
                        if (rgb_left_data_.pixel_rows.size() == 0) {
                            rgb_left_data_.system_time = time_step;
                            rgb_left_data_.left_right = 0;
                            rgb_left_data_.sequence = rgb_left_sq_;
                        } else if (rgb_left_data_.pixel_rows.size() == rgb_left_data_.height) {
                            rgb_left_data_.valid = true;
                            // 将上一帧rgb数据更新到缓冲区
                            if (rgb_left_buffer_.update(rgb_left_data_)) {
                                // std::cout << "RGB_LEFT [Producer] 发送数据 #" << rgb_left_data_.sequence;
                                // std::cout << " (更新计数: " << imu_buffer_.getUpdateCount() << ")" << std::endl;
                                // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                            } else {
                                std::cout << "RGB_LEFT [Producer] 数据验证失败，丢弃" << std::endl;
                            }
                            
                            rgb_left_data_.pixel_rows.clear();
                            rgb_left_data_.system_time = time_step;
                            rgb_left_sq_ += 1;
                            rgb_left_data_.sequence = rgb_left_sq_;
                            rgb_left_data_.valid = false;
                            rgb_left_data_.left_right = 0;
                        } else {
                            std::cout << "左目数据每满，丢弃！！！" << std::endl;
                            rgb_left_data_.pixel_rows.clear();
                            rgb_left_data_.system_time = time_step;
                            rgb_left_data_.sequence = rgb_left_sq_;
                            rgb_left_data_.valid = false;
                            rgb_left_data_.left_right = 0;
                        }

                        rgb_left_change_ = false;

                    }

                    DecodeErrorCode re = decodeRGB(&frameBuf[offset], rgb_left_data_);
                    offset += rgb_left_data_.height * 4;
                    // printIMU(imu_data);
                    // std::cout << "imu_data.sequence: " << imu_data.sequence << "imu_data.valid: " << imu_data.valid << std::endl;

                } else if ((frameBuf[offset] | 0xCF) == 0xFF) {
                    // 右目RGB
                    bool flag = bool(frameBuf[offset] & 0x01);
                    // std::cout << "解码flag: " << flag << std::endl; 
                    if (flag != rgb_right_flag_) {
                        // std::cout << "变帧!!! " << std::endl;
                        rgb_right_change_ = true;
                        rgb_right_flag_ = flag;
                    }
                    
                    offset += 3;
                    uint64_t time_step = 0;
                    uint32_t time_step_low = 0;
                    uint32_t time_step_high = 0;
                    for (int i = 3; i >= 0; i--) {
                        time_step_low |= static_cast<uint32_t>(frameBuf[offset + i]) << (8 * (3-i));
                    }
                    for (int i = 7; i >= 4; i--) {
                        time_step_high |= static_cast<uint32_t>(frameBuf[offset + i]) << (8 * (7-i));
                    }
                    time_step |= static_cast<uint64_t>(time_step_low);
                    time_step |= static_cast<uint64_t>(time_step_high) << 32;
                    offset += 8;

                    // std::cout << "time_step: " << time_step << std::endl;

                    // rgb每行进行传输，
                    // if (rgb_right_data_.system_time != time_step) {
                    if (rgb_right_change_) {
                        if (rgb_right_data_.pixel_rows.size() == 0) {
                            rgb_right_data_.system_time = time_step;
                            rgb_right_data_.left_right = 1;
                            rgb_right_data_.sequence = rgb_right_sq_;
                        } else if (rgb_right_data_.pixel_rows.size() == rgb_right_data_.height) {
                            rgb_right_data_.valid = true;
                            // 将上一帧rgb数据更新到缓冲区
                            if (rgb_right_buffer_.update(rgb_right_data_)) {
                                // std::cout << "RGB_RIGHT [Producer] 发送数据 #" << rgb_right_data_.sequence;
                                // std::cout << " (更新计数: " << imu_buffer_.getUpdateCount() << ")" << std::endl;
                                // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                            } else {
                                std::cout << "RGB_RIGHT [Producer] 数据验证失败，丢弃" << std::endl;
                            }
                            
                            rgb_right_data_.pixel_rows.clear();
                            rgb_right_data_.system_time = time_step;
                            rgb_right_sq_ += 1;
                            rgb_right_data_.sequence = rgb_right_sq_;
                            rgb_right_data_.valid = false;
                            rgb_right_data_.left_right = 1;
                        } else {
                            std::cout << "右目数据每满，丢弃！！！" << std::endl;
                            rgb_right_data_.pixel_rows.clear();
                            rgb_right_data_.system_time = time_step;
                            rgb_right_data_.sequence = rgb_right_sq_;
                            rgb_right_data_.valid = false;
                            rgb_right_data_.left_right = 1;
                        }
                        rgb_right_change_ = false;
                    }

                    DecodeErrorCode re = decodeRGB(&frameBuf[offset], rgb_right_data_);
                    offset += rgb_right_data_.height * 4;

                } else {
                    std::cerr << "RGB左右目格式错误！！！ " << std::endl;
                    break;
                }
            }  
            else if ((frameBuf[offset] & 0xC0) == 0x00) {
                // 前面已经解码过imu0x00，后面高两位依然为00可能是ROD
                // std::cout << "ROD数据" << std::endl;
                int of = 0;
                DecodeErrorCode re = decodeROD(&frameBuf[offset], of);
                // std::cout << "of: " << of << std::endl;
                offset += of;
            }
            else if ((frameBuf[offset] & 0xF0) == 0xF0) {
                // std::cout << "此包结束，解析下一包数据！！！" << std::endl;
                break;
            } else {
                std::cerr << "未出现的数据类型...，发送的数据格式存在错误！！！" << std::endl;
                break;
            }
        }

    }

}

// 16位小端序转主机序
static uint16_t le16_to_host(const uint8_t* buf) {
    return (static_cast<uint16_t>(buf[1]) << 8) | static_cast<uint16_t>(buf[0]);
}

// 32位小端序转主机序
static uint32_t le32_to_host(const uint8_t* buf) {
    return (static_cast<uint32_t>(buf[3]) << 24) | (static_cast<uint32_t>(buf[2]) << 16) |
           (static_cast<uint32_t>(buf[1]) << 8)  | static_cast<uint32_t>(buf[0]);
}

// 小端序字节流转float（32位）
static float le_bytes_to_float(const uint8_t* buf) {
    uint32_t val = le32_to_host(buf);
    float f;
    std::memcpy(&f, &val, sizeof(float));
    return f;
}

// 打印解码结果（便于调试）
void DecodePacket::printIMU(IMUData& out_data) {
    std::cout << "===== IMU数据解码结果 =====" << std::endl;
    // std::cout << "Tag值: 0x" << std::hex << static_cast<int>(out_data.tag) << std::endl;
    // std::cout << "保留字段: 0x" << out_data.reserved << std::dec << std::endl;
    std::cout << "温度: " << static_cast<int>(out_data.temperature) << " ℃" << std::endl;
    std::cout << "气压: " << out_data.air_pressure << " Pa" << std::endl;
    std::cout << "时间戳: " << out_data.system_time << " ms" << std::endl;
    std::cout << "加速度X轴: " << out_data.acc_b_x << " G" << std::endl;
    std::cout << "加速度Y轴: " << out_data.acc_b_y << " G" << std::endl;
    std::cout << "加速度Z轴: " << out_data.acc_b_z << " G" << std::endl;
    std::cout << "角速度X轴: " << out_data.gyr_b_x << " dps" << std::endl;
    std::cout << "角速度Y轴: " << out_data.gyr_b_y << " dps" << std::endl;
    std::cout << "角速度Z轴: " << out_data.gyr_b_z << " dps" << std::endl;
    std::cout << "磁场X轴: " << out_data.mag_b_x << " uT" << std::endl;
    std::cout << "磁场Y轴: " << out_data.mag_b_y << " uT" << std::endl;
    std::cout << "磁场Z轴: " << out_data.mag_b_z << " uT" << std::endl;
    std::cout << "横滚角: " << out_data.roll << " deg" << std::endl;
    std::cout << "俯仰角: " << out_data.pitch << " deg" << std::endl;
    std::cout << "航向角: " << out_data.yaw << " deg" << std::endl;
    std::cout << "四元数W: " << out_data.q_w << std::endl;
    std::cout << "四元数X: " << out_data.q_x << std::endl;
    std::cout << "四元数Y: " << out_data.q_y << std::endl;
    std::cout << "四元数Z: " << out_data.q_z << std::endl;
    std::cout << "==============================" << std::endl;
}

DecodeErrorCode DecodePacket::decodeRGB(uint8_t* raw_data, RGBData& out_data) {
    size_t offset = 0;
    std::vector<RGBPixel> row_data;
    for (uint32_t y = 0; y < out_data.height; y++) {
        RGBPixel p;
        p.r = raw_data[offset];
        p.g = raw_data[offset + 1];
        p.b = raw_data[offset + 2];
        row_data.push_back(p);
        offset += 4;
    }
    // std::cout << "decodeRGB row_data: " << row_data.size() << " out_data: " << out_data.pixel_rows.size() << std::endl;
    out_data.pixel_rows.push_back(row_data);
    return DecodeErrorCode::SUCCESS;
}

DecodeErrorCode DecodePacket::decodeROD(uint8_t* raw_data, int& of_B) {
    // 所有的0xFA 00**均进入这里进行解码,raw_data从0xFA后面开始
    if ((raw_data[of_B] & 0xF0) == 0x20) {
        // std::cout << "ROD左" << std::endl;
        // 判断是不是FN数据
        if ((raw_data[of_B] & 0xFE) == 0x24) {
            // std::cout << "左FN" << std::endl;
            // 后面先不解析，直接跳过
            of_B += 3;

            if ((td_left_data_.valid) & (td_left_data_.system_time != 0)) {
                // 更新到缓冲区
                if (td_left_buffer_.update(td_left_data_)) {
                    // std::cout << "左TD数据更新" << std::endl;
                    td_left_data_.valid = false;
                    td_left_data_.rod_data = std::vector<std::vector<int8_t>>(160, std::vector<int8_t>(160, 0));

                    td_left_sq_ += 1;
                    td_left_data_.sequence = td_left_sq_;

                    // std::cout << "[Producer] 发送数据 #" << imu_data.sequence;
                    // std::cout << " (更新计数: " << imu_buffer_.getUpdateCount() << ")" << std::endl;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } else {
                    std::cout << "[Producer] 数据验证失败，丢弃" << std::endl;
                }

            }

            if ((sdl_left_data_.valid) & (sdl_left_data_.system_time != 0)) {
                if (sdl_left_buffer_.update(sdl_left_data_)) {
                    sdl_left_data_.valid = false;
                    sdl_left_data_.rod_data = std::vector<std::vector<int8_t>>(160, std::vector<int8_t>(160, 0));;
                    sdl_left_sq_ += 1;
                    sdl_left_data_.sequence = sdl_left_sq_;
                    // std::cout << "[Producer] 发送数据 #" << imu_data.sequence;
                    // std::cout << " (更新计数: " << imu_buffer_.getUpdateCount() << ")" << std::endl;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } else {
                    std::cout << "[Producer] 数据验证失败，丢弃" << std::endl;
                }
            }
            if ((sdr_left_data_.valid) & (sdr_left_data_.system_time != 0)) {
                if (sdr_left_buffer_.update(sdr_left_data_)) {
                    sdr_left_data_.valid = false;
                    sdr_left_data_.rod_data = std::vector<std::vector<int8_t>>(160, std::vector<int8_t>(160, 0));;
                    sdr_left_sq_ += 1;
                    sdr_left_data_.sequence = sdr_left_sq_;
                    // std::cout << "[Producer] 发送数据 #" << imu_data.sequence;
                    // std::cout << " (更新计数: " << imu_buffer_.getUpdateCount() << ")" << std::endl;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } else {
                    std::cout << "[Producer] 数据验证失败，丢弃" << std::endl;
                }
            }

            uint64_t time_step = 0;
            uint32_t time_step_low = 0;
            uint32_t time_step_high = 0;

            for (int i = 3; i >= 0; i--) {
                time_step_low |= static_cast<uint32_t>(raw_data[of_B + i]) << (8 * (3-i));
            }
            for (int i = 7; i >= 4; i--) {
                time_step_high |= static_cast<uint32_t>(raw_data[of_B + i]) << (8 * (7-i));
            }

            time_step |= static_cast<uint64_t>(time_step_low);
            time_step |= static_cast<uint64_t>(time_step_high) << 32;

            sdl_left_data_.system_time = time_step;
            sdr_left_data_.system_time = time_step;
            td_left_data_.system_time = time_step;
            of_B += 8;

        } 
        else if (raw_data[of_B] == 0x20) {
            // std::cout << "左TD一行" << std::endl;
            of_B += 1;
            // 这里group_num放置的是否正确？
            uint8_t high7_bits = raw_data[of_B] & 0xFE;
            uint8_t group_num = high7_bits >> 1;
            if (group_num > 80) {
                std::cout << "左TD:group_num错误，软件可能core_dump: " << static_cast<int>(group_num) << std::endl;
            }
            of_B += 1;
            uint8_t row_addr = raw_data[of_B];
            of_B += 1;
            // 开始进行一行数据的解码过程
            for (int i = group_num; i > 0; i--) {
                // group_num最高位为1
                if ((raw_data[of_B] & 0x80) == 0x80) {
                    uint8_t col_addr = raw_data[of_B] & 0x7F;
                    col_addr = col_addr * 2;
                    if (col_addr > 160) {
                        std::cout << "左TD:col_addr错误，软件可能core_dump: " << static_cast<int>(col_addr) << std::endl;
                    }
                    of_B += 1;
                    td_left_data_.rod_data[row_addr][col_addr] = static_cast<int8_t>(raw_data[of_B]);
                    td_left_data_.rod_data[row_addr][col_addr + 1] = static_cast<int8_t>(raw_data[of_B + 1]);
                    of_B += 3;
                } else {
                    std::cout << "左TD,group数据错误" << std::endl;
                    of_B += 4;
                }
            }
            td_left_data_.valid = true;
        }
        else if (raw_data[of_B] == 0x28) {
            // std::cout << "左sdl一行" << std::endl;
            of_B += 1;
            // 这里group_num放置的是否正确？
            uint8_t high7_bits = raw_data[of_B] & 0xFE;
            uint8_t group_num = high7_bits >> 1;
            if (group_num > 80) {
                std::cout << "左sdl:group_num错误，软件可能core_dump: " << static_cast<int>(group_num) << std::endl;
            }
            of_B += 1;
            uint8_t row_addr = raw_data[of_B];
            of_B += 1;
            // 开始进行一行数据的解码过程
            for (int i = group_num; i > 0; i--) {
                // group_num最高位为1
                if ((raw_data[of_B] & 0x80) == 0x80) {
                    uint8_t col_addr = raw_data[of_B] & 0x7F;
                    col_addr = col_addr * 2;
                    if (col_addr > 160) {
                        std::cout << "左sdl:col_addr错误，软件可能core_dump: " << static_cast<int>(col_addr) << std::endl;
                    }
                    of_B += 1;
                    sdl_left_data_.rod_data[row_addr][col_addr] = static_cast<int8_t>(raw_data[of_B]);
                    sdl_left_data_.rod_data[row_addr][col_addr + 1] = static_cast<int8_t>(raw_data[of_B + 1]);
                    of_B += 3;
                } else {
                    std::cout << "左sdl,group数据错误" << std::endl;
                    of_B += 4;
                }
            }
            sdl_left_data_.valid = true;
        } 
        else if (raw_data[of_B] == 0x2C) {
            // std::cout << "左sdr一行" << std::endl;
            of_B += 1;
            // 这里group_num放置的是否正确？
            uint8_t high7_bits = raw_data[of_B] & 0xFE;
            uint8_t group_num = high7_bits >> 1;
            if (group_num > 80) {
                std::cout << "左sdr:group_num错误，软件可能core_dump: " << static_cast<int>(group_num) << std::endl;
            }
            of_B += 1;
            uint8_t row_addr = raw_data[of_B];
            of_B += 1;
            // 开始进行一行数据的解码过程
            for (int i = group_num; i > 0; i--) {
                // group_num最高位为1
                if ((raw_data[of_B] & 0x80) == 0x80) {
                    uint8_t col_addr = raw_data[of_B] & 0x7F;
                    col_addr = col_addr * 2;
                    if (col_addr > 160) {
                        std::cout << "左sdr:col_addr错误，软件可能core_dump: " << static_cast<int>(col_addr) << std::endl;
                    }
                    of_B += 1;
                    sdr_left_data_.rod_data[row_addr][col_addr] = static_cast<int8_t>(raw_data[of_B]);
                    sdr_left_data_.rod_data[row_addr][col_addr + 1] = static_cast<int8_t>(raw_data[of_B + 1]);
                    of_B += 3;
                } else {
                    std::cout << "左sdr,group数据错误" << std::endl;
                    of_B += 4;
                }
            }
            sdr_left_data_.valid = true;
        }
    }
    else if ((raw_data[of_B] & 0xF0) == 0x30) {
        // std::cout << "ROD右" << std::endl;
         // 判断是不是FN数据
        if ((raw_data[of_B] & 0xFE) == 0x34) {
            // std::cout << "右FN" << std::endl;
            // 后面先不解析，直接跳过
            of_B += 3;

            if ((td_right_data_.valid) & (td_right_data_.system_time != 0)) {
                // 更新到缓冲区
                if (td_right_buffer_.update(td_right_data_)) {
                    // std::cout << "右TD数据更新" << std::endl;
                    td_right_data_.valid = false;
                    // td_right_data_.rod_data.clear();
                    td_right_data_.rod_data = std::vector<std::vector<int8_t>>(160, std::vector<int8_t>(160, 0));

                    td_right_sq_ += 1;
                    td_right_data_.sequence = td_right_sq_;

                    // std::cout << "[Producer] 发送数据 #" << imu_data.sequence;
                    // std::cout << " (更新计数: " << imu_buffer_.getUpdateCount() << ")" << std::endl;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } else {
                    std::cout << "[Producer] 数据验证失败，丢弃" << std::endl;
                }

            }

            if (sdl_right_data_.valid & (sdl_right_data_.system_time != 0)) {
                if (sdl_right_buffer_.update(sdl_right_data_)) {
                    sdl_right_data_.valid = false;
                    // sdl_right_data_.rod_data.clear();
                    sdl_right_data_.rod_data = std::vector<std::vector<int8_t>>(160, std::vector<int8_t>(160, 0));

                    sdl_right_sq_ += 1;
                    sdl_right_data_.sequence = sdl_right_sq_;

                    // std::cout << "[Producer] 发送数据 #" << imu_data.sequence;
                    // std::cout << " (更新计数: " << imu_buffer_.getUpdateCount() << ")" << std::endl;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } else {
                    std::cout << "[Producer] 数据验证失败，丢弃" << std::endl;
                }
            }

            if (sdr_right_data_.valid & (sdr_right_data_.system_time != 0)) {
                if (sdr_right_buffer_.update(sdr_right_data_)) {
                    sdr_right_data_.valid = false;
                    // sdr_right_data_.rod_data.clear();
                    sdr_right_data_.rod_data = std::vector<std::vector<int8_t>>(160, std::vector<int8_t>(160, 0));

                    sdr_right_sq_ += 1;
                    sdr_right_data_.sequence = sdr_right_sq_;

                    // std::cout << "[Producer] 发送数据 #" << imu_data.sequence;
                    // std::cout << " (更新计数: " << imu_buffer_.getUpdateCount() << ")" << std::endl;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } else {
                    std::cout << "[Producer] 数据验证失败，丢弃" << std::endl;
                }
            }

            uint64_t time_step = 0;
            uint32_t time_step_low = 0;
            uint32_t time_step_high = 0;

            for (int i = 3; i >= 0; i--) {
                time_step_low |= static_cast<uint32_t>(raw_data[of_B + i]) << (8 * (3-i));
            }
            for (int i = 7; i >= 4; i--) {
                time_step_high |= static_cast<uint32_t>(raw_data[of_B + i]) << (8 * (7-i));
            }

            time_step |= static_cast<uint64_t>(time_step_low);
            time_step |= static_cast<uint64_t>(time_step_high) << 32;

            sdl_right_data_.system_time = time_step;
            sdr_right_data_.system_time = time_step;
            td_right_data_.system_time = time_step;
            of_B += 8;

        } 
        else if (raw_data[of_B] == 0x30) {
            // std::cout << "右TD一行" << std::endl;
            of_B += 1;
            // 这里group_num放置的是否正确？
            uint8_t high7_bits = raw_data[of_B] & 0xFE;
            uint8_t group_num = high7_bits >> 1;
            if (group_num > 80) {
                std::cout << "右TD:group_num错误，软件可能core_dump: " << static_cast<int>(group_num) << std::endl;
            }
            of_B += 1;
            uint8_t row_addr = raw_data[of_B];
            // std::cout << "row_addr: " << static_cast<int>(row_addr) << std::endl;
            of_B += 1;
            // 开始进行一行数据的解码过程
            for (int i = group_num; i > 0; i--) {
                // group_num最高位为1
                if ((raw_data[of_B] & 0x80) == 0x80) {
                    uint8_t col_addr = raw_data[of_B] & 0x7F;
                    col_addr = col_addr * 2;
                    if (col_addr > 160) {
                        std::cout << "右TD:col_addr错误，软件可能core_dump: " << static_cast<int>(col_addr) << std::endl;
                    }
                    of_B += 1;
                    td_right_data_.rod_data[row_addr][col_addr] = static_cast<int8_t>(raw_data[of_B]);
                    td_right_data_.rod_data[row_addr][col_addr + 1] = static_cast<int8_t>(raw_data[of_B + 1]);
                    of_B += 3;
                } else {
                    std::cout << "右TD,group数据错误" << std::endl;
                    of_B += 4;
                }
            }
            td_right_data_.valid = true;
        }
        else if (raw_data[of_B] == 0x38) {
            // std::cout << "右sdl一行" << std::endl;
            of_B += 1;
            // 这里group_num放置的是否正确？
            uint8_t high7_bits = raw_data[of_B] & 0xFE;
            uint8_t group_num = high7_bits >> 1;
            if (group_num > 80) {
                std::cout << "右sdl:group_num错误，软件可能core_dump: " << static_cast<int>(group_num) << std::endl;
            }
            of_B += 1;
            uint8_t row_addr = raw_data[of_B];
            of_B += 1;
            // 开始进行一行数据的解码过程
            for (int i = group_num; i > 0; i--) {
                // group_num最高位为1
                if ((raw_data[of_B] & 0x80) == 0x80) {
                    uint8_t col_addr = raw_data[of_B] & 0x7F;
                    col_addr = col_addr * 2;
                    if (col_addr > 160) {
                        std::cout << "右sdl:col_addr错误，软件可能core_dump: " << static_cast<int>(col_addr) << std::endl;
                    }
                    of_B += 1;
                    sdl_right_data_.rod_data[row_addr][col_addr] = static_cast<int8_t>(raw_data[of_B]);
                    sdl_right_data_.rod_data[row_addr][col_addr + 1] = static_cast<int8_t>(raw_data[of_B + 1]);
                    of_B += 3;
                } else {
                    std::cout << "右sdl,group数据错误" << std::endl;
                    of_B += 4;
                }
            }
            sdl_right_data_.valid = true;
        } 
        else if (raw_data[of_B] == 0x3C) {
            // std::cout << "右sdr一行" << std::endl;
            of_B += 1;
            // 这里group_num放置的是否正确？
            uint8_t high7_bits = raw_data[of_B] & 0xFE;
            uint8_t group_num = high7_bits >> 1;
            if (group_num > 80) {
                std::cout << "右sdr:group_num错误，软件可能core_dump: " << static_cast<int>(group_num) << std::endl;
            }
            of_B += 1;
            uint8_t row_addr = raw_data[of_B];
            of_B += 1;
            // 开始进行一行数据的解码过程
            for (int i = group_num; i > 0; i--) {
                // group_num最高位为1
                if ((raw_data[of_B] & 0x80) == 0x80) {
                    uint8_t col_addr = raw_data[of_B] & 0x7F;
                    col_addr = col_addr * 2;
                    if (col_addr > 160) {
                        std::cout << "右sdr:col_addr错误，软件可能core_dump: " << static_cast<int>(col_addr) << std::endl;
                    }
                    of_B += 1;
                    sdr_right_data_.rod_data[row_addr][col_addr] = static_cast<int8_t>(raw_data[of_B]);
                    sdr_right_data_.rod_data[row_addr][col_addr + 1] = static_cast<int8_t>(raw_data[of_B + 1]);
                    of_B += 3;
                } else {
                    std::cout << "右sdr,group数据错误" << std::endl;
                    of_B += 4;
                }
            }
            sdr_right_data_.valid = true;
        }
        else {
            std::cout << "rod数据错误" << std::endl;
        }
    }
    else {
        std::cerr << "ROD左右目格式错误！！！ " << std::endl;
        return DecodeErrorCode::ERR_ROD_DATA;
    }

    return DecodeErrorCode::SUCCESS;

}

DecodeErrorCode DecodePacket::decodeIMU(uint8_t* raw_data, IMUData& out_data){
    size_t offset = 0;

    // 5. 解析Tag值
    out_data.tag = raw_data[offset];
    offset += 1;
    if (out_data.tag != 0x91) { // 仅支持0x91标签的数据包
        return DecodeErrorCode::ERR_UNKNOWN_IMU_TAG;
    }

    // 6. 解析Payload数据域（严格按图片字段顺序）
    // 6.1 保留字段（uint16_t）
    out_data.reserved = le16_to_host(raw_data + offset);
    offset += 2;

    // 6.2 温度（int8_t）
    out_data.temperature = static_cast<int8_t>(raw_data[offset]);
    offset += 1;

    // 6.3 气压（float，4字节）
    out_data.air_pressure = le_bytes_to_float(raw_data + offset);
    offset += 4;

    // 6.4 时间戳（uint32_t）
    // out_data.system_time = le32_to_host(raw_data + offset);
    offset += 4;

    // 6.5 加速度X/Y/Z（float×3，各4字节）
    out_data.acc_b_x = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.acc_b_y = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.acc_b_z = le_bytes_to_float(raw_data + offset);
    offset += 4;

    // 6.6 角速度X/Y/Z（float×3，各4字节）
    out_data.gyr_b_x = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.gyr_b_y = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.gyr_b_z = le_bytes_to_float(raw_data + offset);
    offset += 4;

    // 6.7 磁场X/Y/Z（float×3，各4字节）
    out_data.mag_b_x = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.mag_b_y = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.mag_b_z = le_bytes_to_float(raw_data + offset);
    offset += 4;

    // 6.8 姿态角（横滚/俯仰/航向，float×3，各4字节）
    out_data.roll = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.pitch = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.yaw = le_bytes_to_float(raw_data + offset);
    offset += 4;

    // 6.9 四元数W/X/Y/Z（float×4，各4字节）
    out_data.q_w = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.q_x = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.q_y = le_bytes_to_float(raw_data + offset);
    offset += 4;
    out_data.q_z = le_bytes_to_float(raw_data + offset);
    offset += 4;

    // printIMU(out_data);

    return DecodeErrorCode::SUCCESS;
}