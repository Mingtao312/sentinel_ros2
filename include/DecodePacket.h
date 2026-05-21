#pragma once

#include "FrameBuffer.h"
#include "Device.h"
#include "PingPongBuffer.h"

struct IMUData {
    uint32_t sequence = 0;        // 序列号
    bool valid = false;               // 数据有效性

    uint8_t tag = 0x91;
    uint16_t reserved = 2323;

    int8_t temperature;
    float air_pressure;
    uint64_t system_time;
    // 加速度
    float acc_b_x;
    float acc_b_y;
    float acc_b_z;
    // 角速度
    float gyr_b_x;
    float gyr_b_y;
    float gyr_b_z;
    // 磁场
    float mag_b_x;
    float mag_b_y;
    float mag_b_z;

    float roll;
    float pitch;
    float yaw;

    float q_w;
    float q_x;
    float q_y;
    float q_z;

    // 用于PingPongBuffer验证
    static bool validate(const IMUData& data) {
        return data.valid;
    }

};

struct RGBPixel {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct RGBData {
    uint32_t sequence = 0;        // 序列号
    bool valid = false;           // 数据有效性
    bool left_right;              // left = 0; right = 1           
    uint64_t system_time;

    uint32_t width = 318;
    uint32_t height = 318;

    // std::vector<RGBPixel> pixels;
    std::vector<std::vector<RGBPixel>> pixel_rows;


    // 用于PingPongBuffer验证
    static bool validate(const RGBData& data) {
        return data.valid;
    }

};

struct RODData {
    uint32_t sequence = 0;        // 序列号
    bool valid = false;           // 数据有效性
    bool left_right;              // left = 0; right = 1           
    uint64_t system_time=0;
    
    uint32_t width = 160;
    uint32_t height = 160;

    std::vector<std::vector<int8_t>> rod_data{160, std::vector<int8_t>(160, 0)};

    // 用于PingPongBuffer验证
    static bool validate(const RODData& data) {
        return data.valid;
    }

};

// 解码错误码
enum class DecodeErrorCode {
    SUCCESS = 0,
    ERR_UNKNOWN_IMU_TAG = 2,      // 未知IMU Tag值
    ERR_ROD_DATA = 3,           // 不为ROD数据
};

// ========== 数据处理器类（独立封装） ==========
class DecodePacket {
public:
    DecodePacket(FrameBuffer& frameBuffer, Device& device, size_t frame_total_size);
    ~DecodePacket();

    // 启动解包线程
    void start();
    // 停止解包线程   
    void stop();

    // bool getIMUData(IMUData& imu_data, uint32_t seq);
    bool getIMUData(IMUData& imu_data);
    bool hasIMUData() const;

    bool getRGBLeftData(RGBData& rgb_data);
    bool getRGBRightData(RGBData& rgb_data);

    bool getSDLLeftData(RODData& rod_data);
    bool getSDRLeftData(RODData& rod_data);
    bool getTDLeftData(RODData& rod_data);

    bool getSDLRightData(RODData& rod_data);
    bool getSDRRightData(RODData& rod_data);
    bool getTDRightData(RODData& rod_data);

private:
    int imu_sq_ = 0;
    int rgb_left_sq_ = 0;
    int rgb_right_sq_ = 0;

    int td_left_sq_ = 0;
    int sdl_left_sq_ = 0;
    int sdr_left_sq_ = 0;

    int td_right_sq_ = 0;
    int sdl_right_sq_ = 0;
    int sdr_right_sq_ = 0;

    bool rgb_left_flag_ = false;
    bool rgb_left_change_ = false;

    bool rgb_right_flag_ = false;
    bool rgb_right_change_ = false;

    uint64_t last_imu_count_;
    uint64_t last_rgb_left_count_;
    uint64_t last_rgb_right_count_;

    uint64_t last_td_left_count_;
    uint64_t last_sdl_left_count_;
    uint64_t last_sdr_left_count_;

    uint64_t last_td_right_count_;
    uint64_t last_sdl_right_count_;
    uint64_t last_sdr_right_count_;
    
    // 解包线程函数
    void decodeWorker();
    DecodeErrorCode decodeIMU(uint8_t* pvalue, IMUData& out_data);
    DecodeErrorCode decodeRGB(uint8_t* raw_data, RGBData& out_data);
    DecodeErrorCode decodeROD(uint8_t* raw_data, int& of_B);
    void printIMU(IMUData& out_data);

    size_t frame_total_size_;
    FrameBuffer& frameBuffer_;
    Device& device_;
    std::thread* decodeThread_ = nullptr;

    RGBData rgb_left_data_;
    RGBData rgb_right_data_;

    RODData sdl_left_data_;
    RODData sdr_left_data_;
    RODData td_left_data_;

    RODData sdl_right_data_;
    RODData sdr_right_data_;
    RODData td_right_data_;

    PingPongBuffer<IMUData> imu_buffer_;
    PingPongBuffer<RGBData> rgb_left_buffer_;
    PingPongBuffer<RGBData> rgb_right_buffer_;

    PingPongBuffer<RODData> sdl_left_buffer_;
    PingPongBuffer<RODData> sdr_left_buffer_;
    PingPongBuffer<RODData> td_left_buffer_;

    PingPongBuffer<RODData> sdl_right_buffer_;
    PingPongBuffer<RODData> sdr_right_buffer_;
    PingPongBuffer<RODData> td_right_buffer_;
};
