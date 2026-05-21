#include <libusb-1.0/libusb.h>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "Device.h"
#include "DecodePacket.h"

// ================= buffer =================
std::deque<RGBData> rgb_right_buf;
std::mutex rgb_right_mtx;

std::deque<RGBData> rgb_left_buf;
std::mutex rgb_left_mtx;

std::deque<RODData> td_right_buf;
std::mutex td_right_mtx;

std::deque<RODData> td_left_buf;
std::mutex td_left_mtx;

std::deque<RODData> sdl_right_buf;
std::mutex sdl_right_mtx;

std::deque<RODData> sdr_right_buf;
std::mutex sdr_right_mtx;

std::deque<RODData> sdl_left_buf;
std::mutex sdl_left_mtx;

std::deque<RODData> sdr_left_buf;
std::mutex sdr_left_mtx;

std::deque<IMUData> imu_buf;
std::mutex imu_mtx;

std::atomic<bool> running{true};

cv::Mat rgb_to_mat(const RGBData& frame)
{
    int rows = frame.pixel_rows.size();
    int cols = frame.pixel_rows[0].size(); // 假设每行是RGB连续数据
    cv::Mat mat(rows, cols, CV_8UC3);
    for (int i = 0; i < rows; i++) {
        memcpy(mat.ptr(i), frame.pixel_rows[i].data(), cols * 3);
    }
    return mat;
}

inline void whiteBalanceGrayWorldInplace(cv::Mat& bgr_img)
{
    CV_Assert(bgr_img.type() == CV_8UC3);

    // 转 float（避免溢出）
    cv::Mat float_img;
    bgr_img.convertTo(float_img, CV_32FC3);

    // 计算均值（直接整体算，避免 split）
    cv::Scalar mean = cv::mean(float_img);

    float mean_b = mean[0];
    float mean_g = mean[1];
    float mean_r = mean[2];

    // 防止除0
    const float eps = 1e-6f;

    float gain_r = mean_g / (mean_r + eps);
    float gain_b = mean_g / (mean_b + eps);

    // ===== 原地操作（无 split）=====
    for (int y = 0; y < float_img.rows; ++y)
    {
        cv::Vec3f* ptr = float_img.ptr<cv::Vec3f>(y);
        for (int x = 0; x < float_img.cols; ++x)
        {
            ptr[x][0] *= gain_b; // B
            // G 不变
            ptr[x][2] *= gain_r; // R
        }
    }

    // 截断并转回 uint8
    cv::threshold(float_img, float_img, 255.0, 255.0, cv::THRESH_TRUNC);
    cv::threshold(float_img, float_img, 0.0, 0.0, cv::THRESH_TOZERO);

    float_img.convertTo(bgr_img, CV_8UC3);
}

// ================= RGB线程 =================
void rgb_thread(DecodePacket* packet)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    int synced_frame_count = 0;

    const double interval = 1.0;

    RGBData last_left, last_right;
    bool has_left = false;
    bool has_right = false;

    while (running)
    {
        RGBData rgb_left_data, rgb_right_data;

        // ===== Right =====
        if (packet->getRGBRightData(rgb_right_data))
        {
            {
                std::lock_guard<std::mutex> lk(rgb_right_mtx);
                rgb_right_buf.push_back(rgb_right_data);
                if (rgb_right_buf.size() > 5)
                    rgb_right_buf.pop_front();
            }

            last_right = rgb_right_data;
            has_right = true;
        }

        // ===== Left =====
        if (packet->getRGBLeftData(rgb_left_data))
        {
            {
                std::lock_guard<std::mutex> lk(rgb_left_mtx);
                rgb_left_buf.push_back(rgb_left_data);
                if (rgb_left_buf.size() > 5)
                    rgb_left_buf.pop_front();
            }

            last_left = rgb_left_data;
            has_left = true;
        }

        // ===== 同步拼接（核心）=====
        if (has_left && has_right &&
            last_left.sequence == last_right.sequence)
        {
            // 转图
            cv::Mat left_m = rgb_to_mat(last_left);
            cv::Mat right_m = rgb_to_mat(last_right);

            // cv::cvtColor(left_m, left_m, cv::COLOR_RGB2BGR);
            // cv::cvtColor(right_m, right_m, cv::COLOR_RGB2BGR);
            whiteBalanceGrayWorldInplace(left_m);
            whiteBalanceGrayWorldInplace(right_m);

            cv::resize(left_m, left_m, cv::Size(320, 160));
            cv::resize(right_m, right_m, cv::Size(320, 160));
            cv::flip(left_m, left_m, 1);
            cv::flip(right_m, right_m, 1);

            // 拼接
            cv::Mat stitched;
            cv::hconcat(left_m, right_m, stitched);

            cv::imshow("RGB Stereo Viewer", stitched);

            synced_frame_count++;

            // 防止重复使用同一帧
            has_left = false;
            has_right = false;
        }

        // ===== FPS统计（真实RGB帧率）=====
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();

        if (elapsed >= interval)
        {
            double rgb_fps = synced_frame_count / elapsed;

            std::cout << "[RGB Stereo FPS]: "
                      << std::fixed << std::setprecision(1)
                      << rgb_fps << std::endl;

            synced_frame_count = 0;
            start_time = now;
        }

        // ===== 退出 =====
        if (cv::waitKey(1) == 27)
        {
            running = false;
            break;
        }
    }
}

struct SDParam {
    int var_fil_ksize;
    float var_th;
    int adapt_th_min;
    int adapt_th_max;
};

cv::Mat localVar(const cv::Mat& src, int ksize, float eps, float var_th)
{
    CV_Assert(src.type() == CV_32F);

    cv::Mat mean, mean_sq;
    cv::Mat src_sq = src.mul(src);

    cv::blur(src, mean, cv::Size(ksize, ksize), cv::Point(-1,-1), cv::BORDER_CONSTANT);
    cv::blur(src_sq, mean_sq, cv::Size(ksize, ksize), cv::Point(-1,-1), cv::BORDER_CONSTANT);

    cv::Mat var = mean_sq - mean.mul(mean);

    cv::Mat result = cv::Mat::zeros(src.size(), CV_32F);

    for (int y = 0; y < src.rows; y++) {
        const float* s_ptr = src.ptr<float>(y);
        const float* v_ptr = var.ptr<float>(y);
        float* r_ptr = result.ptr<float>(y);
        for (int x = 0; x < src.cols; x++) {
            r_ptr[x] = (v_ptr[x] > var_th ? s_ptr[x] : 0.0f);
        }
    }

    return result;
}

// ---------- SD 去噪函数 ----------
void diff_denoise(cv::Mat& diff, const SDParam& p)
{
    CV_Assert(diff.type() == CV_8SC1);

    int rows = diff.rows;
    int cols = diff.cols;

    // 转 float
    cv::Mat diff_f;
    diff.convertTo(diff_f, CV_32F);

    // 局部方差滤波
    cv::Mat out_f = localVar(diff_f, p.var_fil_ksize, 0.5f, p.var_th);

    // 转回 int8
    for (int y = 0; y < rows; y++) {
        const float* in_ptr = out_f.ptr<float>(y);
        int8_t* out_ptr = diff.ptr<int8_t>(y);

        for (int x = 0; x < cols; x++) {
            int val = static_cast<int>(std::round(in_ptr[x]));
            out_ptr[x] = static_cast<int8_t>(
                val < -128 ? -128 : (val > 127 ? 127 : val)
            );
        }
    }
}

cv::Mat rod_to_mat(RODData& frame)
{
    int rows = 160;
    int cols = 160;

    // === 1. 转成 Mat ===
    cv::Mat diff_mat(rows, cols, CV_8SC1);
    for (int y = 0; y < rows; ++y) {
        memcpy(diff_mat.ptr<int8_t>(y), frame.rod_data[y].data(), cols);
    }

    // === 2. 降噪 ===
    SDParam sd_param = {3, 4.0f, 3, 9};
    diff_denoise(diff_mat, sd_param);

    // 直接返回单通道 int8 矩阵
    return diff_mat;
}

void diff_thread(DecodePacket* packet)
{
    int loop_count = 0;
    const double interval = 1.0;
    auto start_time = std::chrono::high_resolution_clock::now();

    // 最新组帧数据
    cv::Mat td_right_mat, td_left_mat;
    cv::Mat sdl_right_mat, sdr_right_mat;
    cv::Mat sdl_left_mat, sdr_left_mat;

    // 输出三通道 Diff 图像
    cv::Mat Diff_left, Diff_right;

    while (running)
    {
        bool got_frame = false;

        // ===== TDRight =====
        RODData td_right_data;
        if (packet->getTDRightData(td_right_data))
        {
            std::lock_guard<std::mutex> lk(td_right_mtx);
            td_right_buf.push_back(td_right_data);
            if (td_right_buf.size() > 5) td_right_buf.pop_front();
            td_right_mat = rod_to_mat(td_right_data);
            got_frame = true;
        }

        // ===== TDLeft =====
        RODData td_left_data;
        if (packet->getTDLeftData(td_left_data))
        {
            std::lock_guard<std::mutex> lk(td_left_mtx);
            td_left_buf.push_back(td_left_data);
            if (td_left_buf.size() > 5) td_left_buf.pop_front();
            td_left_mat = rod_to_mat(td_left_data);
            got_frame = true;
        }

        // ===== SDLRight =====
        RODData sdl_right_data;
        if (packet->getSDLRightData(sdl_right_data))
        {
            std::lock_guard<std::mutex> lk(sdl_right_mtx);
            sdl_right_buf.push_back(sdl_right_data);
            if (sdl_right_buf.size() > 5) sdl_right_buf.pop_front();
            sdl_right_mat = rod_to_mat(sdl_right_data);
            got_frame = true;
        }

        // ===== SDLLeft =====
        RODData sdl_left_data;
        if (packet->getSDLLeftData(sdl_left_data))
        {
            std::lock_guard<std::mutex> lk(sdl_left_mtx);
            sdl_left_buf.push_back(sdl_left_data);
            if (sdl_left_buf.size() > 5) sdl_left_buf.pop_front();
            sdl_left_mat = rod_to_mat(sdl_left_data);
            got_frame = true;
        }

        // ===== SDRRight =====
        RODData sdr_right_data;
        if (packet->getSDRRightData(sdr_right_data))
        {
            std::lock_guard<std::mutex> lk(sdr_right_mtx);
            sdr_right_buf.push_back(sdr_right_data);
            if (sdr_right_buf.size() > 5) sdr_right_buf.pop_front();
            sdr_right_mat = rod_to_mat(sdr_right_data);
            got_frame = true;
        }

        // ===== SDRLeft =====
        RODData sdr_left_data;
        if (packet->getSDRLeftData(sdr_left_data))
        {
            std::lock_guard<std::mutex> lk(sdr_left_mtx);
            sdr_left_buf.push_back(sdr_left_data);
            if (sdr_left_buf.size() > 5) sdr_left_buf.pop_front();
            sdr_left_mat = rod_to_mat(sdr_left_data);
            got_frame = true;
        }

        // ===== 生成左右目 Diff 图像 =====
        if (!td_left_mat.empty() && !sdl_left_mat.empty() && !sdr_left_mat.empty())
        {
            std::vector<cv::Mat> channels_left = {td_left_mat, sdl_left_mat, sdr_left_mat};
            cv::merge(channels_left, Diff_left);
        }

        if (!td_right_mat.empty() && !sdl_right_mat.empty() && !sdr_right_mat.empty())
        {
            std::vector<cv::Mat> channels_right = {td_right_mat, sdl_right_mat, sdr_right_mat};
            cv::merge(channels_right, Diff_right);
        }

        // ===== FPS统计 =====
        if (got_frame)
            loop_count++;

        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed >= interval)
        {
            double fps = loop_count / 2 / elapsed; // 每秒更新平均 Diff FPS
            std::cout << "[Diff Thread] Avg Diff FPS: "
                      << std::fixed << std::setprecision(1)
                      << fps << std::endl;

            loop_count = 0;
            start_time = now;
        }
    }
}

// ================= IMU线程 =================
void imu_thread(DecodePacket* packet)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    int loop_count = 0;
    int imu_count = 0;
    const double interval = 1.0;

    while (running)
    {
        loop_count++;

        IMUData imu_data;

        if (packet->getIMUData(imu_data))
        {
            imu_count++;

            std::lock_guard<std::mutex> lk(imu_mtx);
            imu_buf.push_back(imu_data);
            if (imu_buf.size() > 500)
                imu_buf.pop_front();
        }

        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();

        if (elapsed >= interval)
        {
            double loop_fps = loop_count / elapsed;
            double imu_fps  = imu_count / elapsed;

            std::cout << "[IMU Thread]"
                      << "IMU FPS: "
                      << imu_fps
                      << std::endl;

            loop_count = 0;
            imu_count = 0;
            start_time = now;
        }
    }
}


// ================= main =================
int main()
{
    FrameBuffer frame_buffer(1024 * 16);

    Device dev{
        frame_buffer,
        uint16_t(0x04b4),
        uint16_t(0x00f1),
        uint8_t(0x01),
        uint8_t(0x81)
    };

    dev.StartCollect();

    DecodePacket packet(frame_buffer, dev, 1024 * 16);
    packet.start();

    std::thread t_rgb(rgb_thread, &packet);
    std::thread t_diff(diff_thread, &packet);
    std::thread t_imu(imu_thread, &packet);

    t_rgb.join();
    t_diff.join();
    t_imu.join();

    return 0;
}