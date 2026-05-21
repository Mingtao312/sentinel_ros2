// sentinel_node.cpp
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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cv_bridge/cv_bridge.h"

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

// ================= util =================
cv::Mat rgb_to_mat(const RGBData& frame)
{
    int rows = frame.pixel_rows.size();
    int cols = frame.pixel_rows[0].size();
    cv::Mat mat(rows, cols, CV_8UC3);
    for (int i = 0; i < rows; i++) {
        memcpy(mat.ptr(i), frame.pixel_rows[i].data(), cols * 3);
    }
    return mat;
}

inline void whiteBalanceGrayWorldInplace(cv::Mat& bgr_img)
{
    CV_Assert(bgr_img.type() == CV_8UC3);
    cv::Mat float_img;
    bgr_img.convertTo(float_img, CV_32FC3);
    cv::Scalar mean = cv::mean(float_img);
    float mean_b = mean[0], mean_g = mean[1], mean_r = mean[2];
    const float eps = 1e-6f;
    float gain_r = mean_g / (mean_r + eps);
    float gain_b = mean_g / (mean_b + eps);

    for (int y = 0; y < float_img.rows; ++y)
    {
        cv::Vec3f* ptr = float_img.ptr<cv::Vec3f>(y);
        for (int x = 0; x < float_img.cols; ++x)
        {
            ptr[x][0] *= gain_b;
            ptr[x][2] *= gain_r;
        }
    }
    cv::threshold(float_img, float_img, 255.0, 255.0, cv::THRESH_TRUNC);
    cv::threshold(float_img, float_img, 0.0, 0.0, cv::THRESH_TOZERO);
    float_img.convertTo(bgr_img, CV_8UC3);
}

// ================= SD 相关 =================
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
    cv::blur(src, mean, cv::Size(ksize, ksize));
    cv::blur(src_sq, mean_sq, cv::Size(ksize, ksize));
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

void diff_denoise(cv::Mat& diff, const SDParam& p)
{
    CV_Assert(diff.type() == CV_8SC1);
    cv::Mat diff_f;
    diff.convertTo(diff_f, CV_32F);
    cv::Mat out_f = localVar(diff_f, p.var_fil_ksize, 0.5f, p.var_th);
    diff_f = out_f;
    diff_f.convertTo(diff, CV_8SC1);
}

cv::Mat rod_to_mat(RODData& frame)
{
    int rows = 160, cols = 160;
    cv::Mat diff_mat(rows, cols, CV_8SC1);
    for (int y = 0; y < rows; ++y)
        memcpy(diff_mat.ptr<int8_t>(y), frame.rod_data[y].data(), cols);
    SDParam sd_param = {3, 4.0f, 3, 9};
    diff_denoise(diff_mat, sd_param);
    return diff_mat;
}

void visualizeDiff(const cv::Mat& diff, const std::string& win_name,
                   float gain = 3.5f, float thresh = 3.0f)
{
    if (diff.empty()) {
        std::cout << win_name << " is empty!" << std::endl;
        return;
    }

    CV_Assert(diff.type() == CV_8SC3);

    int rows = diff.rows;
    int cols = diff.cols;

    cv::Mat vis(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));

    const cv::Vec3b navyBlue(120, 80, 40);
    const cv::Vec3b white(255, 255, 255);

    for (int y = 0; y < rows; ++y)
    {
        const cv::Vec3b* d_ptr = diff.ptr<cv::Vec3b>(y); // 实际是int8，要转换
        uchar* pRow = vis.ptr<uchar>(y);

        for (int x = 0; x < cols; ++x)
        {
            // 取三个通道（TD / SDL / SDR）
            int8_t td  = reinterpret_cast<const int8_t*>(d_ptr)[3*x + 0];
            int8_t sdl = reinterpret_cast<const int8_t*>(d_ptr)[3*x + 1];
            int8_t sdr = reinterpret_cast<const int8_t*>(d_ptr)[3*x + 2];

            float diff_val = td + sdl + sdr; // 简单叠加用于可视化

            // 正极性
            if (diff_val > thresh)
            {
                float intensity = (diff_val - thresh) * gain;
                float alpha = std::min(1.0f, intensity / 255.0f);
                alpha = std::pow(alpha, 0.7f);

                pRow[3*x+0] = cv::saturate_cast<uchar>(white[0] - alpha * (white[0] - navyBlue[0]));
                pRow[3*x+1] = cv::saturate_cast<uchar>(white[1] - alpha * (white[1] - navyBlue[1]));
                pRow[3*x+2] = cv::saturate_cast<uchar>(white[2] - alpha * (white[2] - navyBlue[2]));
            }
            // 负极性
            else if (diff_val < -thresh)
            {
                float intensity = (std::abs(diff_val) - thresh) * gain;
                float alpha = std::min(1.0f, intensity / 255.0f);
                alpha = std::pow(alpha, 0.6f);

                uchar dark = cv::saturate_cast<uchar>(255.0f * (1.0f - alpha));

                pRow[3*x+0] = dark;
                pRow[3*x+1] = dark;
                pRow[3*x+2] = dark;
            }
        }
    }

    cv::imshow(win_name, vis);
    cv::waitKey(1);
}

inline rclcpp::Time toRosTime(uint64_t t)
{
    // ===== 根据你的设备修改这里 =====
    // 如果是纳秒：
    return rclcpp::Time(t);

    // 如果是微秒：
    // return rclcpp::Time(t * 1000);

    // 如果是毫秒：
    // return rclcpp::Time(t * 1000000);
}

void decode_thread(DecodePacket* packet, rclcpp::Node::SharedPtr node)
{
    // ===== publishers =====
    auto pub_left_rgb  = node->create_publisher<sensor_msgs::msg::Image>("/camera/left/rgb", 10);
    auto pub_right_rgb = node->create_publisher<sensor_msgs::msg::Image>("/camera/right/rgb", 10);
    auto pub_left_diff  = node->create_publisher<sensor_msgs::msg::Image>("/camera/left/diff", 10);
    auto pub_right_diff = node->create_publisher<sensor_msgs::msg::Image>("/camera/right/diff", 10);
    auto pub_imu = node->create_publisher<sensor_msgs::msg::Imu>("/imu", 100);

    // ===== RGB cache =====
    RGBData last_left, last_right;
    bool has_left = false, has_right = false;

    // ===== Diff cache & flags =====
    cv::Mat td_left_mat, td_right_mat, sdl_left_mat, sdl_right_mat, sdr_left_mat, sdr_right_mat;
    bool new_td_left = false, new_sdl_left = false, new_sdr_left = false;
    bool new_td_right = false, new_sdl_right = false, new_sdr_right = false;

    while (running)
    {
        bool got_any = false;

        // ================= IMU =================
        IMUData imu_data;
        if (packet->getIMUData(imu_data))
        {
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = toRosTime(imu_data.system_time);
            imu_msg.linear_acceleration.x = imu_data.acc_b_x;
            imu_msg.linear_acceleration.y = imu_data.acc_b_y;
            imu_msg.linear_acceleration.z = imu_data.acc_b_z;
            imu_msg.angular_velocity.x = imu_data.gyr_b_x;
            imu_msg.angular_velocity.y = imu_data.gyr_b_y;
            imu_msg.angular_velocity.z = imu_data.gyr_b_z;
            pub_imu->publish(imu_msg);
            got_any = true;
        }

        // ================= RGB =================
        RGBData rgb_left_data, rgb_right_data;
        if (packet->getRGBLeftData(rgb_left_data))   { last_left = rgb_left_data;  has_left = true; got_any = true; }
        if (packet->getRGBRightData(rgb_right_data)) { last_right = rgb_right_data; has_right = true; got_any = true; }

        if (has_left && has_right && last_left.sequence == last_right.sequence)
        {
            cv::Mat left_m = rgb_to_mat(last_left);
            cv::Mat right_m = rgb_to_mat(last_right);

            whiteBalanceGrayWorldInplace(left_m);
            whiteBalanceGrayWorldInplace(right_m);

            cv::cvtColor(left_m, left_m, cv::COLOR_RGB2BGR);
            cv::cvtColor(right_m, right_m, cv::COLOR_RGB2BGR);

            cv::resize(left_m, left_m, cv::Size(320, 160));
            cv::resize(right_m, right_m, cv::Size(320, 160));

            cv::flip(left_m, left_m, 1);
            cv::flip(right_m, right_m, 1);

            auto left_msg  = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", left_m).toImageMsg();
            auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", right_m).toImageMsg();

            left_msg->header.stamp  = toRosTime(last_left.system_time);
            right_msg->header.stamp = toRosTime(last_right.system_time);

            pub_left_rgb->publish(*left_msg);
            pub_right_rgb->publish(*right_msg);

            has_left = has_right = false;
        }

        // ================= Diff =================
        RODData td_right_data, td_left_data, sdl_right_data, sdl_left_data, sdr_right_data, sdr_left_data;

        if (packet->getTDRightData(td_right_data))   { td_right_mat = rod_to_mat(td_right_data); new_td_right = true; got_any = true; }
        if (packet->getTDLeftData(td_left_data))     { td_left_mat  = rod_to_mat(td_left_data);  new_td_left = true;  got_any = true; }
        if (packet->getSDLRightData(sdl_right_data)) { sdl_right_mat = rod_to_mat(sdl_right_data); new_sdl_right = true; got_any = true; }
        if (packet->getSDLLeftData(sdl_left_data))   { sdl_left_mat  = rod_to_mat(sdl_left_data);  new_sdl_left = true;  got_any = true; }
        if (packet->getSDRRightData(sdr_right_data)) { sdr_right_mat = rod_to_mat(sdr_right_data); new_sdr_right = true; got_any = true; }
        if (packet->getSDRLeftData(sdr_left_data))   { sdr_left_mat  = rod_to_mat(sdr_left_data);  new_sdr_left = true;  got_any = true; }

        // ===== publish left diff =====
        if (new_td_left && new_sdl_left && new_sdr_left)
        {
            cv::Mat Diff_left;
            cv::merge(std::vector<cv::Mat>{td_left_mat, sdl_left_mat, sdr_left_mat}, Diff_left);
            Diff_left.convertTo(Diff_left, CV_32FC3);

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC3", Diff_left).toImageMsg();
            msg->header.stamp = toRosTime(td_left_data.system_time);
            pub_left_diff->publish(*msg);

            new_td_left = new_sdl_left = new_sdr_left = false;
        }

        // ===== publish right diff =====
        if (new_td_right && new_sdl_right && new_sdr_right)
        {
            cv::Mat Diff_right;
            cv::merge(std::vector<cv::Mat>{td_right_mat, sdl_right_mat, sdr_right_mat}, Diff_right);
            Diff_right.convertTo(Diff_right, CV_32FC3);

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC3", Diff_right).toImageMsg();
            msg->header.stamp = toRosTime(td_right_data.system_time);
            pub_right_diff->publish(*msg);

            new_td_right = new_sdl_right = new_sdr_right = false;
        }

        // ================= idle sleep =================
        if (!got_any)
            std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

// ================= main =================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sentinel_node");

    FrameBuffer frame_buffer(1024 * 16);
    Device dev{ frame_buffer, uint16_t(0x04b4), uint16_t(0x00f1), uint8_t(0x01), uint8_t(0x81) };
    dev.StartCollect();

    DecodePacket packet(frame_buffer, dev, 1024*16);
    packet.start();

    std::thread t_decode(decode_thread, &packet, node);

    rclcpp::spin(node);

    running = false;
    t_decode.join();

    rclcpp::shutdown();
    return 0;
}