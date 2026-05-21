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


// ================= 时间转换（核心） =================
inline rclcpp::Time deviceTimeToRosTime(uint64_t system_time)
{
    // 1. 定义一个静态偏移量，仅在第一次调用时初始化
    // 这个偏移量代表了：设备 0 时刻 对应的 系统 Unix 时间
    static uint64_t unix_base_ns = []() {
        auto now = std::chrono::system_clock::now();
        uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            now.time_since_epoch()).count();
        // 假设当前设备刚启动或我们取当前作为同步点
        // 实际工程中，建议从传感器发出的第一帧对时包里提取同步基准
        return now_ns;
    }();

    // 2. 计算设备自身的增量（20ns 步进）
    uint64_t device_inc_ns = system_time * 20ULL;

    // 3. 叠加得到绝对 Unix 时间
    uint64_t total_ns = unix_base_ns + device_inc_ns;

    uint64_t sec  = total_ns / 1000000000ULL;
    uint64_t nsec = total_ns % 1000000000ULL;

    return rclcpp::Time(sec, nsec);
}

// ================= RGB线程 =================
// void rgb_thread(DecodePacket* packet, rclcpp::Node::SharedPtr node)
// {
//     auto pub_left_rgb  = node->create_publisher<sensor_msgs::msg::Image>("/camera/left/rgb", 10);
//     auto pub_right_rgb = node->create_publisher<sensor_msgs::msg::Image>("/camera/right/rgb", 10);

//     RGBData last_left, last_right;
//     bool has_left = false, has_right = false;

//     while (running)
//     {
//         RGBData rgb_left_data, rgb_right_data;

//         if (packet->getRGBRightData(rgb_right_data)) { last_right = rgb_right_data; has_right = true; }
//         if (packet->getRGBLeftData(rgb_left_data))   { last_left  = rgb_left_data;  has_left  = true; }

//         if (has_left && has_right && last_left.sequence == last_right.sequence)
//         {
//             cv::Mat left_m  = rgb_to_mat(last_left);
//             cv::Mat right_m = rgb_to_mat(last_right);

//             whiteBalanceGrayWorldInplace(left_m);
//             whiteBalanceGrayWorldInplace(right_m);

//             cv::cvtColor(left_m, left_m, cv::COLOR_RGB2BGR);
//             cv::cvtColor(right_m, right_m, cv::COLOR_RGB2BGR);

//             cv::resize(left_m, left_m, cv::Size(320, 160));
//             cv::resize(right_m, right_m, cv::Size(320, 160));

//             cv::flip(left_m, left_m, 1);
//             cv::flip(right_m, right_m, 1);

//             auto left_msg  = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", left_m).toImageMsg();
//             auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", right_m).toImageMsg();

//             // ✅ 使用设备时间
//             left_msg->header.stamp  = deviceTimeToRosTime(last_left.system_time);
//             right_msg->header.stamp = deviceTimeToRosTime(last_right.system_time);

//             pub_left_rgb->publish(*left_msg);
//             pub_right_rgb->publish(*right_msg);

//             has_left = false;
//             has_right = false;
//         }
//         else
//         {
//             std::this_thread::sleep_for(std::chrono::microseconds(50));
//         }
//     }
// }

void rgb_thread(DecodePacket* packet, rclcpp::Node::SharedPtr node, bool do_remap = false)
{
    auto pub_left_rgb  = node->create_publisher<sensor_msgs::msg::Image>("/camera/left/rgb", 10);
    auto pub_right_rgb = node->create_publisher<sensor_msgs::msg::Image>("/camera/right/rgb", 10);

    // 1️⃣ 静态保存 remap 矩阵，只加载一次
    static cv::Mat map1_left, map2_left, map1_right, map2_right;
    static bool maps_loaded = false;
    if (do_remap && !maps_loaded) {
        cv::FileStorage fscv("/home/mingtao/calibration_ws/warping.xml", cv::FileStorage::READ);
        fscv["map1_left"] >> map1_left;
        fscv["map2_left"] >> map2_left;
        fscv["map1_right"] >> map1_right;
        fscv["map2_right"] >> map2_right;
        fscv.release();
        maps_loaded = true;
    }

    RGBData last_left, last_right;
    bool has_left = false, has_right = false;

    while (running)
    {
        RGBData rgb_left_data, rgb_right_data;

        if (packet->getRGBRightData(rgb_right_data)) { last_right = rgb_right_data; has_right = true; }
        if (packet->getRGBLeftData(rgb_left_data))   { last_left  = rgb_left_data;  has_left  = true; }

        if (has_left && has_right && last_left.sequence == last_right.sequence)
        {
            cv::Mat left_m  = rgb_to_mat(last_left);
            cv::Mat right_m = rgb_to_mat(last_right);

            whiteBalanceGrayWorldInplace(left_m);
            whiteBalanceGrayWorldInplace(right_m);

            cv::cvtColor(left_m, left_m, cv::COLOR_RGB2BGR);
            cv::cvtColor(right_m, right_m, cv::COLOR_RGB2BGR);

            cv::resize(left_m, left_m, cv::Size(320, 160));
            cv::resize(right_m, right_m, cv::Size(320, 160));

            // 2️⃣ remap 可选
            if (do_remap) {
                cv::remap(left_m, left_m, map1_left, map2_left, cv::INTER_LINEAR);
                cv::remap(right_m, right_m, map1_right, map2_right, cv::INTER_LINEAR);
            }

            cv::flip(left_m, left_m, 1);
            cv::flip(right_m, right_m, 1);

            auto left_msg  = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", left_m).toImageMsg();
            auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", right_m).toImageMsg();

            left_msg->header.stamp  = deviceTimeToRosTime(last_left.system_time);
            right_msg->header.stamp = deviceTimeToRosTime(last_right.system_time);

            pub_left_rgb->publish(*left_msg);
            pub_right_rgb->publish(*right_msg);

            has_left = false;
            has_right = false;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }
}

// // ================= Diff线程 =================
// void diff_thread(DecodePacket* packet, rclcpp::Node::SharedPtr node, bool use_float)
// {
//     auto pub_left_diff  = node->create_publisher<sensor_msgs::msg::Image>("/camera/left/diff", 10);
//     auto pub_right_diff = node->create_publisher<sensor_msgs::msg::Image>("/camera/right/diff", 10);

//     cv::Mat td_left_mat, td_right_mat, sdl_left_mat, sdl_right_mat, sdr_left_mat, sdr_right_mat;
//     RODData td_left_data_last, td_right_data_last;

//     const double target_fps = 250.0;
//     const auto frame_interval = std::chrono::duration<double>(1.0 / target_fps);
//     auto last_pub_time = std::chrono::steady_clock::now();

//     // 预定义编码字符串，避免在循环中反复判断
//     std::string encoding = use_float ? "32FC3" : "8SC3";
//     int target_type = use_float ? CV_32FC3 : CV_8SC3;

//     while (running)
//     {
//         bool got_frame = false;
//         RODData td_right_data, td_left_data, sdl_right_data, sdl_left_data, sdr_right_data, sdr_left_data;

//         // 获取数据 (省略部分重复逻辑...)
//         if (packet->getTDRightData(td_right_data))   { td_right_mat = rod_to_mat(td_right_data); td_right_data_last = td_right_data; got_frame = true; }
//         if (packet->getTDLeftData(td_left_data))     { td_left_mat  = rod_to_mat(td_left_data);  td_left_data_last  = td_left_data;  got_frame = true; }
//         if (packet->getSDLRightData(sdl_right_data)) { sdl_right_mat = rod_to_mat(sdl_right_data); got_frame = true; }
//         if (packet->getSDLLeftData(sdl_left_data))   { sdl_left_mat  = rod_to_mat(sdl_left_data);  got_frame = true; }
//         if (packet->getSDRRightData(sdr_right_data)) { sdr_right_mat = rod_to_mat(sdr_right_data); got_frame = true; }
//         if (packet->getSDRLeftData(sdr_left_data))   { sdr_left_mat  = rod_to_mat(sdr_left_data);  got_frame = true; }

//         cv::Mat Diff_left, Diff_right;

//         // Merge 得到的是 CV_8SC3
//         if (!td_left_mat.empty() && !sdl_left_mat.empty() && !sdr_left_mat.empty())
//             cv::merge(std::vector<cv::Mat>{td_left_mat, sdl_left_mat, sdr_left_mat}, Diff_left);

//         if (!td_right_mat.empty() && !sdl_right_mat.empty() && !sdr_right_mat.empty())
//             cv::merge(std::vector<cv::Mat>{td_right_mat, sdl_right_mat, sdr_right_mat}, Diff_right);

//         if (got_frame)
//         {
//             auto now = std::chrono::steady_clock::now();
//             if (now - last_pub_time >= frame_interval)
//             {
//                 last_pub_time = now;

//                 auto publish_mat = [&](cv::Mat& img, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, RODData& last_data) {
//                     if (img.empty()) return;

//                     // --- 核心改动：根据参数决定转换逻辑 ---
//                     if (use_float) {
//                         // 如果需要 32FC3，则显式转换
//                         img.convertTo(img, CV_32FC3);
//                     } else {
//                         // 如果需要 8SC3，因为 merge 出来本来就是 8SC3，理论上无需处理
//                         // 但如果你想调用白平衡或其他 inplace 操作，可以传 target_type
//                         // whiteBalanceGrayWorldInplace(img, target_type); 
//                     }

//                     auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, img).toImageMsg();
//                     msg->header.stamp = deviceTimeToRosTime(last_data.system_time);
//                     pub->publish(*msg);
//                 };

//                 publish_mat(Diff_left, pub_left_diff, td_left_data_last);
//                 publish_mat(Diff_right, pub_right_diff, td_right_data_last);
//             }
//         }
//         else { std::this_thread::sleep_for(std::chrono::microseconds(50)); }
//     }
// }

void diff_thread(DecodePacket* packet, rclcpp::Node::SharedPtr node, bool use_float)
{
    auto pub_left_diff  = node->create_publisher<sensor_msgs::msg::Image>("/camera/left/diff", 10);
    auto pub_right_diff = node->create_publisher<sensor_msgs::msg::Image>("/camera/right/diff", 10);

    // 预定义编码
    std::string encoding = use_float ? "32FC3" : "8SC3";

    while (running)
    {
        cv::Mat td_l, sdl_l, sdr_l, td_r, sdl_r, sdr_r;
        RODData data_l, data_r;
        
        bool has_l = false;
        bool has_r = false;
        RODData tmp;
        if (packet->getTDLeftData(tmp))   { td_l  = rod_to_mat(tmp); data_l = tmp; has_l = true; }
        if (packet->getSDLLeftData(tmp))  { sdl_l = rod_to_mat(tmp); }
        if (packet->getSDRLeftData(tmp))  { sdr_l = rod_to_mat(tmp); }

        if (packet->getTDRightData(tmp))  { td_r  = rod_to_mat(tmp); data_r = tmp; has_r = true; }
        if (packet->getSDLRightData(tmp)) { sdl_r = rod_to_mat(tmp); }
        if (packet->getSDRRightData(tmp)) { sdr_r = rod_to_mat(tmp); }

        // 定义发布 Lambda
        auto publish_merged = [&](cv::Mat& b1, cv::Mat& b2, cv::Mat& b3, 
                                 rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, 
                                 RODData& stamp_data) 
        {
            if (b1.empty() || b2.empty() || b3.empty()) return;

            cv::Mat merged;
            cv::merge(std::vector<cv::Mat>{b1, b2, b3}, merged);

            if (use_float) merged.convertTo(merged, CV_32FC3);

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, merged).toImageMsg();
            msg->header.stamp = deviceTimeToRosTime(stamp_data.system_time);
            pub->publish(*msg);
        };

        // 关键点 3：只有当左侧或右侧“三通齐备”时才发布
        if (has_l) publish_merged(td_l, sdl_l, sdr_l, pub_left_diff, data_l);
        if (has_r) publish_merged(td_r, sdl_r, sdr_r, pub_right_diff, data_r);

        // 如果什么都没拿到，稍微休息，避免 CPU 100%
        if (!has_l && !has_r) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

// ================= IMU线程 =================
void imu_thread(DecodePacket* packet, rclcpp::Node::SharedPtr node)
{
    auto pub_imu = node->create_publisher<sensor_msgs::msg::Imu>("/imu", 200);
    const double G_TO_MS2 = 9.8015;
    const double DEG_TO_RAD = M_PI / 180.0;

    while (running)
    {
        IMUData imu_data;
        if (packet->getIMUData(imu_data))
        {
            // 使用 unique_ptr 是 ROS 2 推荐的高频发布方式
            auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

            imu_msg->header.stamp = deviceTimeToRosTime(imu_data.system_time);
            imu_msg->header.frame_id = "imu_link";

            imu_msg->linear_acceleration.x = imu_data.acc_b_x * G_TO_MS2;
            imu_msg->linear_acceleration.y = imu_data.acc_b_y * G_TO_MS2;
            imu_msg->linear_acceleration.z = imu_data.acc_b_z * G_TO_MS2;

            imu_msg->angular_velocity.x = imu_data.gyr_b_x * DEG_TO_RAD;
            imu_msg->angular_velocity.y = imu_data.gyr_b_y * DEG_TO_RAD;
            imu_msg->angular_velocity.z = imu_data.gyr_b_z * DEG_TO_RAD;

            pub_imu->publish(std::move(imu_msg));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
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

    DecodePacket packet(frame_buffer, dev, 1024 * 16);
    packet.start();
    bool use_float_output = false;

    bool use_remap = false; // true 表示对左右目做 remap
    std::thread t_rgb(rgb_thread, &packet, node, use_remap);
    std::thread t_diff(diff_thread, &packet, node, use_float_output);
    std::thread t_imu(imu_thread, &packet, node);

    rclcpp::spin(node);

    running = false;
    t_rgb.join();
    t_diff.join();
    t_imu.join();

    rclcpp::shutdown();
    return 0;
}
