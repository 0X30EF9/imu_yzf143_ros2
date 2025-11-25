/*
 * ROS 2 Driver for ZYF176/ZYF143
 */
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "serial/serial.h"

using namespace std::chrono_literals;

// 协议结构体定义
#pragma pack(push, 1)
struct acc_type
{
    int16_t x;
    int16_t y;
    int16_t z;
};
struct gyro_type
{
    int16_t x;
    int16_t y;
    int16_t z;
};
struct angle_type
{
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
};

struct param_type
{
    acc_type acc;
    gyro_type gyro;
    angle_type angle;
    int16_t timeStamp;
    int16_t crc;
};

union analysis_type
{
    param_type param;
    uint8_t readBuffer[22];
};
#pragma pack(pop)

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("imu_yzf143_node")
    {
        // 参数声明
        std::string port = this->declare_parameter("port", "/dev/ttyUSB0");
        int baud_rate = this->declare_parameter("baud", 115200);
        frame_id_ = this->declare_parameter("frame_id", "imu_link");

        // 初始化发布者
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

        // 初始化串口
        try
        {
            ser_.setPort(port);
            ser_.setBaudrate(baud_rate);
            ser_.setBytesize(serial::eightbits);
            ser_.setStopbits(serial::stopbits_one);
            ser_.setParity(serial::parity_none);
            serial::Timeout to = serial::Timeout::simpleTimeout(10);
            ser_.setTimeout(to);
            ser_.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", port.c_str());
            return;
        }

        if (ser_.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "IMU Initialized on %s @ %d", port.c_str(), baud_rate);
        }

        // 定时器 200Hz (5ms)
        timer_ = this->create_wall_timer(
            5ms, std::bind(&ImuNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (!ser_.isOpen())
            return;

        // 1. 读取数据到缓存
        size_t available = ser_.available();
        if (available > 0)
        {
            std::vector<uint8_t> tmp_buf(available);
            ser_.read(tmp_buf.data(), available);
            buffer_.insert(buffer_.end(), tmp_buf.begin(), tmp_buf.end());
        }

        // 2. 解析缓存 (包长24)
        while (buffer_.size() >= 24)
        {
            // 检查头 A5 A5
            if (buffer_[0] == 0xA5 && buffer_[1] == 0xA5)
            {
                // 拷贝数据
                for (int i = 0; i < 22; i++)
                {
                    analysis_.readBuffer[i] = buffer_[i + 2];
                }

                // 校验
                if (check_crc())
                {
                    publish_imu_msg();
                    buffer_.erase(buffer_.begin(), buffer_.begin() + 24);
                }
                else
                {
                    // CRC 失败，滑动窗口继续找
                    buffer_.erase(buffer_.begin());
                }
            }
            else
            {
                // 头不对，滑动窗口
                buffer_.erase(buffer_.begin());
            }
        }

        // 保护：防止缓存溢出
        if (buffer_.size() > 2048)
        {
            buffer_.clear();
            RCLCPP_WARN(this->get_logger(), "Buffer overflow, cleared.");
        }
    }

    bool check_crc()
    {
        int16_t calc_sum = analysis_.param.gyro.z +
                           analysis_.param.angle.yaw +
                           analysis_.param.angle.pitch +
                           analysis_.param.angle.roll;
        return (analysis_.param.crc == calc_sum);
    }

    void publish_imu_msg()
    {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        // 单位转换系数
        const double to_rad = 0.01 * (3.1415926535 / 180.0);
        const double to_acc = 9.80665 / 10920.0;

        // 欧拉角 -> 四元数
        tf2::Quaternion q;
        q.setRPY(analysis_.param.angle.roll * to_rad,
                 analysis_.param.angle.pitch * to_rad,
                 analysis_.param.angle.yaw * to_rad);
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();

        // 角速度
        msg.angular_velocity.x = analysis_.param.gyro.x * to_rad;
        msg.angular_velocity.y = analysis_.param.gyro.y * to_rad;
        msg.angular_velocity.z = analysis_.param.gyro.z * to_rad;

        // 线加速度
        msg.linear_acceleration.x = analysis_.param.acc.x * to_acc;
        msg.linear_acceleration.y = analysis_.param.acc.y * to_acc;
        msg.linear_acceleration.z = analysis_.param.acc.z * to_acc;

        imu_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial ser_;
    std::string frame_id_;
    analysis_type analysis_;
    std::deque<uint8_t> buffer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}