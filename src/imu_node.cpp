#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serial/serial.h"
#if __has_include("tf2/LinearMath/Quaternion.hpp")
#include "tf2/LinearMath/Quaternion.hpp"
#else
#include "tf2/LinearMath/Quaternion.h"
#endif

#include "imu_yzf143_ros2/frame_parser.hpp"

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("imu_yzf143_node")
  {
    port_ = declare_startup_parameter<std::string>("port", "/dev/ttyUSB0");
    const auto baud = declare_startup_parameter<int64_t>("baud", 115200);
    frame_id_ = declare_startup_parameter<std::string>("frame_id", "imu_link");
    if (port_.empty() || frame_id_.empty()) {
      throw std::invalid_argument("port and frame_id must be nonempty");
    }
    if (baud <= 0 || baud > std::numeric_limits<uint32_t>::max()) {
      throw std::invalid_argument("baud must be a positive 32-bit integer");
    }

    const auto poll_interval = declare_interval("poll_interval_ms", 5);
    const auto serial_timeout = declare_interval("serial_timeout_ms", 5, 0);
    reconnect_interval_ = declare_interval("reconnect_interval_ms", 1000);
    data_timeout_ = declare_interval("data_timeout_ms", 1000, 0);
    orientation_covariance_ = declare_covariance("orientation_variance");
    angular_velocity_covariance_ = declare_covariance("angular_velocity_variance");
    linear_acceleration_covariance_ = declare_covariance("linear_acceleration_variance");

    // Preserve reliable QoS for existing subscribers.
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    ser_.setPort(port_);
    ser_.setBaudrate(static_cast<uint32_t>(baud));
    ser_.setBytesize(serial::eightbits);
    ser_.setStopbits(serial::stopbits_one);
    ser_.setParity(serial::parity_none);
    auto timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(serial_timeout.count()));
    ser_.setTimeout(timeout);

    try_open_port();
    timer_ = create_wall_timer(poll_interval, std::bind(&ImuNode::timer_callback, this));
  }

  ~ImuNode() override
  {
    close_port();
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  template<typename T>
  T declare_startup_parameter(const std::string & name, const T & default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, default_value, descriptor);
  }

  std::chrono::milliseconds declare_interval(
    const std::string & name, int64_t default_value, int64_t minimum = 1)
  {
    const auto value = declare_startup_parameter<int64_t>(name, default_value);
    if (value < minimum || value > std::numeric_limits<uint32_t>::max()) {
      throw std::invalid_argument(name + " is outside its supported millisecond range");
    }
    return std::chrono::milliseconds(value);
  }

  std::array<double, 9> declare_covariance(const std::string & name)
  {
    const auto values = declare_startup_parameter<std::vector<double>>(
      name, std::vector<double>{0.0, 0.0, 0.0});
    if (values.size() != 3 || std::any_of(values.begin(), values.end(), [](double value) {
        return !std::isfinite(value) || value < 0.0;
      }))
    {
      throw std::invalid_argument(name + " must contain three finite, nonnegative variances");
    }
    std::array<double, 9> covariance{};
    for (size_t axis = 0; axis < 3; ++axis) {
      covariance[4 * axis] = values[axis];
    }
    return covariance;
  }

  void try_open_port()
  {
    next_reconnect_at_ = SteadyClock::now() + reconnect_interval_;
    try {
      ser_.open();
      parser_.reset();
      last_valid_frame_at_ = SteadyClock::now();
      RCLCPP_INFO(get_logger(), "Opened IMU serial port %s", port_.c_str());
    } catch (const serial::IOException & error) {
      // The serial library may retain an fd if configuration fails during open().
      // Retry only known device availability errors. Configuration failures exit
      // through main() so repeated attempts cannot accumulate leaked descriptors.
      const int error_number = error.getErrorNumber();
      if (error_number != ENOENT && error_number != EACCES &&
        error_number != ENXIO && error_number != ENODEV)
      {
        throw;
      }
      RCLCPP_WARN(get_logger(), "Cannot open %s, retrying: %s", port_.c_str(), error.what());
      close_port();
    }
  }

  void close_port()
  {
    parser_.reset();
    next_reconnect_at_ = SteadyClock::now() + reconnect_interval_;
    try {
      if (ser_.isOpen()) {
        ser_.close();
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Error closing IMU serial port: %s", error.what());
    }
  }

  void timer_callback()
  {
    if (!ser_.isOpen()) {
      if (SteadyClock::now() >= next_reconnect_at_) {
        try_open_port();
      }
      return;
    }

    try {
      // Reuse storage and bound work per tick, including when the port has a backlog.
      const size_t available = std::min(ser_.available(), read_buffer_.size());
      if (available > 0) {
        const size_t bytes_read = ser_.read(read_buffer_.data(), available);
        const auto previous_failures = parser_.checksum_failures();
        imu_yzf143_ros2::ImuSample sample;
        for (size_t index = 0; index < bytes_read; ++index) {
          if (parser_.push(read_buffer_[index], sample)) {
            last_valid_frame_at_ = SteadyClock::now();
            publish_imu_msg(sample);
          }
        }
        if (parser_.checksum_failures() != previous_failures) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "Discarded IMU frame with invalid checksum");
        }
      }

      if (data_timeout_.count() > 0 &&
        SteadyClock::now() - last_valid_frame_at_ >= data_timeout_)
      {
        RCLCPP_WARN(get_logger(), "No valid IMU frames before data timeout, reconnecting");
        close_port();
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "IMU serial read failed, reconnecting: %s", error.what());
      close_port();
    }
  }

  void publish_imu_msg(const imu_yzf143_ros2::ImuSample & sample)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;

    constexpr double to_rad = 0.01 * (3.14159265358979323846 / 180.0);
    constexpr double scale_acc = 9.80665 / 10920.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(sample.roll * to_rad, sample.pitch * to_rad, sample.yaw * to_rad);
    msg.orientation.x = quaternion.x();
    msg.orientation.y = quaternion.y();
    msg.orientation.z = quaternion.z();
    msg.orientation.w = quaternion.w();
    msg.orientation_covariance = orientation_covariance_;

    msg.linear_acceleration.x = sample.acceleration[0] * scale_acc;
    msg.linear_acceleration.y = sample.acceleration[1] * scale_acc;
    msg.linear_acceleration.z = sample.acceleration[2] * scale_acc;
    msg.linear_acceleration_covariance = linear_acceleration_covariance_;
    msg.angular_velocity.x = sample.angular_velocity[0] * to_rad;
    msg.angular_velocity.y = sample.angular_velocity[1] * to_rad;
    msg.angular_velocity.z = sample.angular_velocity[2] * to_rad;
    msg.angular_velocity_covariance = angular_velocity_covariance_;
    imu_pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  serial::Serial ser_;
  imu_yzf143_ros2::FrameParser parser_;
  std::array<uint8_t, 4096> read_buffer_{};
  std::string port_;
  std::string frame_id_;
  std::chrono::milliseconds reconnect_interval_{1000};
  std::chrono::milliseconds data_timeout_{1000};
  SteadyClock::time_point next_reconnect_at_{};
  SteadyClock::time_point last_valid_frame_at_{};
  std::array<double, 9> orientation_covariance_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<ImuNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_yzf143_node"), "IMU node failed: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}


