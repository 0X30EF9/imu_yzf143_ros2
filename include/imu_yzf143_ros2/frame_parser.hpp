#ifndef IMU_YZF143_ROS2__FRAME_PARSER_HPP_
#define IMU_YZF143_ROS2__FRAME_PARSER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>

namespace imu_yzf143_ros2
{

struct ImuSample
{
  std::array<int16_t, 3> acceleration{};
  std::array<int16_t, 3> angular_velocity{};
  int16_t yaw = 0;
  int16_t pitch = 0;
  int16_t roll = 0;
  uint16_t device_timestamp = 0;
};

class FrameParser
{
public:
  bool push(uint8_t byte, ImuSample & sample)
  {
    buffer_.push_back(byte);
    if (buffer_.size() < 24) {
      return false;
    }

    if (buffer_[0] != 0xA5 || buffer_[1] != 0xA5) {
      buffer_.pop_front();
      return false;
    }

    // Preserve the existing 16-bit additive checksum, including wraparound.
    const int32_t sum = static_cast<int32_t>(read_signed(12)) +
      read_signed(14) + read_signed(16) + read_signed(18);
    if (read_unsigned(22) != static_cast<uint16_t>(sum)) {
      ++checksum_failures_;
      buffer_.pop_front();
      return false;
    }

    for (size_t axis = 0; axis < 3; ++axis) {
      sample.acceleration[axis] = read_signed(2 + 2 * axis);
      sample.angular_velocity[axis] = read_signed(8 + 2 * axis);
    }
    sample.yaw = read_signed(14);
    sample.pitch = read_signed(16);
    sample.roll = read_signed(18);
    sample.device_timestamp = read_unsigned(20);
    buffer_.clear();
    return true;
  }

  void reset()
  {
    buffer_.clear();
  }

  uint64_t checksum_failures() const
  {
    return checksum_failures_;
  }

private:
  uint16_t read_unsigned(size_t offset) const
  {
    return static_cast<uint16_t>(buffer_[offset]) |
           static_cast<uint16_t>(static_cast<uint16_t>(buffer_[offset + 1]) << 8);
  }

  int16_t read_signed(size_t offset) const
  {
    const int32_t value = read_unsigned(offset);
    return static_cast<int16_t>(value < 0x8000 ? value : value - 0x10000);
  }

  // Sliding by one byte preserves overlapping headers and bounds storage to 24 bytes.
  std::deque<uint8_t> buffer_;
  uint64_t checksum_failures_ = 0;
};

}  // namespace imu_yzf143_ros2

#endif  // IMU_YZF143_ROS2__FRAME_PARSER_HPP_
