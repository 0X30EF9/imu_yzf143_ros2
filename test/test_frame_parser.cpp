#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "imu_yzf143_ros2/frame_parser.hpp"

namespace
{

using imu_yzf143_ros2::FrameParser;
using imu_yzf143_ros2::ImuSample;
using Frame = std::array<uint8_t, 24>;

// Acceleration [10920, -10920, 0], gyro [100, -200, 300],
// yaw/pitch/roll [9000, -4500, 1000], timestamp 0x1234, checksum 5800.
const Frame frame = {{
  0xA5, 0xA5, 0xA8, 0x2A, 0x58, 0xD5, 0x00, 0x00,
  0x64, 0x00, 0x38, 0xFF, 0x2C, 0x01, 0x28, 0x23,
  0x6C, 0xEE, 0xE8, 0x03, 0x34, 0x12, 0xA8, 0x16}};

std::vector<ImuSample> feed(FrameParser & parser, const std::vector<uint8_t> & bytes)
{
  std::vector<ImuSample> samples;
  ImuSample sample;
  for (const auto byte : bytes) {
    if (parser.push(byte, sample)) {
      samples.push_back(sample);
    }
  }
  return samples;
}

TEST(FrameParser, DecodesKnownLittleEndianFrame)
{
  FrameParser parser;
  const auto samples = feed(parser, {frame.begin(), frame.end()});
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_EQ(samples[0].acceleration, (std::array<int16_t, 3>{{10920, -10920, 0}}));
  EXPECT_EQ(samples[0].angular_velocity, (std::array<int16_t, 3>{{100, -200, 300}}));
  EXPECT_EQ(samples[0].yaw, 9000);
  EXPECT_EQ(samples[0].pitch, -4500);
  EXPECT_EQ(samples[0].roll, 1000);
  EXPECT_EQ(samples[0].device_timestamp, 0x1234);
  EXPECT_EQ(parser.checksum_failures(), 0U);
}

TEST(FrameParser, WaitsForCompleteFrame)
{
  FrameParser parser;
  ImuSample sample;
  for (size_t index = 0; index + 1 < frame.size(); ++index) {
    EXPECT_FALSE(parser.push(frame[index], sample));
  }
  ASSERT_TRUE(parser.push(frame.back(), sample));
  EXPECT_EQ(sample.yaw, 9000);
}

TEST(FrameParser, ResynchronizesAfterNoiseAndOverlappingHeader)
{
  FrameParser parser;
  std::vector<uint8_t> bytes(100001 + frame.size(), 0x37);
  bytes[100000] = 0xA5;
  std::copy(frame.begin(), frame.end(), bytes.begin() + 100001);
  const auto samples = feed(parser, bytes);
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_EQ(samples[0].device_timestamp, 0x1234);
}

TEST(FrameParser, RejectsBadChecksumThenAcceptsFollowingFrame)
{
  FrameParser parser;
  std::vector<uint8_t> bytes(frame.begin(), frame.end());
  bytes[22] ^= 0x01;
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  const auto samples = feed(parser, bytes);
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_EQ(samples[0].yaw, 9000);
  EXPECT_GT(parser.checksum_failures(), 0U);
}

TEST(FrameParser, AcceptsConsecutiveFramesWithoutDroppingBytes)
{
  FrameParser parser;
  std::vector<uint8_t> bytes;
  for (size_t count = 0; count < 3; ++count) {
    bytes.insert(bytes.end(), frame.begin(), frame.end());
  }
  EXPECT_EQ(feed(parser, bytes).size(), 3U);
}

TEST(FrameParser, HandlesPositiveChecksumOverflowAndSignedExtremes)
{
  FrameParser parser;
  Frame limits = frame;
  limits[2] = 0x00;
  limits[3] = 0x80;
  for (size_t offset = 12; offset <= 18; offset += 2) {
    limits[offset] = 0xFF;
    limits[offset + 1] = 0x7F;
  }
  // 4 * 32767 modulo 65536.
  limits[22] = 0xFC;
  limits[23] = 0xFF;
  const auto samples = feed(parser, {limits.begin(), limits.end()});
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_EQ(samples[0].acceleration[0], -32768);
  EXPECT_EQ(samples[0].angular_velocity[2], 32767);
  EXPECT_EQ(samples[0].yaw, 32767);
}

TEST(FrameParser, HandlesNegativeChecksumOverflow)
{
  FrameParser parser;
  Frame limits = frame;
  for (size_t offset = 12; offset <= 18; offset += 2) {
    limits[offset] = 0x00;
    limits[offset + 1] = 0x80;
  }
  // 4 * -32768 modulo 65536.
  limits[22] = 0x00;
  limits[23] = 0x00;
  const auto samples = feed(parser, {limits.begin(), limits.end()});
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_EQ(samples[0].angular_velocity[2], -32768);
  EXPECT_EQ(samples[0].roll, -32768);
}

TEST(FrameParser, ResetDiscardsIncompleteFrameFromPreviousConnection)
{
  FrameParser parser;
  EXPECT_TRUE(feed(parser, {frame.begin(), frame.begin() + 12}).empty());
  parser.reset();
  EXPECT_TRUE(feed(parser, {frame.begin() + 12, frame.end()}).empty());
  const auto samples = feed(parser, {frame.begin(), frame.end()});
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_EQ(samples[0].acceleration[0], 10920);
}

}  // namespace
