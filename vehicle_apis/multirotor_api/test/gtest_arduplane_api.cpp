// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#include <cstdint>
#include <string>
#include <vector>

#include "arduplane_api.hpp"
#include "gtest/gtest.h"
#include "json.hpp"

namespace projectairsim = microsoft::projectairsim;
using ArduPlaneApi = projectairsim::ArduPlaneApi;
using json = nlohmann::json;

namespace {

// Builds an ArduPilot servo packet the way SIM_JSON.cpp sends it.
std::vector<uint8_t> MakeServoPacket(uint16_t magic, uint16_t frame_rate,
                                     uint32_t frame_count,
                                     const std::vector<uint16_t>& pwm) {
  std::vector<uint8_t> data;
  auto put16 = [&data](uint16_t v) {
    data.push_back(static_cast<uint8_t>(v & 0xff));
    data.push_back(static_cast<uint8_t>(v >> 8));
  };
  put16(magic);
  put16(frame_rate);
  for (int shift = 0; shift < 32; shift += 8) {
    data.push_back(static_cast<uint8_t>((frame_count >> shift) & 0xff));
  }
  for (uint16_t v : pwm) put16(v);
  return data;
}

}  // namespace

TEST(ArduPlaneApi, Parses16ChannelServoPacket) {
  std::vector<uint16_t> pwm(16, 1500);
  pwm[0] = 1100;
  pwm[2] = 1900;
  pwm[15] = 2000;
  const auto data = MakeServoPacket(18458, 400, 0x01020304, pwm);
  ASSERT_EQ(data.size(), 40u);

  ArduPlaneApi::ServoPacket packet;
  ASSERT_TRUE(ArduPlaneApi::ParseServoPacket(data.data(), data.size(), packet));
  EXPECT_EQ(packet.channel_count, 16);
  EXPECT_EQ(packet.frame_rate, 400);
  EXPECT_EQ(packet.frame_count, 0x01020304u);
  EXPECT_EQ(packet.pwm[0], 1100);
  EXPECT_EQ(packet.pwm[1], 1500);
  EXPECT_EQ(packet.pwm[2], 1900);
  EXPECT_EQ(packet.pwm[15], 2000);
  EXPECT_EQ(packet.pwm[16], 0);
}

TEST(ArduPlaneApi, Parses32ChannelServoPacket) {
  std::vector<uint16_t> pwm(32, 1000);
  pwm[31] = 1234;
  const auto data = MakeServoPacket(29569, 1200, 7, pwm);
  ASSERT_EQ(data.size(), 72u);

  ArduPlaneApi::ServoPacket packet;
  ASSERT_TRUE(ArduPlaneApi::ParseServoPacket(data.data(), data.size(), packet));
  EXPECT_EQ(packet.channel_count, 32);
  EXPECT_EQ(packet.pwm[31], 1234);
}

TEST(ArduPlaneApi, RejectsMalformedServoPackets) {
  ArduPlaneApi::ServoPacket packet;

  auto wrong_magic = MakeServoPacket(12345, 400, 1, std::vector<uint16_t>(16));
  EXPECT_FALSE(ArduPlaneApi::ParseServoPacket(wrong_magic.data(),
                                              wrong_magic.size(), packet));

  // A 16-channel magic with a 32-channel body, and a truncated packet.
  auto wrong_size = MakeServoPacket(18458, 400, 1, std::vector<uint16_t>(32));
  EXPECT_FALSE(ArduPlaneApi::ParseServoPacket(wrong_size.data(),
                                              wrong_size.size(), packet));
  auto truncated = MakeServoPacket(18458, 400, 1, std::vector<uint16_t>(15));
  EXPECT_FALSE(ArduPlaneApi::ParseServoPacket(truncated.data(),
                                              truncated.size(), packet));

  EXPECT_FALSE(ArduPlaneApi::ParseServoPacket(nullptr, 40, packet));
}

TEST(ArduPlaneApi, NormalizesPwm) {
  using Range = ArduPlaneApi::OutputRange;

  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(1000, Range::kUnipolar), 0.0f);
  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(1500, Range::kUnipolar), 0.5f);
  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(2000, Range::kUnipolar), 1.0f);
  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(900, Range::kUnipolar), 0.0f);
  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(2100, Range::kUnipolar), 1.0f);

  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(1000, Range::kBipolar), -1.0f);
  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(1500, Range::kBipolar), 0.0f);
  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(1750, Range::kBipolar), 0.5f);
  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(2000, Range::kBipolar), 1.0f);

  // Undriven outputs are neutral.
  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(0, Range::kUnipolar), 0.0f);
  EXPECT_FLOAT_EQ(ArduPlaneApi::NormalizePwm(0, Range::kBipolar), 0.0f);
}

TEST(ArduPlaneApi, FormatsJsonFrameForArduPilot) {
  ArduPlaneApi::FrameState state;
  state.timestamp_sec = 12.004;
  state.gyro_body = {0.1f, -0.2f, 0.3f};
  state.accel_body = {0.0f, 0.0f, -9.80665f};
  state.position_ned = {10.0f, -5.0f, -120.0f};
  state.orientation = projectairsim::Quaternion(0.5f, 0.5f, 0.5f, 0.5f);
  state.velocity_ned = {20.0f, 1.0f, -0.5f};
  state.has_airspeed = true;
  state.airspeed = 19.5f;

  const std::string frame = ArduPlaneApi::FormatFrameState(state);
  ASSERT_FALSE(frame.empty());
  // ArduPilot splits frames on newlines; there must be exactly one, at the end.
  EXPECT_EQ(frame.back(), '\n');
  EXPECT_EQ(frame.find('\n'), frame.size() - 1);

  const json parsed = json::parse(frame);
  EXPECT_DOUBLE_EQ(parsed["timestamp"].get<double>(), 12.004);
  EXPECT_NEAR(parsed["imu"]["gyro"][1].get<double>(), -0.2, 1e-6);
  EXPECT_NEAR(parsed["imu"]["accel_body"][2].get<double>(), -9.80665, 1e-5);
  EXPECT_NEAR(parsed["position"][2].get<double>(), -120.0, 1e-6);
  ASSERT_EQ(parsed["quaternion"].size(), 4u);
  EXPECT_NEAR(parsed["quaternion"][0].get<double>(), 0.5, 1e-6);  // w first
  EXPECT_NEAR(parsed["velocity"][0].get<double>(), 20.0, 1e-6);
  EXPECT_NEAR(parsed["airspeed"].get<double>(), 19.5, 1e-6);
}

TEST(ArduPlaneApi, OmitsAirspeedWhenDisabled) {
  ArduPlaneApi::FrameState state;
  const json parsed = json::parse(ArduPlaneApi::FormatFrameState(state));
  EXPECT_FALSE(parsed.contains("airspeed"));
  EXPECT_TRUE(parsed.contains("timestamp"));
  EXPECT_TRUE(parsed.contains("velocity"));
}
