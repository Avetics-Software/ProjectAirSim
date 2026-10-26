// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#include <memory>
#include <string>

#include "actuators/actuator_impl.hpp"
#include "core_sim/actuators/gimbal.hpp"
#include "core_sim/config_json.hpp"
#include "core_sim/logger.hpp"
#include "core_sim/service_manager.hpp"
#include "core_sim/transforms/transform_utils.hpp"
#include "gtest/gtest.h"
#include "json.hpp"
#include "state_manager.hpp"
#include "topic_manager.hpp"

using json = nlohmann::json;

namespace microsoft {
namespace projectairsim {
namespace {

Logger MakeLogger() {
  auto logger_callback = [](const std::string&, LogLevel, const std::string&) {};
  return Logger(logger_callback);
}

std::unique_ptr<Actuator> LoadActuatorFromJson(const json& config_json) {
  auto logger = MakeLogger();
  return ActuatorImpl::LoadActuator(config_json, logger, "GimbalTestRobot",
                                    "robot_0", TopicManager(logger), "/tests",
                                    ServiceManager(logger), StateManager(logger));
}

void ExpectRotationMatches(const Matrix3x3& actual, const Matrix3x3& expected) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(actual(row, col), expected(row, col), 1.0e-6f);
    }
  }
}

}  // namespace
}  // namespace projectairsim
}  // namespace microsoft

namespace projectairsim = microsoft::projectairsim;

TEST(Gimbal, LoadAppliesOriginRotationToState) {
  // General description:
  // Verifies that gimbal load applies origin rotation to internal state.
  // Arrange: prepare context for `const json config_json = R"({`.
  const json config_json = R"({
    "name": "gimbal_0",
    "type": "gimbal",
    "enabled": true,
    "parent-link": "body",
    "child-link": "camera_mount",
    "origin": {
      "xyz": "0 0 0",
      "rpy-deg": "10 -20 30"
    }
  })"_json;

  // Act: run `auto actuator = projectairsim::LoadActuatorFromJson(config_json);`.
  auto actuator = projectairsim::LoadActuatorFromJson(config_json);
  ASSERT_NE(actuator, nullptr);
  ASSERT_EQ(actuator->GetType(), projectairsim::ActuatorType::kGimbal);

  auto* gimbal = dynamic_cast<projectairsim::Gimbal*>(actuator.get());
  ASSERT_NE(gimbal, nullptr);
  EXPECT_TRUE(gimbal->IsLoaded());

  // Assert: check result from `const auto& state = gimbal->GetGimbalState();`.
  const auto& state = gimbal->GetGimbalState();
  EXPECT_NEAR(state.roll, projectairsim::TransformUtils::ToRadians(10.0f), 1.0e-6f);
  EXPECT_NEAR(state.pitch, projectairsim::TransformUtils::ToRadians(-20.0f), 1.0e-6f);
  EXPECT_NEAR(state.yaw, projectairsim::TransformUtils::ToRadians(30.0f), 1.0e-6f);
}

TEST(Gimbal, UpdateGimbalUsesRateLimitedSetpointsAndUpdatesTransform) {
  // General description:
  // Verifies setpoint update with velocity limits and final transform update.
  // Arrange: prepare context for `const json config_json = R"({`.
  const json config_json = R"({
    "name": "gimbal_1",
    "type": "gimbal",
    "enabled": true,
    "parent-link": "body",
    "child-link": "camera_mount",
    "origin": {
      "xyz": "0 0 0",
      "rpy-deg": "0 0 0"
    }
  })"_json;

  auto actuator = projectairsim::LoadActuatorFromJson(config_json);
  auto* gimbal = dynamic_cast<projectairsim::Gimbal*>(actuator.get());
  ASSERT_NE(gimbal, nullptr);

  projectairsim::IController::GimbalState command;
  command.roll = 1.0f;
  command.pitch = -1.0f;
  command.yaw = 0.25f;
  command.roll_vel = 0.5f;
  command.pitch_vel = -0.25f;
  command.yaw_vel = NAN;
  command.roll_lock = true;
  command.pitch_lock = false;
  command.yaw_lock = true;

  // Act: run `gimbal->UpdateGimbal(command, 1'000'000'000LL);`.
  gimbal->UpdateGimbal(command, 1'000'000'000LL);

  // Assert: check result from `const auto& state = gimbal->GetGimbalState();`.
  const auto& state = gimbal->GetGimbalState();
  EXPECT_FLOAT_EQ(state.roll, 0.5f);
  EXPECT_FLOAT_EQ(state.pitch, -0.25f);
  EXPECT_FLOAT_EQ(state.yaw, 0.25f);
  EXPECT_TRUE(state.roll_lock);
  EXPECT_FALSE(state.pitch_lock);
  EXPECT_TRUE(state.yaw_lock);

  // Assert: check result from `const auto& transforms = gimbal->GetActuatedTransforms();`.
  const auto& transforms = gimbal->GetActuatedTransforms();
  const auto transform_it = transforms.find("camera_mount");
  ASSERT_NE(transform_it, transforms.end());

  const auto expected_quaternion =
      projectairsim::TransformUtils::ToQuaternion(0.5f, -0.25f, 0.25f);
  projectairsim::ExpectRotationMatches(transform_it->second.affine3.linear(),
                                       expected_quaternion.toRotationMatrix());
}

namespace {

constexpr long long kStep = 10'000'000LL;  // 10 ms
const float kDeg = static_cast<float>(M_PI) / 180.0f;

std::unique_ptr<projectairsim::Actuator> LoadGimbal(const char* settings) {
  json config_json = R"({
    "name": "gimbal_1", "type": "gimbal", "enabled": true,
    "parent-link": "body", "child-link": "camera_mount",
    "origin": {"xyz": "0 0 0", "rpy-deg": "0 0 0"}
  })"_json;
  if (settings) config_json["gimbal-settings"] = json::parse(settings);
  return projectairsim::LoadActuatorFromJson(config_json);
}

projectairsim::IController::GimbalState Angles(float pitch_deg, float yaw_deg,
                                               bool yaw_lock = false) {
  projectairsim::IController::GimbalState s;
  s.roll = 0.0f;
  s.pitch = pitch_deg * kDeg;
  s.yaw = yaw_deg * kDeg;
  s.roll_lock = s.pitch_lock = true;
  s.yaw_lock = yaw_lock;
  return s;
}

}  // namespace

TEST(Gimbal, YawLockHoldsCompassHeadingWhileRobotTurns) {
  auto actuator = LoadGimbal(nullptr);
  auto* gimbal = dynamic_cast<projectairsim::Gimbal*>(actuator.get());
  ASSERT_NE(gimbal, nullptr);
  // Yaw lock: yaw 30 deg is a compass heading. The robot turns from 0 to 90.
  auto cmd = Angles(-45.0f, 30.0f, true);
  for (int deg = 0; deg <= 90; deg += 5) {
    gimbal->UpdateGimbal(cmd, kStep, deg * kDeg);
    // Camera heading = robot heading + yaw from the nose stays 30
    EXPECT_NEAR(deg * kDeg + gimbal->GetGimbalState().yaw, 30.0f * kDeg, 1e-4f);
  }
}

TEST(Gimbal, FollowModeIsInstantByDefaultAndTrailsWithFollowTime) {
  auto ideal = LoadGimbal(nullptr);
  auto smooth = LoadGimbal(R"({"yaw-follow-time-s": 1.0})");
  auto* g_ideal = dynamic_cast<projectairsim::Gimbal*>(ideal.get());
  auto* g_smooth = dynamic_cast<projectairsim::Gimbal*>(smooth.get());
  auto cmd = Angles(-45.0f, 0.0f);
  g_ideal->UpdateGimbal(cmd, kStep, 0.0f);
  g_smooth->UpdateGimbal(cmd, kStep, 0.0f);
  // The robot snaps to 90 deg: the ideal gimbal turns with it at once
  g_ideal->UpdateGimbal(cmd, kStep, 90.0f * kDeg);
  EXPECT_NEAR(g_ideal->GetGimbalState().yaw, 0.0f, 1e-4f);
  // The smooth one trails: right after, the camera still looks near the old
  // heading (-90 from the new nose); after 5 time constants it has caught up
  g_smooth->UpdateGimbal(cmd, kStep, 90.0f * kDeg);
  EXPECT_LT(g_smooth->GetGimbalState().yaw, -85.0f * kDeg);
  for (int i = 0; i < 500; ++i) g_smooth->UpdateGimbal(cmd, kStep, 90.0f * kDeg);
  EXPECT_NEAR(g_smooth->GetGimbalState().yaw, 0.0f, 1.0f * kDeg);
}

TEST(Gimbal, SlewRateAndTravelLimits) {
  auto actuator = LoadGimbal(
      R"({"max-pitch-rate-deg": 60, "pitch-min-deg": -90, "pitch-max-deg": 20,
          "max-yaw-rate-deg": 60, "yaw-min-deg": -120, "yaw-max-deg": 120})");
  auto* gimbal = dynamic_cast<projectairsim::Gimbal*>(actuator.get());
  // Pitch from 0 to -60 at 60 deg/s takes 1 s: half way after 0.5 s
  auto cmd = Angles(-60.0f, 0.0f);
  for (int i = 0; i < 50; ++i) gimbal->UpdateGimbal(cmd, kStep, 0.0f);
  EXPECT_NEAR(gimbal->GetGimbalState().pitch, -30.0f * kDeg, 0.5f * kDeg);
  for (int i = 0; i < 60; ++i) gimbal->UpdateGimbal(cmd, kStep, 0.0f);
  EXPECT_NEAR(gimbal->GetGimbalState().pitch, -60.0f * kDeg, 1e-4f);
  // Beyond the travel: stops at the limit
  auto too_far = Angles(-170.0f, 170.0f);
  for (int i = 0; i < 500; ++i) gimbal->UpdateGimbal(too_far, kStep, 0.0f);
  EXPECT_NEAR(gimbal->GetGimbalState().pitch, -90.0f * kDeg, 1e-4f);
  EXPECT_NEAR(gimbal->GetGimbalState().yaw, 120.0f * kDeg, 1e-4f);
}

TEST(Gimbal, YawLockStopsAtTheYawTravelLimit) {
  auto actuator = LoadGimbal(R"({"yaw-min-deg": -100, "yaw-max-deg": 100})");
  auto* gimbal = dynamic_cast<projectairsim::Gimbal*>(actuator.get());
  // Hold north while the robot turns to 150: the mount can only reach -100
  auto cmd = Angles(-45.0f, 0.0f, true);
  for (int deg = 0; deg <= 150; deg += 5) gimbal->UpdateGimbal(cmd, kStep, deg * kDeg);
  EXPECT_NEAR(gimbal->GetGimbalState().yaw, -100.0f * kDeg, 1e-4f);
}

TEST(Gimbal, AngleWithZeroRateGoesToTheAngle) {
  auto actuator = LoadGimbal(nullptr);
  auto* gimbal = dynamic_cast<projectairsim::Gimbal*>(actuator.get());
  // PX4 mission gimbal steps carry the angle with zero rates: up and down,
  // left and right, each reaches its angle
  for (auto [pitch, yaw] : {std::pair{-45.0f, 30.0f}, std::pair{-20.0f, -60.0f},
                            std::pair{-70.0f, 10.0f}}) {
    auto cmd = Angles(pitch, yaw);
    cmd.pitch_vel = cmd.yaw_vel = 0.0f;
    gimbal->UpdateGimbal(cmd, kStep, 0.0f);
    EXPECT_NEAR(gimbal->GetGimbalState().pitch, pitch * kDeg, 1e-4f);
    EXPECT_NEAR(gimbal->GetGimbalState().yaw, yaw * kDeg, 1e-4f);
  }
}

TEST(Gimbal, YawTurnsBackWithinItsTravelNotTheShortWayRound) {
  auto actuator = LoadGimbal(
      R"({"max-yaw-rate-deg": 120, "yaw-min-deg": -160, "yaw-max-deg": 160})");
  auto* gimbal = dynamic_cast<projectairsim::Gimbal*>(actuator.get());
  // At -160 from the nose, asked for +45: the short way (through the back)
  // is outside the travel, so the mount turns back through the front
  auto left = Angles(-45.0f, -160.0f);
  for (int i = 0; i < 300; ++i) gimbal->UpdateGimbal(left, kStep, 0.0f);
  auto right = Angles(-45.0f, 45.0f);
  for (int i = 0; i < 300; ++i) gimbal->UpdateGimbal(right, kStep, 0.0f);
  EXPECT_NEAR(gimbal->GetGimbalState().yaw, 45.0f * kDeg, 1e-4f);
}
