// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#include "arduplane_api.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

#include "core_sim/actuators/actuator.hpp"
#include "core_sim/clock.hpp"
#include "core_sim/physics_common_utils.hpp"
#include "json.hpp"

// JSBSim requires RTTI, so its headers must not be included from
// arduplane_api.hpp.
#include "FGFDMExec.h"
#include "models/FGPropagate.h"

namespace microsoft {
namespace projectairsim {

namespace {

constexpr const char* kControllerName = "ArduPlaneApi";

// Servo packet layout from ArduPilot libraries/SITL/SIM_JSON.h:
// uint16 magic, uint16 frame_rate, uint32 frame_count, uint16 pwm[16 or 32].
constexpr uint16_t kServoPacket16Magic = 18458;
constexpr uint16_t kServoPacket32Magic = 29569;
constexpr size_t kServoPacketHeaderSize = 8;

constexpr double kMetersToFeet = 3.28083989501;

uint16_t ReadU16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0] | (data[1] << 8));
}

uint32_t ReadU32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

void WriteVector(std::ostringstream& buf, const Vector3& v) {
  buf << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
}

}  // namespace

// -----------------------------------------------------------------------------
// class ArduPlaneApi

ArduPlaneApi::ArduPlaneApi(const Robot& robot, TransformTree* ptransformtree)
    : sim_robot_(robot) {
  static_cast<void>(ptransformtree);
  LoadSettings(robot);
}

ArduPlaneApi::~ArduPlaneApi() { CloseSocket(); }

void ArduPlaneApi::LoadSettings(const Robot& robot) {
  const json controller_settings_json =
      json::parse(robot.GetControllerSettings());
  const json settings_json =
      controller_settings_json.value("arduplane-settings", "{ }"_json);

  local_host_ip_ = settings_json.value("local-host-ip", local_host_ip_);
  local_host_udp_port_ =
      settings_json.value("local-host-udp-port", local_host_udp_port_);
  frame_timeout_ms_ = settings_json.value("frame-timeout-ms", frame_timeout_ms_);
  wait_for_ardupilot_ =
      settings_json.value("wait-for-ardupilot", wait_for_ardupilot_);
  send_airspeed_ = settings_json.value("send-airspeed", send_airspeed_);

  if (local_host_udp_port_ <= 0 || local_host_udp_port_ > 65535) {
    throw std::invalid_argument(
        "ArduPlaneApi: local-host-udp-port must be in 1..65535");
  }

  const json launch_json = settings_json.value("launch", "{ }"_json);
  launch_hold_ = launch_json.value("hold", launch_hold_);
  launch_release_throttle_ =
      launch_json.value("release-throttle", launch_release_throttle_);
  launch_speed_ = launch_json.value("speed", launch_speed_);
  launch_duration_sec_ = launch_json.value("duration", launch_duration_sec_);

  if (launch_hold_ && (robot.GetPhysicsType() != PhysicsType::kJSBSimPhysics ||
                       robot.GetJSBSimModel() == nullptr)) {
    throw std::invalid_argument(
        "ArduPlaneApi: launch.hold requires jsbsim-physics");
  }
  if (launch_duration_sec_ <= 0.0f) {
    throw std::invalid_argument("ArduPlaneApi: launch.duration must be > 0");
  }

  // Each actuator-order entry is one ArduPilot servo output: index 0 is
  // SERVO1. Rotors default to the unipolar throttle range, everything else to
  // the bipolar control-surface range.
  const json actuator_order_json =
      settings_json.value("actuator-order", "[ ]"_json);
  if (actuator_order_json.size() > kMaxServoChannels) {
    throw std::invalid_argument(
        "ArduPlaneApi: actuator-order supports at most 32 servo outputs");
  }

  for (const auto& actuator_json : actuator_order_json) {
    OutputMapping mapping;
    mapping.actuator_id = actuator_json.value("id", "");
    mapping.scale = actuator_json.value("scale", 1.0f);

    for (const auto& actuator_ref : robot.GetActuators()) {
      if (actuator_ref.get().GetId() == mapping.actuator_id &&
          actuator_ref.get().GetType() == ActuatorType::kRotor) {
        mapping.range = OutputRange::kUnipolar;
        break;
      }
    }

    const std::string range = actuator_json.value("range", "");
    if (range == "unipolar") {
      mapping.range = OutputRange::kUnipolar;
    } else if (range == "bipolar") {
      mapping.range = OutputRange::kBipolar;
    } else if (!range.empty()) {
      throw std::invalid_argument(
          "ArduPlaneApi: actuator-order range must be 'unipolar' or "
          "'bipolar'");
    }

    const int output_idx = static_cast<int>(output_mappings_.size());
    if (!mapping.actuator_id.empty()) {
      actuator_id_to_output_idx_map_.emplace(mapping.actuator_id, output_idx);
    }
    output_mappings_.push_back(mapping);
  }

  control_outputs_.assign(output_mappings_.size(), 0.0f);
}

//---------------------------------------------------------------------------
// IController overrides

void ArduPlaneApi::BeginUpdate() {
  Reset();
  OpenSocket();
}

void ArduPlaneApi::EndUpdate() { CloseSocket(); }

void ArduPlaneApi::Reset() {
  std::fill(control_outputs_.begin(), control_outputs_.end(), 0.0f);
  connected_ = false;
  last_frame_count_ = 0;
  launch_phase_ = launch_hold_ ? LaunchPhase::kHeld : LaunchPhase::kNone;
  launch_start_sec_ = 0.0;
}

void ArduPlaneApi::SetKinematics(const Kinematics* /*kinematics*/) {}

void ArduPlaneApi::Update() {
  if (udp_socket_ == nullptr) return;

  const double now_sec = SimClock::Get()->NowSimMicros() * 1.0e-6;

  // Freeze the airframe before waiting, so a held robot does not drop while
  // ArduPilot boots.
  UpdateLaunch(now_sec);

  // Once ArduPilot is connected it is in lockstep with this tick, so wait for
  // its frame. Before that, only block when asked to, so the scene keeps
  // running without an autopilot.
  const uint32_t timeout_ms =
      (connected_ || wait_for_ardupilot_) ? frame_timeout_ms_ : 0;

  ServoPacket packet;
  if (!ReceiveServoPacket(timeout_ms, packet)) {
    if (connected_) {
      sim_robot_.GetLogger().LogWarning(
          kControllerName,
          "No servo frame from ArduPilot within %u ms; holding last outputs "
          "until it reconnects.",
          frame_timeout_ms_);
      connected_ = false;
    }
    return;
  }

  // ArduPilot resends its last frame when a reply is late (e.g. while the
  // scene is paused). Answer only the newest one so every reply carries a new
  // timestamp.
  ServoPacket newer;
  while (ReceiveServoPacket(0, newer)) packet = newer;

  if (!connected_) {
    sim_robot_.GetLogger().LogTrace(
        kControllerName, "ArduPilot connected from %s:%u (%d channels, %u Hz)",
        ardupilot_ip_.c_str(), ardupilot_port_, packet.channel_count,
        packet.frame_rate);
    connected_ = true;
  } else if (packet.frame_count < last_frame_count_) {
    sim_robot_.GetLogger().LogTrace(kControllerName,
                                    "ArduPilot restarted (frame count reset)");
  }
  last_frame_count_ = packet.frame_count;

  ApplyServoPacket(packet);
  UpdateLaunch(now_sec);
  SendFrameState(BuildFrameState(now_sec));
}

int ArduPlaneApi::GetControlSignalIndex(const std::string& actuator_id) {
  const auto it = actuator_id_to_output_idx_map_.find(actuator_id);
  if (it == actuator_id_to_output_idx_map_.end()) {
    sim_robot_.GetLogger().LogWarning(
        kControllerName,
        "Actuator '%s' is not in arduplane-settings.actuator-order",
        actuator_id.c_str());
    return -1;
  }
  return it->second;
}

void ArduPlaneApi::GetControlSignalSnapshot(
    std::vector<float>& control_signals) {
  control_signals.assign(control_outputs_.begin(), control_outputs_.end());
}

std::vector<float> ArduPlaneApi::GetControlSignals(int signal_index) {
  if (signal_index < 0 ||
      signal_index >= static_cast<int>(control_outputs_.size())) {
    return {0.f};
  }
  return {control_outputs_[signal_index]};
}

std::vector<float> ArduPlaneApi::GetControlSignals(
    const std::string& actuator_id) {
  return GetControlSignals(GetControlSignalIndex(actuator_id));
}

const IController::GimbalState& ArduPlaneApi::GetGimbalSignal(
    const std::string& /*gimbal_id*/) {
  return gimbal_state_;
}

//---------------------------------------------------------------------------
// Protocol helpers

bool ArduPlaneApi::ParseServoPacket(const uint8_t* data, size_t size,
                                    ServoPacket& packet) {
  if (data == nullptr || size < kServoPacketHeaderSize) return false;

  const uint16_t magic = ReadU16(data);
  int channel_count = 0;
  if (magic == kServoPacket16Magic) {
    channel_count = 16;
  } else if (magic == kServoPacket32Magic) {
    channel_count = 32;
  } else {
    return false;
  }
  if (size != kServoPacketHeaderSize + channel_count * sizeof(uint16_t)) {
    return false;
  }

  packet.frame_rate = ReadU16(data + 2);
  packet.frame_count = ReadU32(data + 4);
  packet.channel_count = channel_count;
  packet.pwm.fill(0);
  for (int i = 0; i < channel_count; ++i) {
    packet.pwm[i] = ReadU16(data + kServoPacketHeaderSize + 2 * i);
  }
  return true;
}

float ArduPlaneApi::NormalizePwm(uint16_t pwm, OutputRange range) {
  // A zero pulse is an output ArduPilot is not driving; treat it as neutral.
  if (pwm == 0) return 0.0f;
  if (range == OutputRange::kUnipolar) {
    return std::clamp((static_cast<float>(pwm) - 1000.0f) / 1000.0f, 0.0f,
                      1.0f);
  }
  return std::clamp((static_cast<float>(pwm) - 1500.0f) / 500.0f, -1.0f, 1.0f);
}

std::string ArduPlaneApi::FormatFrameState(const FrameState& state) {
  std::ostringstream buf;
  buf.imbue(std::locale::classic());
  buf << std::setprecision(9);

  buf << "{\"timestamp\":" << std::fixed << std::setprecision(6)
      << state.timestamp_sec << std::defaultfloat << std::setprecision(9);
  buf << ",\"imu\":{\"gyro\":";
  WriteVector(buf, state.gyro_body);
  buf << ",\"accel_body\":";
  WriteVector(buf, state.accel_body);
  buf << "},\"position\":";
  WriteVector(buf, state.position_ned);
  buf << ",\"quaternion\":[" << state.orientation.w() << ","
      << state.orientation.x() << "," << state.orientation.y() << ","
      << state.orientation.z() << "]";
  buf << ",\"velocity\":";
  WriteVector(buf, state.velocity_ned);
  if (state.has_airspeed) {
    buf << ",\"airspeed\":" << state.airspeed;
  }
  // ArduPilot splits frames on newlines.
  buf << "}\n";
  return buf.str();
}

//---------------------------------------------------------------------------
// Helpers

void ArduPlaneApi::OpenSocket() {
  CloseSocket();
  udp_socket_ = std::make_unique<mavlinkcom::UdpSocket>();
  if (udp_socket_->bind(local_host_ip_, local_host_udp_port_) != 0) {
    sim_robot_.GetLogger().LogError(
        kControllerName, "Could not bind UDP %s:%d for ArduPilot SITL",
        local_host_ip_.c_str(), local_host_udp_port_);
    udp_socket_.reset();
    return;
  }
  sim_robot_.GetLogger().LogTrace(
      kControllerName, "Waiting for ArduPilot JSON SITL on UDP %s:%d",
      local_host_ip_.c_str(), local_host_udp_port_);
}

void ArduPlaneApi::CloseSocket() {
  if (udp_socket_ != nullptr) {
    udp_socket_->close();
    udp_socket_.reset();
  }
  connected_ = false;
}

bool ArduPlaneApi::ReceiveServoPacket(uint32_t timeout_ms,
                                      ServoPacket& packet) {
  // Larger than the 72-byte 32-channel packet so oversized datagrams are
  // rejected instead of truncated into a valid-looking one.
  uint8_t buf[128];
  while (true) {
    const int rc = udp_socket_->recv(buf, sizeof(buf), timeout_ms);
    if (rc <= 0) return false;
    if (ParseServoPacket(buf, static_cast<size_t>(rc), packet)) {
      udp_socket_->last_recv_address(ardupilot_ip_, ardupilot_port_);
      return true;
    }
    // Skip stray datagrams without waiting again.
    timeout_ms = 0;
  }
}

void ArduPlaneApi::ApplyServoPacket(const ServoPacket& packet) {
  for (size_t i = 0; i < output_mappings_.size(); ++i) {
    if (static_cast<int>(i) >= packet.channel_count) {
      control_outputs_[i] = 0.0f;
      continue;
    }
    const auto& mapping = output_mappings_[i];
    control_outputs_[i] =
        NormalizePwm(packet.pwm[i], mapping.range) * mapping.scale;
  }
}

float ArduPlaneApi::GetLaunchThrottle() const {
  float throttle = 0.0f;
  for (size_t i = 0; i < output_mappings_.size(); ++i) {
    if (output_mappings_[i].range == OutputRange::kUnipolar) {
      throttle = std::max(throttle, std::abs(control_outputs_[i]));
    }
  }
  return throttle;
}

void ArduPlaneApi::UpdateLaunch(double now_sec) {
  if (launch_phase_ == LaunchPhase::kNone ||
      launch_phase_ == LaunchPhase::kFlying) {
    return;
  }

  auto model = sim_robot_.GetJSBSimModel();
  if (model == nullptr) return;

  if (launch_phase_ == LaunchPhase::kHeld) {
    // Hold stops JSBSim from integrating, so the airframe stays at its spawn
    // pose while ArduPilot boots, aligns its EKF and arms.
    model->Hold();
    if (!connected_ || GetLaunchThrottle() < launch_release_throttle_) return;

    model->Resume();
    launch_start_sec_ = now_sec;
    launch_phase_ =
        launch_speed_ > 0.0f ? LaunchPhase::kLaunching : LaunchPhase::kFlying;
    sim_robot_.GetLogger().LogTrace(
        kControllerName, "Launch released at throttle %.2f (%.1f m/s)",
        GetLaunchThrottle(), launch_speed_);
  }

  if (launch_phase_ == LaunchPhase::kLaunching) {
    // Catapult: ramp body-forward speed up to launch.speed over
    // launch.duration. JSBSim's velocities/u-fps property is read-only, so set
    // the propagated state directly.
    const double elapsed_sec = now_sec - launch_start_sec_;
    const double fraction = std::min(1.0, elapsed_sec / launch_duration_sec_);
    const double target_fps = launch_speed_ * fraction * kMetersToFeet;
    auto* propagate = model->GetPropagate();
    if (propagate != nullptr && propagate->GetUVW(1) < target_fps) {
      propagate->SetUVW(1, target_fps);
    }
    if (fraction >= 1.0) launch_phase_ = LaunchPhase::kFlying;
  }
}

ArduPlaneApi::FrameState ArduPlaneApi::BuildFrameState(double now_sec) const {
  const Kinematics& kinematics = sim_robot_.GetKinematics();
  const Environment& environment = sim_robot_.GetEnvironment();
  const Vector3& gravity = environment.env_info.gravity;
  const Quaternion& orientation = kinematics.pose.orientation;

  FrameState state;
  state.timestamp_sec = now_sec;
  state.position_ned = kinematics.pose.position;
  state.orientation = orientation;

  if (launch_phase_ == LaunchPhase::kHeld) {
    // Report a vehicle resting on its launcher: no motion, and the
    // accelerometer reads the reaction to gravity.
    state.accel_body =
        PhysicsUtils::TransformVectorToBodyFrame(-gravity, orientation);
  } else {
    state.velocity_ned = kinematics.twist.linear;
    state.gyro_body = kinematics.twist.angular;
    // kinematics.accels.linear is the inertial acceleration in NED including
    // gravity; an accelerometer measures it without gravity, in body axes.
    state.accel_body = PhysicsUtils::TransformVectorToBodyFrame(
        kinematics.accels.linear - gravity, orientation);
    if (launch_phase_ == LaunchPhase::kLaunching) {
      state.accel_body.x() += launch_speed_ / launch_duration_sec_;
    }
  }

  if (send_airspeed_) {
    const Vector3 air_velocity_body = PhysicsUtils::TransformVectorToBodyFrame(
        state.velocity_ned - environment.wind_velocity, orientation);
    state.has_airspeed = true;
    state.airspeed = std::max(0.0f, air_velocity_body.x());
  }
  return state;
}

void ArduPlaneApi::SendFrameState(const FrameState& state) {
  // ArduPilot cannot parse NaN or inf; drop the frame rather than send JSON it
  // would reject, and report the diverged state.
  const bool finite = state.gyro_body.allFinite() &&
                      state.accel_body.allFinite() &&
                      state.position_ned.allFinite() &&
                      state.velocity_ned.allFinite() &&
                      state.orientation.coeffs().allFinite() &&
                      std::isfinite(state.airspeed);
  if (!finite) {
    sim_robot_.GetLogger().LogError(
        kControllerName, "Vehicle state is not finite; not sent to ArduPilot");
    return;
  }

  const std::string frame = FormatFrameState(state);
  udp_socket_->sendto(frame.c_str(), frame.length(), ardupilot_ip_,
                      ardupilot_port_);
}

}  // namespace projectairsim
}  // namespace microsoft
