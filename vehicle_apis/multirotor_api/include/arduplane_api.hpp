// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#ifndef MULTIROTOR_API_INCLUDE_ARDUPLANE_API_HPP_
#define MULTIROTOR_API_INCLUDE_ARDUPLANE_API_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "UdpSocket.hpp"
#include "core_sim/actor/robot.hpp"
#include "core_sim/runtime_components.hpp"
#include "core_sim/transforms/transform_tree.hpp"

namespace microsoft {
namespace projectairsim {

// ArduPlaneApi
// Bridges a JSBSim fixed-wing robot to ArduPilot SITL through ArduPilot's JSON
// physics backend (sim_vehicle.py -f JSON). ArduPilot sends one servo packet
// per physics frame and waits for the matching state reply (lockstep), so the
// controller answers exactly once per Project AirSim tick.
class ArduPlaneApi : public IController {
 public:
  // Servo outputs carried by ArduPilot's 32-channel packet.
  static constexpr int kMaxServoChannels = 32;

  // How an ArduPilot PWM output maps onto a normalized control signal.
  enum class OutputRange {
    kUnipolar,  // 1000..2000 us -> 0..1 (throttle)
    kBipolar    // 1000..2000 us -> -1..1 around 1500 us (control surfaces)
  };

  struct ServoPacket {
    uint16_t frame_rate = 0;
    uint32_t frame_count = 0;
    int channel_count = 0;
    std::array<uint16_t, kMaxServoChannels> pwm{};
  };

  // State reported to ArduPilot, in its JSON backend's units and frames.
  struct FrameState {
    double timestamp_sec = 0.0;
    Vector3 gyro_body = Vector3::Zero();        // rad/s
    Vector3 accel_body = Vector3::Zero();       // specific force, m/s^2
    Vector3 position_ned = Vector3::Zero();     // m from the scene origin
    Quaternion orientation = Quaternion::Identity();
    Vector3 velocity_ned = Vector3::Zero();     // m/s
    bool has_airspeed = false;
    float airspeed = 0.0f;                      // m/s
  };

  ArduPlaneApi() {}
  ArduPlaneApi(const Robot& robot, TransformTree* ptransformtree);

  ~ArduPlaneApi() override;

  //---------------------------------------------------------------------------
  // IController overrides

  void BeginUpdate() override;
  void EndUpdate() override;
  void Reset() override;
  void SetKinematics(const Kinematics* kinematics) override;
  void Update() override;
  int GetControlSignalIndex(const std::string& actuator_id) override;
  void GetControlSignalSnapshot(std::vector<float>& control_signals) override;
  std::vector<float> GetControlSignals(int signal_index) override;
  std::vector<float> GetControlSignals(const std::string& actuator_id) override;
  const GimbalState& GetGimbalSignal(const std::string& gimbal_id) override;

  //---------------------------------------------------------------------------
  // Protocol helpers, public for unit tests

  // Parses a 16- or 32-channel servo packet; false for any other datagram.
  static bool ParseServoPacket(const uint8_t* data, size_t size,
                               ServoPacket& packet);
  static float NormalizePwm(uint16_t pwm, OutputRange range);
  // A mount servo's angle: 1000..2000 us across its travel min..max (deg).
  static float PwmToAngleDeg(uint16_t pwm, float min_deg, float max_deg);
  // One newline-terminated JSON frame for ArduPilot's JSON backend.
  static std::string FormatFrameState(const FrameState& state);

 private:
  struct OutputMapping {
    std::string actuator_id;
    OutputRange range = OutputRange::kBipolar;
    float scale = 1.0f;
  };

  // A self-stabilising gimbal on ArduPilot's mount servo outputs (MNT1_TYPE
  // 7, BrushlessPWM): roll and pitch from the horizon, yaw from the nose, as
  // the gimbal actuator takes them. The travel must equal MNT1_*_MIN/MAX, and
  // the outputs' SERVOn_MIN/MAX 1000/2000.
  struct GimbalOutputs {
    std::string id;                  // the gimbal actuator; empty: no gimbal
    std::array<int, 3> channel{};    // roll, pitch, yaw: SERVOn, 0 = none
    std::array<float, 3> min_deg{};  // travel
    std::array<float, 3> max_deg{};
  };

  // Phases of the optional catapult launch for robots that start airborne.
  enum class LaunchPhase { kNone, kHeld, kLaunching, kFlying };

  void LoadSettings(const Robot& robot);
  void OpenSocket();
  void CloseSocket();
  bool ReceiveServoPacket(uint32_t timeout_ms, ServoPacket& packet);
  void ApplyServoPacket(const ServoPacket& packet);
  void ApplyGimbalOutputs(const ServoPacket& packet);
  float GetLaunchThrottle() const;
  void UpdateLaunch(double now_sec);
  FrameState BuildFrameState(double now_sec) const;
  void SendFrameState(const FrameState& state);

 private:
  Robot sim_robot_;
  std::unique_ptr<mavlinkcom::UdpSocket> udp_socket_;

  // Settings
  std::string local_host_ip_ = "0.0.0.0";
  int local_host_udp_port_ = 9002;
  uint32_t frame_timeout_ms_ = 1000;
  bool wait_for_ardupilot_ = false;
  bool send_airspeed_ = true;
  bool launch_hold_ = false;
  float launch_release_throttle_ = 0.5f;
  float launch_speed_ = 0.0f;           // m/s along body x
  float launch_duration_sec_ = 0.5f;

  std::vector<OutputMapping> output_mappings_;  // index = SERVOn - 1
  GimbalOutputs gimbal_outputs_;
  std::unordered_map<std::string, int> actuator_id_to_output_idx_map_;

  // Runtime state
  std::vector<float> control_outputs_;
  std::string ardupilot_ip_;
  uint16_t ardupilot_port_ = 0;
  bool connected_ = false;
  uint32_t last_frame_count_ = 0;
  LaunchPhase launch_phase_ = LaunchPhase::kNone;
  double launch_start_sec_ = 0.0;
  GimbalState gimbal_state_{};
};

}  // namespace projectairsim
}  // namespace microsoft

#endif  // MULTIROTOR_API_INCLUDE_ARDUPLANE_API_HPP_
