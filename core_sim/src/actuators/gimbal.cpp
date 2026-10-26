// Copyright (C) Microsoft Corporation. 
// Copyright (C) 2025 IAMAI CONSULTING CORP

// MIT License. All rights reserved.

#include "core_sim/actuators/gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "actuator_impl.hpp"
#include "algorithms.hpp"
#include "constant.hpp"
#include "core_sim/actuators/actuator.hpp"
#include "core_sim/logger.hpp"
#include "core_sim/physics_common_types.hpp"
#include "json.hpp"

namespace microsoft {
namespace projectairsim {

using json = nlohmann::json;

//------------------------------------------------------------------------------
// Forward declarations

class Gimbal::Loader {
 public:
  explicit Loader(Gimbal::Impl& impl);

  void Load(const json& json);

 private:
  void LoadOriginSettings(const json& json);

  void LoadGimbalSettings(const json& json);

  Gimbal::Impl& impl_;
};

class Gimbal::Impl : public ActuatorImpl {
 public:
  Impl(const std::string& id, bool is_enabled, const std::string& parent_link,
       const std::string& child_link, const Logger& logger,
       const TopicManager& topic_manager, const std::string& parent_topic_path,
       const ServiceManager& service_manager,
       const StateManager& state_manager);

  void Load(ConfigJson config_json);

  void CreateTopics();

  const Transform& GetOrigin() const;

  void OnBeginUpdate() override;

  void OnEndUpdate() override;

  const ActuatedTransforms& GetActuatedTransforms() const;

  void UpdateActuatorOutput(const ControlSignals& control_signals,
                            const TimeNano sim_dt_nanos);

  void UpdateGimbal(IController::GimbalState& new_state,
                    const TimeNano sim_dt_nanos, float vehicle_yaw);

  const IController::GimbalState& GetGimbalState() const;

  double calcSetpoint(double dt, double lastSetpoint, double newSetpoint,
                      double newRateSetpoint);

  // How a physical gimbal moves (the actuator's "gimbal-settings"). The
  // defaults are an ideal gimbal: every commanded angle reached at once.
  struct Dynamics {
    float max_rate[3] = {INFINITY, INFINITY, INFINITY};      // rad/s, roll/pitch/yaw
    float min_angle[3] = {-INFINITY, -INFINITY, -INFINITY};  // rad (yaw: from the nose)
    float max_angle[3] = {INFINITY, INFINITY, INFINITY};
    float settling_time = 0.0f;    // s: first-order lag reaching a commanded angle
    float yaw_follow_time = 0.0f;  // s: in follow mode, lag behind the robot's turns
  };

 private:
  friend class Gimbal::Loader;
  ActuatedTransforms actuated_transforms_;

  Gimbal::Loader loader_;
  IController::GimbalState gimbal_state_;  // the actual angles

  std::vector<Topic> topics_;

  Dynamics dynamics_;
  // Commanded angles (rates integrated). Roll and pitch are level-referenced
  // (the camera mount doesn't inherit the robot's roll and pitch); yaw is from
  // the nose, or a compass heading while target_yaw_is_heading_ (yaw lock).
  float target_[3] = {NAN, NAN, NAN};
  bool target_yaw_is_heading_ = false;
  float heading_ = NAN;     // the camera's compass heading (rad)
  float follow_ref_ = NAN;  // the robot's heading as the follow mode sees it

  float Approach(float current, float target, int axis, float tau, float dt,
                 bool wrap) const;
};

//------------------------------------------------------------------------------
// class Gimbal

Gimbal::Gimbal(void) : Actuator(std::shared_ptr<ActuatorImpl>(nullptr)) {}

Gimbal::Gimbal(const std::string& id, bool is_enabled,
               const std::string& parent_link, const std::string& child_link,
               const Logger& logger, const TopicManager& topic_manager,
               const std::string& parent_topic_path,
               const ServiceManager& service_manager,
               const StateManager& state_manager)
    : Actuator(std::shared_ptr<ActuatorImpl>(new Gimbal::Impl(
          id, is_enabled, parent_link, child_link, logger, topic_manager,
          parent_topic_path, service_manager, state_manager))) {}

void Gimbal::Load(ConfigJson config_json) {
  static_cast<Gimbal::Impl*>(pimpl_.get())->Load(config_json);
}

void Gimbal::BeginUpdate() {
  static_cast<Gimbal::Impl*>(pimpl_.get())->BeginUpdate();
}

void Gimbal::EndUpdate() {
  static_cast<Gimbal::Impl*>(pimpl_.get())->EndUpdate();
}

const ActuatedTransforms& Gimbal::GetActuatedTransforms() const {
  return static_cast<Gimbal::Impl*>(pimpl_.get())->GetActuatedTransforms();
}

void Gimbal::UpdateGimbal(IController::GimbalState& new_state,
                          const TimeNano sim_dt_nanos, float vehicle_yaw) {
  return static_cast<Gimbal::Impl*>(pimpl_.get())
      ->UpdateGimbal(new_state, sim_dt_nanos, vehicle_yaw);
}

const IController::GimbalState& Gimbal::GetGimbalState() const {
  return static_cast<Gimbal::Impl*>(pimpl_.get())->GetGimbalState();
};

void Gimbal::UpdateActuatorOutput(const ControlSignals& control_signals,
                                  const TimeNano sim_dt_nanos) {
  static_cast<Gimbal::Impl*>(pimpl_.get())
      ->UpdateActuatorOutput(control_signals, sim_dt_nanos);
}

//------------------------------------------------------------------------------
// class Gimbal::Impl

Gimbal::Impl::Impl(const std::string& id, bool is_enabled,
                   const std::string& parent_link,
                   const std::string& child_link, const Logger& logger,
                   const TopicManager& topic_manager,
                   const std::string& parent_topic_path,
                   const ServiceManager& service_manager,
                   const StateManager& state_manager)
    : ActuatorImpl(ActuatorType::kGimbal, id, is_enabled, parent_link,
                   child_link, Constant::Component::gimbal, logger,
                   topic_manager, parent_topic_path, service_manager,
                   state_manager),
      loader_(*this),
      actuated_transforms_() {
  SetTopicPath();
  CreateTopics();
}

void Gimbal::Impl::Load(ConfigJson config_json) {
  json json = config_json;
  loader_.Load(json);
}

void Gimbal::Impl::CreateTopics() {}

void Gimbal::Impl::OnBeginUpdate() {}

void Gimbal::Impl::OnEndUpdate() {}
const IController::GimbalState& Gimbal::Impl::GetGimbalState() const {
  return gimbal_state_;
};

namespace {
float WrapPi(float a) {
  return std::remainder(a, 2.0f * static_cast<float>(M_PI));
}
}  // namespace

float Gimbal::Impl::Approach(float current, float target, int axis, float tau,
                             float dt, bool wrap) const {
  if (!std::isfinite(target)) return current;
  if (!std::isfinite(current)) return target;
  const float error = wrap ? WrapPi(target - current) : target - current;
  // First-order settling, then the slew-rate limit
  float step = tau > 0.0f ? error * (1.0f - std::exp(-dt / tau)) : error;
  const float max_step = dynamics_.max_rate[axis] * dt;
  step = std::max(-max_step, std::min(max_step, step));
  return wrap ? WrapPi(current + step) : current + step;
}

void Gimbal::Impl::UpdateGimbal(IController::GimbalState& new_state,
                                const TimeNano sim_dt_nanos,
                                float vehicle_yaw) {
  const float dt = SimClock::Get()->NanosToSec(sim_dt_nanos);
  const auto& d = dynamics_;
  const float actual[3] = {gimbal_state_.roll, gimbal_state_.pitch,
                           gimbal_state_.yaw};
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(target_[i])) target_[i] = actual[i];
  }

  // Yaw lock (yaw as a compass heading) needs the robot's heading; without
  // it the commanded yaw is applied from the nose, as before
  const bool heading_known = std::isfinite(vehicle_yaw);
  const bool yaw_lock = new_state.yaw_lock && heading_known;
  if (heading_known && yaw_lock != target_yaw_is_heading_) {
    // Switching frames: keep pointing the same way
    target_[2] = yaw_lock ? WrapPi(target_[2] + vehicle_yaw)
                          : WrapPi(target_[2] - vehicle_yaw);
    target_yaw_is_heading_ = yaw_lock;
  }

  // Commanded angles: an angle, a rate integrated onto the last command, or
  // both (the rate, stopping at the angle). Roll and pitch clamped to the
  // gimbal's travel here; yaw's travel is from the nose, applied below.
  const float sp[3] = {new_state.roll, new_state.pitch, new_state.yaw};
  const float vel[3] = {new_state.roll_vel, new_state.pitch_vel,
                        new_state.yaw_vel};
  for (int i = 0; i < 3; ++i) {
    target_[i] = calcSetpoint(dt, target_[i], sp[i], vel[i]);
    if (i < 2) {
      target_[i] =
          std::max(d.min_angle[i], std::min(d.max_angle[i], target_[i]));
    }
  }

  // The actual angles move toward the commanded ones at the gimbal's pace
  const float roll =
      Approach(gimbal_state_.roll, target_[0], 0, d.settling_time, dt, false);
  const float pitch =
      Approach(gimbal_state_.pitch, target_[1], 1, d.settling_time, dt, false);
  float yaw;
  if (heading_known) {
    if (!std::isfinite(heading_)) {
      heading_ = WrapPi(vehicle_yaw + gimbal_state_.yaw);
    }
    if (!std::isfinite(follow_ref_)) follow_ref_ = vehicle_yaw;
    // Follow mode trails the robot's turns by yaw_follow_time; the operator's
    // own yaw commands aren't slowed by it
    const float k = d.yaw_follow_time > 0.0f
                        ? 1.0f - std::exp(-dt / d.yaw_follow_time)
                        : 1.0f;
    follow_ref_ = WrapPi(follow_ref_ + WrapPi(vehicle_yaw - follow_ref_) * k);
    const float wanted = yaw_lock ? target_[2] : WrapPi(follow_ref_ + target_[2]);
    // Turn the way the mount can: measured from the nose within its travel,
    // not the shortest way round, which can run into a travel limit and stay
    // there (e.g. from -160 to +45 the short way passes behind the drone)
    const float now_rel = WrapPi(heading_ - vehicle_yaw);
    const float wanted_rel =
        std::max(d.min_angle[2], std::min(d.max_angle[2], WrapPi(wanted - vehicle_yaw)));
    const bool full_circle = d.max_angle[2] - d.min_angle[2] >= 2.0f * static_cast<float>(M_PI);
    const float error = full_circle ? WrapPi(wanted - heading_) : wanted_rel - now_rel;
    heading_ = WrapPi(Approach(heading_, heading_ + error, 2, d.settling_time, dt, false));
    // The mount's yaw travel is measured from the nose
    yaw = WrapPi(heading_ - vehicle_yaw);
    const float clamped =
        std::max(d.min_angle[2], std::min(d.max_angle[2], yaw));
    if (clamped != yaw) {
      yaw = clamped;
      heading_ = WrapPi(vehicle_yaw + yaw);
    }
  } else {
    const float target_yaw =
        std::max(d.min_angle[2], std::min(d.max_angle[2], target_[2]));
    yaw = Approach(gimbal_state_.yaw, target_yaw, 2, d.settling_time, dt,
                   false);
  }

  // Need to pass these on to Unreal?
  gimbal_state_.yaw_lock = new_state.yaw_lock;
  gimbal_state_.roll_lock = new_state.roll_lock;
  gimbal_state_.pitch_lock = new_state.pitch_lock;

  gimbal_state_.yaw_vel = new_state.yaw_vel;
  gimbal_state_.roll_vel = new_state.roll_vel;
  gimbal_state_.pitch_vel = new_state.pitch_vel;

  if (roll == gimbal_state_.roll && pitch == gimbal_state_.pitch &&
      yaw == gimbal_state_.yaw) {
    return;
  }
  gimbal_state_.roll = roll;
  gimbal_state_.pitch = pitch;
  gimbal_state_.yaw = yaw;
  auto quat_output_ = TransformUtils::ToQuaternion(
      gimbal_state_.roll, gimbal_state_.pitch, gimbal_state_.yaw);

  actuated_transforms_[child_link_] =
      ActuatedTransform(Affine3(quat_output_.toRotationMatrix()));
}

void Gimbal::Impl::UpdateActuatorOutput(const ControlSignals& control_signals,
                                        const TimeNano sim_dt_nanos) {
  return;
}

const ActuatedTransforms& Gimbal::Impl::GetActuatedTransforms() const {
  return actuated_transforms_;
}

double Gimbal::Impl::calcSetpoint(double dt, double lastSetpoint,
                                  double newSetpoint, double newRateSetpoint) {
  const bool setpointValid = std::isfinite(newSetpoint);
  // A zero rate with an angle means "go to the angle": PX4 sends its mission
  // gimbal steps (DO_GIMBAL_MANAGER_PITCHYAW) that way. Taken as a rate it
  // would hold the gimbal still, capped only on one side by the angle.
  const bool rateSetpointValid =
      std::isfinite(newRateSetpoint) &&
      !(setpointValid && std::abs(newRateSetpoint) < 1e-6);

  if (rateSetpointValid) {
    const double rateDiff = dt * newRateSetpoint;
    const double setpointFromRate = lastSetpoint + rateDiff;

    if (setpointValid) {
      // In this case angle and rate are valid, so we use the rate but constrain
      // it by the angle.
      if (rateDiff > 0.0) {
        return std::min(newSetpoint, setpointFromRate);
      } else {
        return std::max(newSetpoint, setpointFromRate);
      }

    } else {
      // Only the rate is valid, so we just use it.
      return setpointFromRate;
    }

  } else if (setpointValid) {
    // Only the angle is valid.
    return newSetpoint;
  }
  return lastSetpoint;
}

//------------------------------------------------------------------------------
// class Gimbal::Loader

Gimbal::Loader::Loader(Gimbal::Impl& impl) : impl_(impl) {}

void Gimbal::Loader::Load(const json& json) {
  impl_.logger_.LogVerbose(impl_.name_, "[%s] Loading 'gimbal'.",
                           impl_.id_.c_str());

  impl_.is_loaded_ = true;
  LoadOriginSettings(json);
  LoadGimbalSettings(json);
  impl_.logger_.LogVerbose(impl_.name_, "[%s] 'gimbal loaded.",
                           impl_.id_.c_str());
}
void Gimbal::Loader::LoadOriginSettings(const json& json) {
  impl_.logger_.LogVerbose(impl_.name_, "Loading gimbal origin settings.");
  auto origin_json = JsonUtils::GetJsonObject(json, Constant::Config::origin);
  if (JsonUtils::IsEmpty(origin_json)) {
    impl_.logger_.LogVerbose(impl_.name_,
                             "'origin' missing or empty. Using default.");
  } else {
    auto origin = JsonUtils::GetTransform(json, Constant::Config::origin);
    auto rpy = TransformUtils::ToRPY(origin.rotation_);

    impl_.gimbal_state_.roll = rpy.x();
    impl_.gimbal_state_.pitch = rpy.y();
    impl_.gimbal_state_.yaw = rpy.z();
  }
  impl_.logger_.LogVerbose(impl_.name_, "'gimbal origin settings' loaded.");
}

void Gimbal::Loader::LoadGimbalSettings(const json& json) {
  // Optional "gimbal-settings": how the physical gimbal moves. Without them
  // every commanded angle is reached at once, as by an ideal gimbal.
  auto settings = JsonUtils::GetJsonObject(json, "gimbal-settings");
  if (JsonUtils::IsEmpty(settings)) return;
  auto& d = impl_.dynamics_;
  const float deg = static_cast<float>(M_PI) / 180.0f;
  const char* axes[3] = {"roll", "pitch", "yaw"};
  for (int i = 0; i < 3; ++i) {
    const std::string axis = axes[i];
    d.max_rate[i] = JsonUtils::GetNumber<float>(
                        settings, "max-" + axis + "-rate-deg", INFINITY) * deg;
    d.min_angle[i] = JsonUtils::GetNumber<float>(
                         settings, axis + "-min-deg", -INFINITY) * deg;
    d.max_angle[i] = JsonUtils::GetNumber<float>(
                         settings, axis + "-max-deg", INFINITY) * deg;
  }
  d.settling_time =
      JsonUtils::GetNumber<float>(settings, "settling-time-s", 0.0f);
  d.yaw_follow_time =
      JsonUtils::GetNumber<float>(settings, "yaw-follow-time-s", 0.0f);
  impl_.logger_.LogVerbose(
      impl_.name_,
      "[%s] gimbal-settings: max rate %.0f/%.0f/%.0f deg/s, settling %.2f s, "
      "yaw follow %.2f s",
      impl_.id_.c_str(), d.max_rate[0] / deg, d.max_rate[1] / deg,
      d.max_rate[2] / deg, d.settling_time, d.yaw_follow_time);
}

}  // namespace projectairsim
}  // namespace microsoft
