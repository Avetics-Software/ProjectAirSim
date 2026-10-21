// Copyright (C) 2025 IAMAI CONSULTING CORP
// MIT License.
//
// Moves a spawned object along a path every frame, staying on the ground.
// Attached at runtime by the SetObjectPath service so that many walkers and
// vehicles can move without the client sending a pose per frame.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "PathFollowerComponent.generated.h"

class UAnimSequenceBase;
class USkeletalMeshComponent;

UENUM()
enum class EPathFollowMode : uint8 { Once, Loop, PingPong };

UCLASS()
class UPathFollowerComponent : public UActorComponent {
  GENERATED_BODY()

 public:
  UPathFollowerComponent();

  // Points are Unreal world positions; only X and Y are used, Z comes from
  // the ground under the object.
  void Start(const TArray<FVector>& points, float speed_mps,
             EPathFollowMode mode, float yaw_offset_deg, bool align_pitch,
             UAnimSequenceBase* walk_anim, UAnimSequenceBase* idle_anim,
             float anim_speed_mps, float start_delay_s);

  void SetPaused(bool paused);

  // Leaves the object where it is, playing its idle animation
  void Stop();

  // Plays an animation on a skeletal mesh, compressing it first when running
  // from the editor binary (see WorldSimApi::setObjectAnimation)
  static void PlayAnim(USkeletalMeshComponent* mesh, UAnimSequenceBase* anim,
                       float play_rate, bool loop = true);

  virtual void TickComponent(float delta_time, ELevelTick tick_type,
                             FActorComponentTickFunction* tick_fn) override;

 protected:
  virtual void EndPlay(const EEndPlayReason::Type reason) override;

 private:
  bool TraceGround(const FVector2D& xy, float near_z, float& ground_z) const;
  void SetMoving(bool moving);

  TArray<FVector2D> points_;
  float speed_ = 0.0f;  // cm/s
  EPathFollowMode mode_ = EPathFollowMode::Once;
  float yaw_offset_ = 0.0f;
  bool align_pitch_ = false;
  float half_length_ = 0.0f;  // cm, for sampling the ground slope

  UPROPERTY()
  UAnimSequenceBase* walk_anim_ = nullptr;
  UPROPERTY()
  UAnimSequenceBase* idle_anim_ = nullptr;
  float anim_speed_ = 0.0f;  // cm/s at which the walk animation plays at 1x

  int32 segment_ = 0;     // moving from points_[segment_] to the next point
  int32 direction_ = 1;   // -1 while returning in PingPong mode
  float along_ = 0.0f;    // cm travelled along the current segment
  float delay_ = 0.0f;
  float ground_offset_ = 0.0f;  // object Z above the ground
  bool have_offset_ = false;
  bool paused_ = false;
  bool finished_ = false;
  bool moving_ = false;
  bool anim_set_ = false;

  // All objects following paths, ignored when tracing for the ground
  static TArray<TWeakObjectPtr<AActor>> followers_;
};
