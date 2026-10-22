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

// Collision object type of the hidden bare-earth terrain tileset that
// UnrealSimLoader adds to Cesium scenes. Nothing else uses this type, and the
// terrain ignores every trace channel, so only object-type queries see it.
constexpr ECollisionChannel kBareEarthObjectType = ECC_Destructible;

UENUM()
enum class EPathFollowMode : uint8 { Once, Loop, PingPong };

// Where an object is along its path
struct FPathCursor {
  int32 segment = 0;    // moving from point `segment` to the next one
  float along = 0.0f;   // cm travelled along the current segment
  int32 direction = 1;  // -1 while returning in PingPong mode
  bool finished = false;
};

UCLASS()
class UPathFollowerComponent : public UActorComponent {
  GENERATED_BODY()

 public:
  UPathFollowerComponent();

  // Points are Unreal world positions; only X and Y are used, Z comes from
  // the ground under the object. A point with Z > 0.5 is on a bridge: there
  // the object stays on the visible surface instead of going under it.
  void Start(const TArray<FVector>& points, float speed_mps,
             EPathFollowMode mode, float yaw_offset_deg, bool align_pitch,
             UAnimSequenceBase* walk_anim, UAnimSequenceBase* idle_anim,
             float anim_speed_mps, float start_delay_s);

  void SetPaused(bool paused);

  // True while the object is under an overhang (tree canopy, a bridge, a
  // roof) and driving on the bare-earth terrain below it
  bool IsCovered() const { return covered_; }

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
  // Ground height under xy. The photorealistic tiles are a single surface:
  // where trees or a bridge overhang a road, their top is all there is. Where
  // that surface is well above where the hidden bare-earth terrain puts the
  // road (terrain plus the surface/terrain offset tracked on open ground),
  // the object goes under it, unless the point is on a bridge.
  bool GroundAt(const FVector2D& xy, float near_z, bool on_bridge, bool learn,
                float& ground_z, bool& covered);
  void EstimateEarthBias(float near_z);
  void Advance(FPathCursor& cursor, float distance) const;
  int32 NextIndex(const FPathCursor& cursor, int32 i) const;
  void Locate(const FPathCursor& cursor, FVector2D& xy, FVector2D& dir,
              bool& on_bridge) const;
  // Under cover, ramp the road height from where the cover began to where
  // the path comes out of it
  void StartBridging(float near_z);
  bool TraceSurface(const FVector2D& xy, float near_z, float& z) const;
  bool TraceBareEarth(const FVector2D& xy, float near_z, float& z) const;
  void SetMoving(bool moving);

  TArray<FVector2D> points_;
  TArray<bool> bridge_;  // per point: on a bridge
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

  FPathCursor cursor_;
  float delay_ = 0.0f;
  float ground_offset_ = 0.0f;  // object Z above the ground
  bool have_offset_ = false;
  bool paused_ = false;
  bool moving_ = false;
  bool anim_set_ = false;
  bool covered_ = false;
  float earth_bias_ = 0.0f;  // visible surface minus bare-earth terrain, cm
  bool have_bias_ = false;
  float covered_cm_ = 0.0f;  // distance travelled while covered
  float last_open_z_ = 0.0f;  // ground height last seen in the open
  bool have_open_z_ = false;
  bool bridging_ = false;
  float bridge_from_z_ = 0.0f;
  float bridge_to_z_ = 0.0f;
  float bridge_length_ = 1.0f;
  float bridge_done_ = 0.0f;
  bool have_z_ = false;      // false until the first height is set

  // Drones, ignored when tracing for the ground
  TArray<TWeakObjectPtr<AActor>> robots_;

  // All objects following paths, ignored when tracing for the ground
  static TArray<TWeakObjectPtr<AActor>> followers_;
};
