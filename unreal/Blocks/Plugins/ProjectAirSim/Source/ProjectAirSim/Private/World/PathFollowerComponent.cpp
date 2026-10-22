// Copyright (C) 2025 IAMAI CONSULTING CORP
// MIT License.

#include "PathFollowerComponent.h"

#include "Animation/AnimSequence.h"
#include "Animation/AnimSequenceBase.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "Robot/UnrealRobot.h"

TArray<TWeakObjectPtr<AActor>> UPathFollowerComponent::followers_;

namespace {
constexpr float kTurnRateDegPerSec = 180.0f;
constexpr float kSearchUpCm = 10000.0f;  // ground is searched from this high
constexpr float kSearchDownCm = 10000.0f;
constexpr float kMinSegmentCm = 1.0f;
// The visible surface this far above where the bare-earth terrain puts the
// road is an overhang (canopy, bridge, roof) rather than the road
constexpr float kOverhangCm = 400.0f;
// The photorealistic tiles and the terrain disagree on height by a few metres,
// varying from place to place. The offset between them is tracked as the
// object moves: an overhang only ever raises the visible surface, so lower
// readings are trusted quickly and higher ones slowly.
constexpr float kBiasRateDown = 0.3f;
constexpr float kBiasRateUp = 0.05f;
constexpr float kBiasSampleSpanCm = 5000.0f;  // sample the first 50 m
constexpr int32 kBiasSamples = 11;
constexpr float kBiasPercentile = 0.2f;
// Real overhangs over a road are short. Being "covered" for longer than this
// means the tiles (often a coarse level of detail) and the terrain disagree
// here, so trust the visible surface again.
constexpr float kMaxCoveredCm = 15000.0f;
constexpr float kBridgeStepCm = 200.0f;  // look-ahead step for the end of cover
constexpr float kHeightSmoothing = 10.0f;  // 1/s; smooths steps at overhang edges
}  // namespace

UPathFollowerComponent::UPathFollowerComponent() {
  PrimaryComponentTick.bCanEverTick = true;
  // Stand still while the simulation is paused
  PrimaryComponentTick.bTickEvenWhenPaused = false;
}

void UPathFollowerComponent::Start(const TArray<FVector>& points,
                                   float speed_mps, EPathFollowMode mode,
                                   float yaw_offset_deg, bool align_pitch,
                                   UAnimSequenceBase* walk_anim,
                                   UAnimSequenceBase* idle_anim,
                                   float anim_speed_mps, float start_delay_s) {
  points_.Reset();
  bridge_.Reset();
  for (const FVector& p : points) {
    points_.Add(FVector2D(p.X, p.Y));
    bridge_.Add(p.Z > 0.5f);
  }
  speed_ = speed_mps * 100.0f;
  mode_ = mode;
  yaw_offset_ = yaw_offset_deg;
  align_pitch_ = align_pitch;
  walk_anim_ = walk_anim;
  idle_anim_ = idle_anim;
  anim_speed_ = anim_speed_mps * 100.0f;
  delay_ = start_delay_s;
  cursor_ = FPathCursor();
  cursor_.finished = points_.Num() < 2;
  paused_ = false;
  anim_set_ = false;

  AActor* owner = GetOwner();
  const FVector extent = owner->GetComponentsBoundingBox(true).GetExtent();
  half_length_ = 0.8f * FMath::Max(extent.X, extent.Y);

  // Keep the height above ground the object was spawned with. It was set on
  // the visible surface (ObjectSpawner), so measure against that.
  float ground_z;
  const FVector location = owner->GetActorLocation();
  have_offset_ = TraceSurface(FVector2D(location), location.Z, ground_z);
  if (have_offset_) ground_offset_ = location.Z - ground_z;
  covered_ = false;
  have_z_ = false;
  bridging_ = false;
  have_open_z_ = false;
  EstimateEarthBias(location.Z);

  robots_.Reset();
  for (TActorIterator<AUnrealRobot> it(GetWorld()); it; ++it) robots_.Add(*it);

  followers_.AddUnique(owner);
  SetComponentTickEnabled(true);
}

void UPathFollowerComponent::SetPaused(bool paused) { paused_ = paused; }

void UPathFollowerComponent::Stop() {
  cursor_.finished = true;
  SetMoving(false);
}

void UPathFollowerComponent::PlayAnim(USkeletalMeshComponent* mesh,
                                      UAnimSequenceBase* anim,
                                      float play_rate, bool loop) {
  if (!mesh || !anim) return;
#if WITH_EDITOR
  // Running from the editor binary, an animation that hasn't finished
  // compressing is evaluated from its raw data model. For animations authored
  // with Control Rig that path can crash on the parallel animation thread, so
  // compress it now and play the compressed data.
  if (UAnimSequence* sequence = Cast<UAnimSequence>(anim)) {
    sequence->CacheDerivedDataForCurrentPlatform();
    sequence->WaitOnExistingCompression(true);
  }
#endif
  mesh->PlayAnimation(anim, loop);
  mesh->SetPlayRate(play_rate);
}

void UPathFollowerComponent::SetMoving(bool moving) {
  if (anim_set_ && moving == moving_) return;
  moving_ = moving;
  anim_set_ = true;
  USkeletalMeshComponent* mesh =
      GetOwner()->FindComponentByClass<USkeletalMeshComponent>();
  if (!mesh) return;
  if (moving && walk_anim_) {
    PlayAnim(mesh, walk_anim_, anim_speed_ > 0.0f ? speed_ / anim_speed_ : 1.0f);
  } else if (!moving && idle_anim_) {
    PlayAnim(mesh, idle_anim_, 1.0f);
  }
}

bool UPathFollowerComponent::TraceSurface(const FVector2D& xy, float near_z,
                                          float& z) const {
  // The top of the visible world, as a camera sees it, but not drones or
  // moving objects. A placed object counts only if its underside is well
  // above the road (a roof or shelter, which can be an overhang); objects
  // standing on the ground, like parked cars, are passed through.
  FCollisionQueryParams params(SCENE_QUERY_STAT(PathFollowerSurface), true);
  for (const TWeakObjectPtr<AActor>& follower : followers_) {
    if (follower.IsValid()) params.AddIgnoredActor(follower.Get());
  }
  for (const TWeakObjectPtr<AActor>& robot : robots_) {
    if (robot.IsValid()) params.AddIgnoredActor(robot.Get());
  }
  params.AddIgnoredActor(GetOwner());
  for (int32 attempt = 0; attempt < 4; ++attempt) {
    FHitResult hit;
    if (!GetWorld()->LineTraceSingleByChannel(
            hit, FVector(xy.X, xy.Y, near_z + kSearchUpCm),
            FVector(xy.X, xy.Y, near_z - kSearchDownCm), ECC_Visibility,
            params)) {
      return false;
    }
    AActor* actor = hit.GetActor();
    const bool is_tiles =
        actor && actor->GetClass()->GetName().Contains(TEXT("Cesium"));
    if (actor && !is_tiles &&
        actor->GetComponentsBoundingBox(true).Min.Z < near_z + kOverhangCm) {
      params.AddIgnoredActor(actor);
      continue;
    }
    z = hit.ImpactPoint.Z;
    return true;
  }
  return false;
}

bool UPathFollowerComponent::TraceBareEarth(const FVector2D& xy, float near_z,
                                            float& z) const {
  FHitResult hit;
  if (!GetWorld()->LineTraceSingleByObjectType(
          hit, FVector(xy.X, xy.Y, near_z + kSearchUpCm),
          FVector(xy.X, xy.Y, near_z - kSearchDownCm),
          FCollisionObjectQueryParams(kBareEarthObjectType),
          FCollisionQueryParams(SCENE_QUERY_STAT(PathFollowerEarth), true))) {
    return false;
  }
  z = hit.ImpactPoint.Z;
  return true;
}

void UPathFollowerComponent::EstimateEarthBias(float near_z) {
  // Sample the surface/terrain offset near the start of the path and take a
  // low value, which comes from open ground rather than from under trees
  have_bias_ = false;
  covered_cm_ = 0.0f;
  float total = 0.0f;
  for (int32 i = 1; i < points_.Num(); ++i) {
    total += FVector2D::Distance(points_[i - 1], points_[i]);
  }
  total = FMath::Min(total, kBiasSampleSpanCm);
  TArray<float> offsets;
  for (int32 k = 0; k < kBiasSamples && total > 0.0f; ++k) {
    float target = total * k / (kBiasSamples - 1);
    for (int32 i = 1; i < points_.Num(); ++i) {
      const float length = FVector2D::Distance(points_[i - 1], points_[i]);
      if (target > length) {
        target -= length;
        continue;
      }
      const FVector2D xy = points_[i - 1] + (points_[i] - points_[i - 1]) *
                                                (length > 0 ? target / length : 0);
      float surface_z, earth_z;
      if (TraceSurface(xy, near_z, surface_z) &&
          TraceBareEarth(xy, near_z, earth_z)) {
        offsets.Add(surface_z - earth_z);
      }
      break;
    }
  }
  if (offsets.Num() > 0) {
    offsets.Sort();
    earth_bias_ = offsets[FMath::FloorToInt(kBiasPercentile * (offsets.Num() - 1))];
    have_bias_ = true;
  }
}

bool UPathFollowerComponent::GroundAt(const FVector2D& xy, float near_z,
                                      bool on_bridge, bool learn,
                                      float& ground_z, bool& covered) {
  float surface_z, earth_z;
  const bool has_surface = TraceSurface(xy, near_z, surface_z);
  const bool has_earth = TraceBareEarth(xy, near_z, earth_z);
  covered = false;
  if (has_surface && has_earth) {
    const float offset = surface_z - earth_z;
    if (!have_bias_ && learn) {
      earth_bias_ = offset;
      have_bias_ = true;
    }
    const float above = offset - earth_bias_;
    if (above > kOverhangCm && !on_bridge) {
      covered = true;
      ground_z = earth_z + earth_bias_;
      return true;
    }
    if (learn) {
      earth_bias_ += (above < 0.0f ? kBiasRateDown : kBiasRateUp) * above;
    }
    ground_z = surface_z;
    return true;
  }
  if (has_surface) {
    ground_z = surface_z;
    return true;
  }
  if (has_earth) {
    ground_z = earth_z + earth_bias_;
    return true;
  }
  return false;
}

void UPathFollowerComponent::Advance(FPathCursor& cursor,
                                     float distance) const {
  // Move along the path, carrying distance over waypoints
  const int32 n = points_.Num();
  for (int32 guard = 0; guard < 2 * n + 2 && !cursor.finished; ++guard) {
    const int32 to = NextIndex(cursor, cursor.segment);
    const float length =
        FVector2D::Distance(points_[cursor.segment], points_[to]);
    if (distance < length - cursor.along && length > kMinSegmentCm) {
      cursor.along += distance;
      return;
    }
    distance -= FMath::Max(length - cursor.along, 0.0f);
    cursor.along = 0.0f;
    cursor.segment = to;
    if (mode_ == EPathFollowMode::Once && cursor.segment == n - 1) {
      cursor.finished = true;
      return;
    }
    if (mode_ == EPathFollowMode::PingPong &&
        (cursor.segment == n - 1 || cursor.segment == 0)) {
      cursor.direction = cursor.segment == 0 ? 1 : -1;
    }
  }
}

int32 UPathFollowerComponent::NextIndex(const FPathCursor& cursor,
                                        int32 i) const {
  return mode_ == EPathFollowMode::Loop ? (i + 1) % points_.Num()
                                        : i + cursor.direction;
}

void UPathFollowerComponent::Locate(const FPathCursor& cursor, FVector2D& xy,
                                    FVector2D& dir, bool& on_bridge) const {
  const int32 seg_to = cursor.finished ? cursor.segment
                                       : NextIndex(cursor, cursor.segment);
  FVector2D from = points_[cursor.segment];
  FVector2D to = points_[seg_to];
  if (cursor.finished) from = points_[FMath::Max(cursor.segment - 1, 0)];
  dir = (to - from).GetSafeNormal();
  xy = cursor.finished ? to : points_[cursor.segment] + dir * cursor.along;
  on_bridge = bridge_[cursor.segment] || bridge_[seg_to];
}

void UPathFollowerComponent::StartBridging(float near_z) {
  // Look ahead along the path for where the cover ends, then ramp the road
  // height from where it began to there. The terrain under trees is too
  // coarse to drive on directly.
  bridging_ = false;
  for (float ahead = kBridgeStepCm; ahead <= kMaxCoveredCm;
       ahead += kBridgeStepCm) {
    FPathCursor probe = cursor_;
    Advance(probe, ahead);
    FVector2D xy, dir;
    bool on_bridge, covered;
    float ground_z;
    Locate(probe, xy, dir, on_bridge);
    if (GroundAt(xy, near_z, on_bridge, false, ground_z, covered) && !covered) {
      bridging_ = true;
      bridge_from_z_ = have_open_z_ ? last_open_z_ : near_z;
      bridge_to_z_ = ground_z;
      bridge_length_ = ahead;
      bridge_done_ = 0.0f;
      return;
    }
    if (probe.finished) return;
  }
}

void UPathFollowerComponent::TickComponent(
    float delta_time, ELevelTick tick_type,
    FActorComponentTickFunction* tick_fn) {
  Super::TickComponent(delta_time, tick_type, tick_fn);
  if (cursor_.finished || paused_) {
    SetMoving(false);
    return;
  }
  if (delay_ > 0.0f) {
    delay_ -= delta_time;
    SetMoving(false);
    return;
  }

  const float travelled = speed_ * delta_time;
  Advance(cursor_, travelled);
  FVector2D xy, dir;
  bool on_bridge;
  Locate(cursor_, xy, dir, on_bridge);

  // Height from the ground under the new position
  AActor* owner = GetOwner();
  const FVector location = owner->GetActorLocation();
  const float near_z = location.Z - ground_offset_;
  float z = location.Z;
  float ground_z;
  bool covered;
  if (GroundAt(xy, near_z, on_bridge, true, ground_z, covered)) {
    if (!have_offset_) {
      ground_offset_ = location.Z - ground_z;
      have_offset_ = true;
    }
    covered_cm_ = covered ? covered_cm_ + travelled : 0.0f;
    if (covered_cm_ > kMaxCoveredCm) {
      // Not a real overhang: re-anchor to the visible surface
      float surface_z, earth_z;
      if (TraceSurface(xy, near_z, surface_z) &&
          TraceBareEarth(xy, near_z, earth_z)) {
        earth_bias_ = surface_z - earth_z;
        ground_z = surface_z;
      }
      covered = false;
      covered_cm_ = 0.0f;
    }
    if (covered) {
      if (!covered_) StartBridging(near_z);
      if (bridging_) {
        bridge_done_ += travelled;
        ground_z = FMath::Lerp(bridge_from_z_, bridge_to_z_,
                               FMath::Clamp(bridge_done_ / bridge_length_, 0.0f,
                                            1.0f));
      }
    } else {
      bridging_ = false;
      last_open_z_ = ground_z;
      have_open_z_ = true;
    }
    covered_ = covered;
    const float target = ground_z + ground_offset_;
    // Ease the small steps where the object goes under or out of cover
    z = have_z_ ? FMath::FInterpTo(location.Z, target, delta_time,
                                   kHeightSmoothing)
                : target;
    have_z_ = true;
  }

  FRotator rotation = owner->GetActorRotation();
  if (!dir.IsNearlyZero()) {
    const float target_yaw =
        FMath::RadiansToDegrees(FMath::Atan2(dir.Y, dir.X)) + yaw_offset_;
    rotation.Yaw = FMath::FixedTurn(rotation.Yaw, target_yaw,
                                    kTurnRateDegPerSec * delta_time);
  }
  // Tilt with the slope in the open; under cover the ground isn't visible,
  // so keep the last tilt
  if (align_pitch_ && half_length_ > 0.0f && !dir.IsNearlyZero() && !covered_) {
    float front_z, back_z;
    bool front_covered, back_covered;
    if (GroundAt(xy + dir * half_length_, near_z, on_bridge, false, front_z,
                 front_covered) &&
        GroundAt(xy - dir * half_length_, near_z, on_bridge, false, back_z,
                 back_covered) &&
        !front_covered && !back_covered) {
      const float pitch = FMath::RadiansToDegrees(
          FMath::Atan2(front_z - back_z, 2.0f * half_length_));
      // The mesh may face along its Y axis (yaw_offset); tilt about the
      // axis across the direction of travel
      const float offset = FMath::DegreesToRadians(yaw_offset_);
      rotation.Pitch = pitch * FMath::Cos(offset);
      rotation.Roll = pitch * FMath::Sin(offset);
    }
  }

  owner->SetActorLocationAndRotation(FVector(xy.X, xy.Y, z), rotation, false,
                                     nullptr, ETeleportType::TeleportPhysics);
  SetMoving(!cursor_.finished);
}

void UPathFollowerComponent::EndPlay(const EEndPlayReason::Type reason) {
  followers_.Remove(GetOwner());
  followers_.RemoveAll(
      [](const TWeakObjectPtr<AActor>& actor) { return !actor.IsValid(); });
  Super::EndPlay(reason);
}
