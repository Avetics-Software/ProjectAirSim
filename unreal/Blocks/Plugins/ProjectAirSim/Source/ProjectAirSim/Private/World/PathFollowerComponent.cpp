// Copyright (C) 2025 IAMAI CONSULTING CORP
// MIT License.

#include "PathFollowerComponent.h"

#include "Animation/AnimSequence.h"
#include "Animation/AnimSequenceBase.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

TArray<TWeakObjectPtr<AActor>> UPathFollowerComponent::followers_;

namespace {
constexpr float kTurnRateDegPerSec = 180.0f;
constexpr float kStepUpCm = 200.0f;      // ground is searched from this high
constexpr float kSearchUpCm = 10000.0f;  // ...or from this high if not found
constexpr float kSearchDownCm = 10000.0f;
constexpr float kMinSegmentCm = 1.0f;
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
  for (const FVector& p : points) points_.Add(FVector2D(p.X, p.Y));
  speed_ = speed_mps * 100.0f;
  mode_ = mode;
  yaw_offset_ = yaw_offset_deg;
  align_pitch_ = align_pitch;
  walk_anim_ = walk_anim;
  idle_anim_ = idle_anim;
  anim_speed_ = anim_speed_mps * 100.0f;
  delay_ = start_delay_s;
  segment_ = 0;
  direction_ = 1;
  along_ = 0.0f;
  paused_ = false;
  finished_ = points_.Num() < 2;
  anim_set_ = false;

  AActor* owner = GetOwner();
  const FVector extent = owner->GetComponentsBoundingBox(true).GetExtent();
  half_length_ = 0.8f * FMath::Max(extent.X, extent.Y);

  // Keep the height above ground the object was spawned with
  float ground_z;
  const FVector location = owner->GetActorLocation();
  have_offset_ = TraceGround(FVector2D(location), location.Z, ground_z);
  if (have_offset_) ground_offset_ = location.Z - ground_z;

  followers_.AddUnique(owner);
  SetComponentTickEnabled(true);
}

void UPathFollowerComponent::SetPaused(bool paused) { paused_ = paused; }

void UPathFollowerComponent::Stop() {
  finished_ = true;
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

bool UPathFollowerComponent::TraceGround(const FVector2D& xy, float near_z,
                                         float& ground_z) const {
  FCollisionQueryParams params(SCENE_QUERY_STAT(PathFollowerGround), true);
  for (const TWeakObjectPtr<AActor>& follower : followers_) {
    if (follower.IsValid()) params.AddIgnoredActor(follower.Get());
  }
  params.AddIgnoredActor(GetOwner());

  // Search from just above the object first so it walks under bridges and
  // trees instead of onto them
  const float starts[] = {near_z + kStepUpCm, near_z + kSearchUpCm};
  for (float start_z : starts) {
    FHitResult hit;
    if (GetWorld()->LineTraceSingleByChannel(
            hit, FVector(xy.X, xy.Y, start_z),
            FVector(xy.X, xy.Y, near_z - kSearchDownCm), ECC_Visibility,
            params)) {
      ground_z = hit.ImpactPoint.Z;
      return true;
    }
  }
  return false;
}

void UPathFollowerComponent::TickComponent(
    float delta_time, ELevelTick tick_type,
    FActorComponentTickFunction* tick_fn) {
  Super::TickComponent(delta_time, tick_type, tick_fn);
  if (finished_ || paused_) {
    SetMoving(false);
    return;
  }
  if (delay_ > 0.0f) {
    delay_ -= delta_time;
    SetMoving(false);
    return;
  }

  const int32 n = points_.Num();
  auto next_index = [&](int32 i) {
    return mode_ == EPathFollowMode::Loop ? (i + 1) % n : i + direction_;
  };

  // Advance along the path, carrying distance over waypoints
  float distance = speed_ * delta_time;
  for (int32 guard = 0; guard < 2 * n + 2; ++guard) {
    const int32 to = next_index(segment_);
    const float length = FVector2D::Distance(points_[segment_], points_[to]);
    if (distance < length - along_ && length > kMinSegmentCm) {
      along_ += distance;
      break;
    }
    distance -= FMath::Max(length - along_, 0.0f);
    along_ = 0.0f;
    segment_ = to;
    if (mode_ == EPathFollowMode::Once && segment_ == n - 1) {
      finished_ = true;
      break;
    }
    if (mode_ == EPathFollowMode::PingPong &&
        (segment_ == n - 1 || segment_ == 0)) {
      direction_ = segment_ == 0 ? 1 : -1;
    }
  }

  AActor* owner = GetOwner();
  FVector2D from = points_[segment_];
  FVector2D to = finished_ ? from : points_[next_index(segment_)];
  if (finished_) from = points_[FMath::Max(segment_ - 1, 0)];
  const FVector2D dir = (to - from).GetSafeNormal();
  const FVector2D xy =
      finished_ ? to : points_[segment_] + dir * along_;

  // Height from the ground under the new position
  const FVector location = owner->GetActorLocation();
  float z = location.Z;
  float ground_z;
  if (TraceGround(xy, location.Z - ground_offset_, ground_z)) {
    if (!have_offset_) {
      ground_offset_ = location.Z - ground_z;
      have_offset_ = true;
    }
    z = ground_z + ground_offset_;
  }

  FRotator rotation = owner->GetActorRotation();
  if (!dir.IsNearlyZero()) {
    const float target_yaw =
        FMath::RadiansToDegrees(FMath::Atan2(dir.Y, dir.X)) + yaw_offset_;
    rotation.Yaw = FMath::FixedTurn(rotation.Yaw, target_yaw,
                                    kTurnRateDegPerSec * delta_time);
  }
  if (align_pitch_ && half_length_ > 0.0f && !dir.IsNearlyZero()) {
    float front_z, back_z;
    const float near_z = z - ground_offset_;
    if (TraceGround(xy + dir * half_length_, near_z, front_z) &&
        TraceGround(xy - dir * half_length_, near_z, back_z)) {
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
  SetMoving(!finished_);
}

void UPathFollowerComponent::EndPlay(const EEndPlayReason::Type reason) {
  followers_.Remove(GetOwner());
  followers_.RemoveAll(
      [](const TWeakObjectPtr<AActor>& actor) { return !actor.IsValid(); });
  Super::EndPlay(reason);
}
