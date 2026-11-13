#include "execution/experiment_manager.h"

#include <algorithm>
#include <cmath>
#include <map>

#include "common/math/math_utils.h"

namespace execution {
namespace {

double PoseDistance(const scene::Pose2d& lhs, const scene::Pose2d& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double YawDistance(double lhs, double rhs) {
  return std::abs(common::math::NormalizeAngle(lhs - rhs));
}

}  // namespace

ExperimentManager::ExperimentManager(Configuration configuration)
    : configuration_(configuration) {}

bool ExperimentManager::SceneMoved(const scene::SceneSnapshot& lhs,
                                   const scene::SceneSnapshot& rhs) const {
  if (lhs.obstacles.size() != rhs.obstacles.size()) return true;
  std::map<std::string, const scene::SceneObstacle*> lookup;
  for (const auto& obstacle : lhs.obstacles) lookup[obstacle.id] = &obstacle;
  for (const auto& obstacle : rhs.obstacles) {
    const auto found = lookup.find(obstacle.id);
    if (found == lookup.end()) return true;
    if (std::hypot(found->second->x - obstacle.x,
                   found->second->y - obstacle.y) >
            configuration_.scene_position_threshold ||
        YawDistance(found->second->yaw, obstacle.yaw) >
            configuration_.scene_yaw_threshold ||
        std::abs(found->second->length - obstacle.length) >
            configuration_.scene_position_threshold ||
        std::abs(found->second->width - obstacle.width) >
            configuration_.scene_position_threshold) return true;
  }
  return false;
}

void ExperimentManager::Observe(const scene::SceneSnapshot& snapshot,
                                bool data_valid, std::uint64_t revision,
                                double now) {
  latest_ = snapshot;
  latest_revision_ = revision;
  have_latest_ = true;
  if (!data_valid) {
    frozen_scene_valid_ = false;
    if (state_ == State::kPlanning || state_ == State::kTracking)
      state_ = State::kWaitingForData;
    else if (state_ != State::kCompleted)
      state_ = State::kWaitingForData;
    return;
  }
  if (state_ == State::kWaitingForData) {
    stable_reference_ = snapshot;
    stable_since_ = now;
    state_ = State::kWaitingForStationary;
    return;
  }
  if (state_ == State::kPlanning) {
    if (PoseDistance(snapshot.start, frozen_.start) >
            configuration_.scene_position_threshold ||
        YawDistance(snapshot.start.yaw, frozen_.start.yaw) >
            configuration_.scene_yaw_threshold ||
        SceneMoved(snapshot, frozen_)) {
      frozen_scene_valid_ = false;
    }
    return;
  }
  if (state_ == State::kTracking) {
    // Vehicle motion is expected while tracking. Only the frozen obstacle set
    // (or data validity handled above) can invalidate an executing task.
    if (SceneMoved(snapshot, frozen_)) {
      frozen_scene_valid_ = false;
      state_ = State::kWaitingForSceneChange;
    }
    return;
  }
  if (state_ == State::kCompleted) {
    if (have_stopped_pose_ &&
        PoseDistance(snapshot.start, stopped_pose_) >=
            configuration_.rearm_distance) {
      stable_reference_ = snapshot;
      stable_since_ = now;
      state_ = State::kWaitingForStationary;
    }
    return;
  }
  if (state_ == State::kWaitingForSceneChange) {
    if (SceneMoved(snapshot, frozen_) ||
        PoseDistance(snapshot.start, frozen_.start) >
            configuration_.scene_position_threshold) {
      stable_reference_ = snapshot;
      stable_since_ = now;
      state_ = State::kWaitingForStationary;
    }
    return;
  }
  if (state_ != State::kWaitingForStationary) return;
  const bool moving =
      PoseDistance(snapshot.start, stable_reference_.start) >
          configuration_.position_threshold ||
      YawDistance(snapshot.start.yaw, stable_reference_.start.yaw) >
          configuration_.yaw_threshold ||
      SceneMoved(snapshot, stable_reference_);
  if (moving) {
    stable_reference_ = snapshot;
    stable_since_ = now;
  } else if (now - stable_since_ >= configuration_.stationary_duration) {
    state_ = State::kReadyToPlan;
  }
}

bool ExperimentManager::BeginPlanning(scene::SceneSnapshot* frozen,
                                      std::uint64_t* generation) {
  if (state_ != State::kReadyToPlan || !have_latest_) return false;
  frozen_ = latest_;
  frozen_.task_id = static_cast<int>(++task_id_);
  frozen_revision_ = latest_revision_;
  frozen_scene_valid_ = true;
  state_ = State::kPlanning;
  if (frozen) *frozen = frozen_;
  if (generation) *generation = frozen_revision_;
  return true;
}

void ExperimentManager::PlanningFinished(bool success, bool execute) {
  if (state_ != State::kPlanning) return;
  if (!success || !frozen_scene_valid_) {
    state_ = State::kWaitingForSceneChange;
    return;
  }
  state_ = execute ? State::kTracking : State::kCompleted;
  if (!execute) {
    stopped_pose_ = latest_.start;
    have_stopped_pose_ = true;
  }
}

void ExperimentManager::Reached(const scene::Pose2d& stopped_pose) {
  if (state_ != State::kTracking) return;
  stopped_pose_ = stopped_pose;
  have_stopped_pose_ = true;
  state_ = State::kCompleted;
}

void ExperimentManager::ForceSafeWait() {
  frozen_scene_valid_ = false;
  state_ = State::kWaitingForData;
}

std::string ExperimentManager::StateName() const {
  switch (state_) {
    case State::kWaitingForData: return "Waiting for mocap";
    case State::kWaitingForStationary: return "Waiting for stationary scene";
    case State::kReadyToPlan: return "Ready to plan";
    case State::kPlanning: return "Planning";
    case State::kTracking: return "Tracking";
    case State::kCompleted: return "Completed; move vehicle to rearm";
    case State::kWaitingForSceneChange: return "Waiting for scene change";
  }
  return "Unknown";
}

}  // namespace execution
