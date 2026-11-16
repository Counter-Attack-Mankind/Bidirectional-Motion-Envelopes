#include "execution/experiment_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>
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

void ExperimentManager::InitializeTaskSequence(std::uint64_t last_task_id) {
  if (task_id_ == 0 && state_ == State::kWaitingForData)
    task_id_ = last_task_id;
}

double ExperimentManager::ObstacleBoundaryDistance(
    const scene::SceneObstacle& lhs, const scene::SceneObstacle& rhs) {
  const auto left_polygon = lhs.polygon();
  const auto right_polygon = rhs.polygon();
  const auto& left = left_polygon.points();
  const auto& right = right_polygon.points();
  if (left.size() != 4 || right.size() != 4)
    return std::numeric_limits<double>::infinity();
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t offset = 0; offset < 4; ++offset) {
    for (int direction : {1, -1}) {
      double maximum = 0.0;
      for (std::size_t i = 0; i < 4; ++i) {
        const std::size_t index = static_cast<std::size_t>(
            (static_cast<int>(offset) + direction * static_cast<int>(i) + 8) % 4);
        maximum = std::max(maximum, left[i].DistanceTo(right[index]));
      }
      best = std::min(best, maximum);
    }
  }
  return best;
}

bool ExperimentManager::ObstaclesMoved(const scene::SceneSnapshot& lhs,
                                       const scene::SceneSnapshot& rhs,
                                       double boundary_threshold) const {
  if (lhs.obstacles.size() != rhs.obstacles.size()) return true;
  std::map<std::string, const scene::SceneObstacle*> lookup;
  for (const auto& obstacle : lhs.obstacles) lookup[obstacle.id] = &obstacle;
  for (const auto& obstacle : rhs.obstacles) {
    const auto found = lookup.find(obstacle.id);
    if (found == lookup.end()) return true;
    if (ObstacleBoundaryDistance(*found->second, obstacle) >
        boundary_threshold) return true;
  }
  return false;
}

void ExperimentManager::Observe(const scene::SceneSnapshot& snapshot,
                                bool data_valid, std::uint64_t revision,
                                double now) {
  latest_ = snapshot;
  latest_revision_ = revision;
  have_latest_ = true;
  if (state_ == State::kFailedLocked) return;
  if (!data_valid) {
    frozen_scene_valid_ = false;
    if (state_ == State::kPlanning || state_ == State::kTracking) {
      EnterFailedLocked("mocap_data_invalid");
    } else if (state_ != State::kCompleted) {
      state_ = State::kWaitingForData;
    }
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
            configuration_.frozen_vehicle_position_threshold ||
        YawDistance(snapshot.start.yaw, frozen_.start.yaw) >
            configuration_.frozen_vehicle_yaw_threshold ||
        ObstaclesMoved(snapshot, frozen_,
                       configuration_.frozen_obstacle_boundary_threshold)) {
      frozen_scene_valid_ = false;
      EnterFailedLocked("scene_moved_during_planning");
    }
    return;
  }
  if (state_ == State::kTracking) {
    // Vehicle motion is expected while tracking. Only the frozen obstacle set
    // (or data validity handled above) can invalidate an executing task.
    if (ObstaclesMoved(snapshot, frozen_,
                       configuration_.frozen_obstacle_boundary_threshold)) {
      frozen_scene_valid_ = false;
      EnterFailedLocked("obstacle_moved_during_tracking");
    }
    return;
  }
  if (state_ == State::kCompleted) {
    if (task_completed_ && have_stopped_pose_ &&
        PoseDistance(snapshot.start, stopped_pose_) >=
            configuration_.rearm_distance) {
      stable_reference_ = snapshot;
      stable_since_ = now;
      state_ = State::kWaitingForStationary;
    }
    return;
  }
  if (state_ == State::kWaitingForSceneChange) {
    if (ObstaclesMoved(snapshot, frozen_,
                       configuration_.frozen_obstacle_boundary_threshold) ||
        PoseDistance(snapshot.start, frozen_.start) >
            configuration_.frozen_vehicle_position_threshold) {
      stable_reference_ = snapshot;
      stable_since_ = now;
      state_ = State::kWaitingForStationary;
    }
    return;
  }
  if (state_ != State::kWaitingForStationary) return;
  if (task_completed_ && have_stopped_pose_ &&
      PoseDistance(snapshot.start, stopped_pose_) <
          configuration_.rearm_distance) {
    state_ = State::kCompleted;
    return;
  }
  const bool moving =
      PoseDistance(snapshot.start, stable_reference_.start) >
          configuration_.vehicle_stationary_position_threshold ||
      YawDistance(snapshot.start.yaw, stable_reference_.start.yaw) >
          configuration_.vehicle_stationary_yaw_threshold ||
      ObstaclesMoved(snapshot, stable_reference_,
                     configuration_.obstacle_stationary_boundary_threshold);
  if (moving) {
    stable_reference_ = snapshot;
    stable_since_ = now;
  } else if (now - stable_since_ >= configuration_.stationary_duration) {
    task_completed_ = false;
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
  // task_id_ is monotonically increasing even when the mocap geometry
  // revision is unchanged.  Async work must be keyed by this unique token.
  if (generation) *generation = task_id_;
  return true;
}

void ExperimentManager::PlanningFinished(bool success, bool execute,
                                         const std::string& failure_reason) {
  if (state_ != State::kPlanning) return;
  if (!success || !frozen_scene_valid_) {
    EnterFailedLocked(failure_reason);
    return;
  }
  state_ = execute ? State::kTracking : State::kCompleted;
  if (!execute) task_completed_ = false;
}

void ExperimentManager::Reached(const scene::Pose2d& stopped_pose) {
  if (state_ != State::kTracking) return;
  stopped_pose_ = stopped_pose;
  have_stopped_pose_ = true;
  task_completed_ = true;
  state_ = State::kCompleted;
}

void ExperimentManager::EnterFailedLocked(const std::string& reason) {
  task_completed_ = false;
  frozen_scene_valid_ = false;
  failure_reason_ = reason;
  state_ = State::kFailedLocked;
}

void ExperimentManager::TrackingFailed(const std::string& reason) {
  if (state_ != State::kTracking) return;
  EnterFailedLocked(reason);
}

void ExperimentManager::ForceSafeWait(const std::string& reason) {
  EnterFailedLocked(reason);
}

void ExperimentManager::AbortTask(const std::string& reason) {
  EnterFailedLocked(reason);
}

std::string ExperimentManager::StateName() const {
  switch (state_) {
    case State::kWaitingForData: return "Waiting for mocap";
    case State::kWaitingForStationary: return "Waiting for stationary scene";
    case State::kReadyToPlan: return "Ready to plan";
    case State::kPlanning: return "Planning";
    case State::kTracking: return "Tracking";
    case State::kCompleted:
      return task_completed_ ? "Completed; move vehicle to rearm"
                             : "Planning-only task completed; restart required";
    case State::kWaitingForSceneChange: return "Waiting for scene change";
    case State::kFailedLocked: return "FAILED LOCKED: " + failure_reason_;
  }
  return "Unknown";
}

}  // namespace execution
