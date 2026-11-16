#pragma once

#include <cstdint>
#include <string>

#include "scene/scene_snapshot.h"

namespace execution {

class ExperimentManager {
 public:
  enum class State {
    kWaitingForData,
    kWaitingForStationary,
    kReadyToPlan,
    kPlanning,
    kTracking,
    kCompleted,
    kWaitingForSceneChange,
    kFailedLocked,
  };

  struct Configuration {
    double stationary_duration = 3.0;
    double vehicle_stationary_position_threshold = 0.002;
    double vehicle_stationary_yaw_threshold = 0.0087266463;
    double obstacle_stationary_boundary_threshold = 0.002;
    double frozen_vehicle_position_threshold = 0.003;
    double frozen_vehicle_yaw_threshold = 0.0087266463;
    double frozen_obstacle_boundary_threshold = 0.003;
    double rearm_distance = 0.10;
  };

  explicit ExperimentManager(Configuration configuration);
  void Observe(const scene::SceneSnapshot& snapshot, bool data_valid,
               std::uint64_t revision, double now);
  bool BeginPlanning(scene::SceneSnapshot* frozen, std::uint64_t* generation);
  void PlanningFinished(bool success, bool execute,
                        const std::string& failure_reason = "planning_failed");
  void Reached(const scene::Pose2d& stopped_pose);
  void TrackingFailed(const std::string& reason = "tracking_failed");
  void ForceSafeWait(const std::string& reason = "safety_protection");
  void AbortTask(const std::string& reason = "task_aborted");
  void InitializeTaskSequence(std::uint64_t last_task_id);

  static double ObstacleBoundaryDistance(
      const scene::SceneObstacle& lhs, const scene::SceneObstacle& rhs);

  State state() const { return state_; }
  bool frozen_scene_valid() const { return frozen_scene_valid_; }
  bool task_completed() const { return task_completed_; }
  const std::string& failure_reason() const { return failure_reason_; }
  std::uint64_t task_id() const { return task_id_; }
  std::string StateName() const;

 private:
  bool ObstaclesMoved(const scene::SceneSnapshot& lhs,
                      const scene::SceneSnapshot& rhs,
                      double boundary_threshold) const;
  void EnterFailedLocked(const std::string& reason);

  Configuration configuration_;
  State state_ = State::kWaitingForData;
  scene::SceneSnapshot latest_;
  scene::SceneSnapshot stable_reference_;
  scene::SceneSnapshot frozen_;
  scene::Pose2d stopped_pose_;
  bool have_latest_ = false;
  bool have_stopped_pose_ = false;
  bool task_completed_ = false;
  bool frozen_scene_valid_ = false;
  std::string failure_reason_;
  double stable_since_ = 0.0;
  std::uint64_t latest_revision_ = 0;
  std::uint64_t frozen_revision_ = 0;
  std::uint64_t task_id_ = 0;
};

}  // namespace execution
