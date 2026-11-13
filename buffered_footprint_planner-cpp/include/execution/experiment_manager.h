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
  };

  struct Configuration {
    double stationary_duration = 3.0;
    double position_threshold = 0.002;
    double yaw_threshold = 0.0087266463;
    double scene_position_threshold = 0.015;
    double scene_yaw_threshold = 0.034906585;
    double rearm_distance = 0.10;
  };

  explicit ExperimentManager(Configuration configuration);
  void Observe(const scene::SceneSnapshot& snapshot, bool data_valid,
               std::uint64_t revision, double now);
  bool BeginPlanning(scene::SceneSnapshot* frozen, std::uint64_t* generation);
  void PlanningFinished(bool success, bool execute);
  void Reached(const scene::Pose2d& stopped_pose);
  void ForceSafeWait();

  State state() const { return state_; }
  bool frozen_scene_valid() const { return frozen_scene_valid_; }
  std::uint64_t task_id() const { return task_id_; }
  std::string StateName() const;

 private:
  bool SceneMoved(const scene::SceneSnapshot& lhs,
                  const scene::SceneSnapshot& rhs) const;

  Configuration configuration_;
  State state_ = State::kWaitingForData;
  scene::SceneSnapshot latest_;
  scene::SceneSnapshot stable_reference_;
  scene::SceneSnapshot frozen_;
  scene::Pose2d stopped_pose_;
  bool have_latest_ = false;
  bool have_stopped_pose_ = false;
  bool frozen_scene_valid_ = false;
  double stable_since_ = 0.0;
  std::uint64_t latest_revision_ = 0;
  std::uint64_t frozen_revision_ = 0;
  std::uint64_t task_id_ = 0;
};

}  // namespace execution
