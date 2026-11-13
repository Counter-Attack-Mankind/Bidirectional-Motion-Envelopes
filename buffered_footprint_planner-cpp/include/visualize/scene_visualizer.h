#pragma once

#include <cstddef>
#include <array>
#include <string>

#include <ros/ros.h>

#include "common/util/color.h"
#include "hybrid_astar/hybrid_a_star.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "optimizer/trajectory.h"
#include "scene/scene_snapshot.h"
#include "visualize/trajectory_player.h"

namespace visualization {

using common::util::Color;

// Queues the common scene and planning markers used by live simulation,
// scene creation, and offline experiments.
class SceneVisualizer {
 public:
  void Initialize(ros::NodeHandle& node, const std::string& frame_id,
                  const VehicleParameter& vehicle,
                  const std::string& marker_topic =
                      "/buffered_footprint_planner/markers");

  void QueueScene(const scene::SceneSnapshot& snapshot) const;
  void QueueMap(const scene::SceneSnapshot& snapshot) const;
  void QueueObstacle(const scene::SceneObstacle& obstacle,
                     std::size_t index) const;
  void QueueStart(const scene::Pose2d& start, bool locked = false) const;
  void QueueVehicle(const scene::Pose2d& pose,
                    const std::string& label) const;
  void QueueGoal(const scene::Pose2d& goal) const;
  void QueuePlanningStage(const std::array<double, 4>& bounds,
                          const std::string& text) const;
  void ClearPlanningStage() const;
  void QueueHybridSearch(
      const planning::HybridSearchProgress& progress) const;
  void QueueHybridPath(const planning::HybridAStartResult& path) const;
  void QueueOptimizedTrajectory(const optimizer::Trajectory& trajectory,
                                bool clear_existing = true) const;
  void QueueOptimizedConfigurationPoints(
      const optimizer::Trajectory& trajectory) const;
  void QueuePlaybackVehicle(const PlaybackState& state) const;

  void ClearPlanningResults() const;
  void ClearHybridSearch() const;
  void ClearTrajectoryPlayback() const;
  void Trigger() const;

 private:
  void QueuePose(const scene::Pose2d& pose, const std::string& role,
                 const std::string& label, const Color& color) const;

  VehicleParameter vehicle_;
};

}  // namespace visualization
