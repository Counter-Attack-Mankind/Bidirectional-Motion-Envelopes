#pragma once

#include <string>
#include <vector>

#include "optimizer/trajectory.h"
#include "planning/hybrid_astar/vehicle_parameter.h"

namespace visualization {

// Visualization-only resampling using accumulated corner motion
// ||delta p|| + r_B |delta theta|. It does not alter optimizer nodes or BME.
class BodyHistoryVisualizer {
 public:
  bool Initialize(const VehicleParameter& vehicle, double spacing,
                  std::string* error,
                  std::string marker_namespace = "trajectory_body_history");
  void Reset(const optimizer::TrajectoryNode& pose, bool forward);
  void Update(const optimizer::TrajectoryNode& pose, bool forward);
  void QueueMarkers();
  void Clear();
  std::size_t sample_count() const;

 private:
  void AddSample(const optimizer::TrajectoryNode& pose, bool forward);

  VehicleParameter vehicle_;
  double spacing_ = 0.005;
  double body_radius_ = 0.0;
  double accumulated_motion_ = 0.0;
  double next_sample_motion_ = 0.005;
  optimizer::TrajectoryNode previous_pose_;
  bool has_previous_pose_ = false;
  bool dirty_ = false;
  std::vector<optimizer::TrajectoryNode> forward_poses_;
  std::vector<optimizer::TrajectoryNode> reverse_poses_;
  std::string marker_namespace_;
};

}  // namespace visualization
