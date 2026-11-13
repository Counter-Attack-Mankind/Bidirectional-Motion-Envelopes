#pragma once

#include <string>
#include <vector>

#include "planning/hybrid_astar/vehicle_parameter.h"
#include "optimizer/trajectory.h"
#include "scene/scene_snapshot.h"
#include "visualize/footprint_visualizer.h"
#include "visualize/swept_area_visualizer.h"

namespace visualization {

// Displays measured motion separately from the planned reference.  The BME
// envelope is the optimized interval closest to the current measured pose.
class RealVehicleVisualizer {
 public:
  RealVehicleVisualizer() : footprint_("actual_bme") {}
  bool Initialize(const VehicleParameter& vehicle, double swept_resolution,
                  std::string* error);
  void StartTask(const optimizer::Trajectory& reference);
  void Update(const scene::Pose2d& pose);
  void Clear();

 private:
  optimizer::Trajectory reference_;
  FootprintVisualizer footprint_;
  SweptAreaVisualizer swept_area_;
  std::vector<double> trail_x_;
  std::vector<double> trail_y_;
  bool active_ = false;
};

}  // namespace visualization
