#include "visualize/real_vehicle_visualizer.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/util/color.h"
#include "visualize/visualization_plot.h"

namespace visualization {

bool RealVehicleVisualizer::Initialize(const VehicleParameter& vehicle,
                                       double swept_resolution,
                                       std::string* error) {
  return swept_area_.Initialize(vehicle, swept_resolution, error,
                                "actual_swept_area");
}

void RealVehicleVisualizer::StartTask(
    const optimizer::Trajectory& reference) {
  Clear();
  reference_ = reference;
  active_ = !reference_.nodes.empty() && !reference_.intervals.empty();
}

void RealVehicleVisualizer::Update(const scene::Pose2d& pose) {
  if (!active_) return;
  optimizer::TrajectoryNode measured;
  measured.x = pose.x;
  measured.y = pose.y;
  measured.theta = pose.yaw;
  swept_area_.Update(measured);
  swept_area_.QueueDirtyMarkers();
  trail_x_.push_back(pose.x);
  trail_y_.push_back(pose.y);
  VisualizationPlot::Plot(trail_x_, trail_y_, 0.012,
                          Color::fromRGB(1.0f, 0.55f, 0.0f), 0,
                          "actual_vehicle_trail");

  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < reference_.nodes.size(); ++i) {
    const double distance = std::hypot(reference_.nodes[i].x - pose.x,
                                       reference_.nodes[i].y - pose.y);
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }
  nearest = std::min(nearest, reference_.intervals.size() - 1);
  footprint_.Update(reference_.intervals[nearest], nearest);
  VisualizationPlot::Trigger();
}

void RealVehicleVisualizer::Clear() {
  footprint_.Clear();
  swept_area_.Clear();
  VisualizationPlot::ClearMarkers("actual_vehicle_trail", 0, 0);
  reference_ = optimizer::Trajectory();
  trail_x_.clear();
  trail_y_.clear();
  active_ = false;
  VisualizationPlot::Trigger();
}

}  // namespace visualization
