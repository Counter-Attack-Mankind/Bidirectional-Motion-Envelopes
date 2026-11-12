#pragma once

#include <cstddef>
#include <string>

#include <ros/ros.h>

#include "hybrid_astar/vehicle_parameter.h"
#include "optimizer/trajectory.h"
#include "visualize/footprint_visualizer.h"
#include "visualize/scene_visualizer.h"
#include "visualize/swept_area_visualizer.h"
#include "visualize/trajectory_player.h"

namespace visualization {

// Coordinates the reusable optimized-trajectory animation modules. Planning
// and scene lifecycle remain owned by the launch-mode node.
class TrajectoryPlaybackVisualizer {
 public:
  bool Initialize(ros::NodeHandle& node, SceneVisualizer* scene_visualizer,
                  const VehicleParameter& vehicle, double playback_speed,
                  double playback_frequency_hz,
                  double swept_area_resolution, std::string* error);
  bool Start(const optimizer::Trajectory& trajectory, std::string* error);
  void StopAndClear();
  bool active() const { return player_.active(); }

 private:
  void OnPlaybackState(const PlaybackState& state);
  void UpdateSweptArea(const PlaybackState& state);

  ros::NodeHandle* node_ = nullptr;
  SceneVisualizer* scene_visualizer_ = nullptr;
  VehicleParameter vehicle_;
  optimizer::Trajectory trajectory_;
  TrajectoryPlayer player_;
  FootprintVisualizer footprint_visualizer_;
  SweptAreaVisualizer swept_area_visualizer_;
  double playback_speed_ = 1.0;
  double playback_frequency_hz_ = 100.0;
  double swept_area_resolution_ = 0.005;
  std::size_t completed_cycles_ = 0;
  PlaybackState previous_state_;
  bool has_previous_state_ = false;
  bool initialized_ = false;
};

}  // namespace visualization
