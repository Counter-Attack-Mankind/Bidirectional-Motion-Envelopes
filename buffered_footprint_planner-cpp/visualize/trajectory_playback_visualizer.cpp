#include "visualize/trajectory_playback_visualizer.h"

#include <cmath>

namespace visualization {

bool TrajectoryPlaybackVisualizer::Initialize(
    ros::NodeHandle& node, SceneVisualizer* scene_visualizer,
    const VehicleParameter& vehicle, double playback_speed,
    double playback_frequency_hz, double swept_area_resolution,
    std::string* error) {
  if (!scene_visualizer) {
    if (error) *error = "Scene visualizer is null";
    return false;
  }
  if (!std::isfinite(playback_speed) || playback_speed <= 0.0) {
    if (error) *error = "Playback speed must be finite and positive";
    return false;
  }
  if (!std::isfinite(playback_frequency_hz) ||
      playback_frequency_hz <= 0.0) {
    if (error) *error = "Playback frequency must be finite and positive";
    return false;
  }
  if (!swept_area_visualizer_.Initialize(
          vehicle, swept_area_resolution, error)) {
    return false;
  }
  node_ = &node;
  scene_visualizer_ = scene_visualizer;
  vehicle_ = vehicle;
  playback_speed_ = playback_speed;
  playback_frequency_hz_ = playback_frequency_hz;
  swept_area_resolution_ = swept_area_resolution;
  initialized_ = true;
  return true;
}

bool TrajectoryPlaybackVisualizer::Start(
    const optimizer::Trajectory& trajectory, std::string* error) {
  if (!initialized_) {
    if (error) *error = "Trajectory playback visualizer is not initialized";
    return false;
  }
  StopAndClear();
  trajectory_ = trajectory;
  completed_cycles_ = 0;
  has_previous_state_ = false;
  if (!player_.Start(
          *node_, trajectory_, playback_speed_, playback_frequency_hz_,
          [this](const PlaybackState& state) { OnPlaybackState(state); },
          error)) {
    trajectory_ = optimizer::Trajectory();
    return false;
  }
  scene_visualizer_->QueueOptimizedTrajectory(trajectory_);
  scene_visualizer_->QueueOptimizedConfigurationPoints(trajectory_);
  scene_visualizer_->Trigger();
  ROS_INFO("Trajectory playback started at %.3fx and %.1f Hz, swept "
           "resolution %.4f m", playback_speed_, playback_frequency_hz_,
           swept_area_resolution_);
  return true;
}

void TrajectoryPlaybackVisualizer::StopAndClear() {
  player_.Stop();
  footprint_visualizer_.Clear();
  swept_area_visualizer_.Clear();
  if (scene_visualizer_) {
    scene_visualizer_->ClearTrajectoryPlayback();
    scene_visualizer_->Trigger();
  }
  trajectory_ = optimizer::Trajectory();
  has_previous_state_ = false;
}

void TrajectoryPlaybackVisualizer::UpdateSweptArea(
    const PlaybackState& state) {
  if (!has_previous_state_) {
    swept_area_visualizer_.Reset(state.pose);
    previous_state_ = state;
    has_previous_state_ = true;
    return;
  }
  if (state.new_cycle ||
      state.trajectory_time + 1e-12 < previous_state_.trajectory_time) {
    swept_area_visualizer_.Reset(state.pose);
    previous_state_ = state;
    return;
  }

  swept_area_visualizer_.UpdateAlongTrajectory(
      trajectory_, previous_state_.interval_index, state.interval_index,
      state.pose);
  previous_state_ = state;
}

void TrajectoryPlaybackVisualizer::OnPlaybackState(
    const PlaybackState& state) {
  if (!player_.active() || state.interval_index >= trajectory_.intervals.size()) {
    return;
  }
  if (state.new_cycle) {
    ROS_INFO("Playback cycle %zu completed: swept cells=%zu, adaptive body "
             "samples=%zu", ++completed_cycles_,
             swept_area_visualizer_.cell_count(),
             swept_area_visualizer_.raster_sample_count());
    footprint_visualizer_.Clear();
  }
  UpdateSweptArea(state);
  footprint_visualizer_.Update(
      trajectory_.intervals[state.interval_index], state.interval_index,
      state.new_cycle);
  scene_visualizer_->QueuePlaybackVehicle(state);
  swept_area_visualizer_.QueueDirtyMarkers();
  scene_visualizer_->Trigger();
}

}  // namespace visualization
