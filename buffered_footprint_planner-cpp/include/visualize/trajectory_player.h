#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "optimizer/trajectory.h"

namespace visualization {

struct PlaybackState {
  optimizer::TrajectoryNode pose;
  double trajectory_time = 0.0;
  double total_time = 0.0;
  std::size_t interval_index = 0;
  std::size_t interval_count = 0;
  bool gear = true;
  bool paused_at_end = false;
  bool new_cycle = false;
};

class TrajectoryPlayer {
 public:
  using StateCallback = std::function<void(const PlaybackState&)>;

  bool Start(ros::NodeHandle& node, const optimizer::Trajectory& trajectory,
             double playback_speed, StateCallback callback,
             std::string* error);
  void Stop();
  bool active() const { return active_; }

 private:
  bool ValidateAndBuildTimeline(std::string* error);
  PlaybackState Sample(bool new_cycle) const;
  void OnTimer(const ros::TimerEvent& event);

  optimizer::Trajectory trajectory_;
  std::vector<double> cumulative_time_;
  StateCallback callback_;
  ros::Timer timer_;
  ros::Time last_tick_;
  double playback_speed_ = 1.0;
  double trajectory_time_ = 0.0;
  double pause_elapsed_ = 0.0;
  bool paused_at_end_ = false;
  bool active_ = false;
};

}  // namespace visualization
