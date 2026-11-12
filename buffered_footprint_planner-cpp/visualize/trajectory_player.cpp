#include "visualize/trajectory_player.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/math/math_utils.h"

namespace visualization {
namespace {

bool IsFiniteNode(const optimizer::TrajectoryNode& node) {
  return std::isfinite(node.x) && std::isfinite(node.y) &&
         std::isfinite(node.theta) && std::isfinite(node.v) &&
         std::isfinite(node.a) && std::isfinite(node.phy) &&
         std::isfinite(node.w);
}

double Interpolate(double first, double second, double ratio) {
  return first + ratio * (second - first);
}

}  // namespace

bool TrajectoryPlayer::Start(ros::NodeHandle& node,
                             const optimizer::Trajectory& trajectory,
                             double playback_speed,
                             double playback_frequency_hz,
                             StateCallback callback,
                             std::string* error) {
  Stop();
  if (!std::isfinite(playback_speed) || playback_speed <= 0.0) {
    if (error) *error = "Playback speed must be finite and positive";
    return false;
  }
  if (!std::isfinite(playback_frequency_hz) ||
      playback_frequency_hz <= 0.0) {
    if (error) *error = "Playback frequency must be finite and positive";
    return false;
  }
  if (!callback) {
    if (error) *error = "Trajectory playback callback is empty";
    return false;
  }
  trajectory_ = trajectory;
  if (!ValidateAndBuildTimeline(error)) return false;
  playback_speed_ = playback_speed;
  playback_frequency_hz_ = playback_frequency_hz;
  callback_ = std::move(callback);
  trajectory_time_ = 0.0;
  pause_elapsed_ = 0.0;
  paused_at_end_ = false;
  last_tick_ = ros::Time();
  active_ = true;
  timer_ = node.createTimer(ros::Duration(1.0 / playback_frequency_hz_),
                            &TrajectoryPlayer::OnTimer, this);
  return true;
}

void TrajectoryPlayer::Stop() {
  timer_.stop();
  active_ = false;
  callback_ = StateCallback();
  cumulative_time_.clear();
}

bool TrajectoryPlayer::ValidateAndBuildTimeline(std::string* error) {
  if (trajectory_.nodes.size() < 2 ||
      trajectory_.intervals.size() + 1 != trajectory_.nodes.size()) {
    if (error) {
      *error = "Optimized trajectory must have N nodes and N-1 intervals";
    }
    return false;
  }
  for (const auto& node : trajectory_.nodes) {
    if (!IsFiniteNode(node)) {
      if (error) *error = "Optimized trajectory contains a non-finite node";
      return false;
    }
  }
  cumulative_time_.assign(trajectory_.nodes.size(), 0.0);
  for (std::size_t i = 0; i < trajectory_.intervals.size(); ++i) {
    const auto& interval = trajectory_.intervals[i];
    if (!std::isfinite(interval.dt) || interval.dt <= 0.0) {
      if (error) *error = "Optimized trajectory contains an invalid dt";
      return false;
    }
    const double scales[] = {interval.up, interval.down,
                             interval.left, interval.right};
    for (double scale : scales) {
      if (!std::isfinite(scale)) {
        if (error) *error = "Optimized footprint contains a non-finite scale";
        return false;
      }
    }
    for (const auto& corner : interval.corners) {
      if (!std::isfinite(corner.x()) || !std::isfinite(corner.y())) {
        if (error) *error = "Optimized footprint contains a non-finite corner";
        return false;
      }
    }
    cumulative_time_[i + 1] = cumulative_time_[i] + interval.dt;
    if (!std::isfinite(cumulative_time_[i + 1])) {
      if (error) *error = "Optimized trajectory duration is not finite";
      return false;
    }
  }
  if (cumulative_time_.back() <= 0.0) {
    if (error) *error = "Optimized trajectory duration must be positive";
    return false;
  }
  return true;
}

PlaybackState TrajectoryPlayer::Sample(bool new_cycle) const {
  PlaybackState state;
  state.trajectory_time = trajectory_time_;
  state.total_time = cumulative_time_.back();
  state.interval_count = trajectory_.intervals.size();
  state.paused_at_end = paused_at_end_;
  state.new_cycle = new_cycle;

  auto upper = std::upper_bound(cumulative_time_.begin(),
                                cumulative_time_.end(), trajectory_time_);
  std::size_t interval = upper == cumulative_time_.begin()
      ? 0
      : static_cast<std::size_t>(upper - cumulative_time_.begin() - 1);
  interval = std::min(interval, trajectory_.intervals.size() - 1);
  state.interval_index = interval;
  state.gear = trajectory_.intervals[interval].gear;

  const double begin = cumulative_time_[interval];
  const double dt = trajectory_.intervals[interval].dt;
  const double ratio = std::clamp((trajectory_time_ - begin) / dt, 0.0, 1.0);
  const auto& first = trajectory_.nodes[interval];
  const auto& second = trajectory_.nodes[interval + 1];
  state.pose.x = Interpolate(first.x, second.x, ratio);
  state.pose.y = Interpolate(first.y, second.y, ratio);
  state.pose.theta = common::math::NormalizeAngle(
      first.theta + ratio * common::math::NormalizeAngle(
          second.theta - first.theta));
  state.pose.v = Interpolate(first.v, second.v, ratio);
  state.pose.a = Interpolate(first.a, second.a, ratio);
  state.pose.phy = Interpolate(first.phy, second.phy, ratio);
  state.pose.w = Interpolate(first.w, second.w, ratio);
  return state;
}

void TrajectoryPlayer::OnTimer(const ros::TimerEvent& event) {
  if (!active_ || !callback_) return;
  bool new_cycle = false;
  if (last_tick_.isZero()) {
    last_tick_ = event.current_real;
  } else {
    const double wall_dt = std::max(
        0.0, (event.current_real - last_tick_).toSec());
    last_tick_ = event.current_real;
    if (paused_at_end_) {
      pause_elapsed_ += wall_dt;
      if (pause_elapsed_ >= 3.0) {
        trajectory_time_ = 0.0;
        pause_elapsed_ = 0.0;
        paused_at_end_ = false;
        new_cycle = true;
      }
    } else {
      trajectory_time_ += wall_dt * playback_speed_;
      if (trajectory_time_ >= cumulative_time_.back()) {
        trajectory_time_ = cumulative_time_.back();
        paused_at_end_ = true;
        pause_elapsed_ = 0.0;
      }
    }
  }
  callback_(Sample(new_cycle));
}

}  // namespace visualization
