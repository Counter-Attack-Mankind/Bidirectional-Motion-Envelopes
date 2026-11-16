#include "visualize/body_history_visualizer.h"

#include <cmath>
#include <utility>

#include "common/math/math_utils.h"
#include "visualize/visualization_palette.h"
#include "visualize/visualization_plot.h"

namespace visualization {

bool BodyHistoryVisualizer::Initialize(const VehicleParameter& vehicle,
                                       double spacing, std::string* error,
                                       std::string marker_namespace) {
  if (!std::isfinite(spacing) || spacing <= 0.0) {
    if (error) *error = "Body-history spacing must be finite and positive";
    return false;
  }
  vehicle_ = vehicle;
  spacing_ = spacing;
  marker_namespace_ = std::move(marker_namespace);
  const double front = vehicle_.wheel_base + vehicle_.front_hang;
  body_radius_ = std::hypot(std::max(front, vehicle_.rear_hang),
                            0.5 * vehicle_.width);
  return true;
}

void BodyHistoryVisualizer::Reset(const optimizer::TrajectoryNode& pose,
                                  bool forward) {
  Clear();
  previous_pose_ = pose;
  has_previous_pose_ = true;
  accumulated_motion_ = 0.0;
  next_sample_motion_ = spacing_;
  AddSample(pose, forward);
}

void BodyHistoryVisualizer::Update(const optimizer::TrajectoryNode& pose,
                                   bool forward) {
  if (!has_previous_pose_) {
    Reset(pose, forward);
    return;
  }
  const double angle = common::math::NormalizeAngle(
      pose.theta - previous_pose_.theta);
  const double segment_motion =
      std::hypot(pose.x - previous_pose_.x, pose.y - previous_pose_.y) +
      body_radius_ * std::abs(angle);
  const double segment_start = accumulated_motion_;
  const double segment_end = segment_start + segment_motion;
  if (segment_motion > 1e-12) {
    while (next_sample_motion_ <= segment_end + 1e-12) {
      const double ratio = std::min(
          1.0, std::max(0.0,
                        (next_sample_motion_ - segment_start) / segment_motion));
      optimizer::TrajectoryNode sample;
      sample.x = previous_pose_.x + ratio * (pose.x - previous_pose_.x);
      sample.y = previous_pose_.y + ratio * (pose.y - previous_pose_.y);
      sample.theta = common::math::NormalizeAngle(
          previous_pose_.theta + ratio * angle);
      AddSample(sample, forward);
      next_sample_motion_ += spacing_;
    }
  }
  accumulated_motion_ = segment_end;
  previous_pose_ = pose;
}

void BodyHistoryVisualizer::AddSample(
    const optimizer::TrajectoryNode& pose, bool forward) {
  (forward ? forward_poses_ : reverse_poses_).push_back(pose);
  dirty_ = true;
}

void BodyHistoryVisualizer::QueueMarkers() {
  if (!dirty_) return;
  auto queue = [&](const std::vector<optimizer::TrajectoryNode>& poses,
                   bool forward, int id) {
    std::vector<double> x, y, theta;
    x.reserve(poses.size()); y.reserve(poses.size()); theta.reserve(poses.size());
    for (const auto& pose : poses) {
      x.push_back(pose.x); y.push_back(pose.y); theta.push_back(pose.theta);
    }
    VisualizationPlot::PlotVehicleFootprintBatch(
        x, y, theta, vehicle_.wheel_base + vehicle_.front_hang +
                         vehicle_.rear_hang,
        vehicle_.width, vehicle_.rear_hang,
        palette::WithAlpha(forward ? palette::Forward() : palette::Reverse(),
                           palette::kHistoryOutlineAlpha),
        palette::kHistoryLineWidth, id, marker_namespace_);
  };
  queue(forward_poses_, true, 0);
  queue(reverse_poses_, false, 1);
  dirty_ = false;
}

void BodyHistoryVisualizer::Clear() {
  VisualizationPlot::ClearMarkers(marker_namespace_, 0, 1);
  forward_poses_.clear();
  reverse_poses_.clear();
  has_previous_pose_ = false;
  dirty_ = false;
}

std::size_t BodyHistoryVisualizer::sample_count() const {
  return forward_poses_.size() + reverse_poses_.size();
}

}  // namespace visualization
