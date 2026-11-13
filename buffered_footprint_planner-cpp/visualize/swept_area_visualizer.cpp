#include "visualize/swept_area_visualizer.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/math/math_utils.h"
#include "common/math/pose.h"
#include "common/util/color.h"
#include "visualize/visualization_plot.h"

namespace visualization {

bool SweptAreaVisualizer::Initialize(const VehicleParameter& vehicle,
                                     double resolution,
                                     std::string* error,
                                     std::string marker_namespace) {
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    if (error) *error = "Swept-area resolution must be finite and positive";
    return false;
  }
  vehicle_ = vehicle;
  marker_namespace_ = std::move(marker_namespace);
  resolution_ = resolution;
  const double front = vehicle_.wheel_base + vehicle_.front_hang;
  vehicle_radius_ = std::hypot(std::max(front, vehicle_.rear_hang),
                               vehicle_.width * 0.5);
  return true;
}

void SweptAreaVisualizer::Update(
    const optimizer::TrajectoryNode& pose) {
  if (!has_previous_pose_) {
    RasterizeVehicle(pose);
    previous_pose_ = pose;
    has_previous_pose_ = true;
    return;
  }
  AddInterpolatedSweep(previous_pose_, pose);
  previous_pose_ = pose;
}

void SweptAreaVisualizer::UpdateAlongTrajectory(
    const optimizer::Trajectory& trajectory,
    std::size_t previous_interval, std::size_t current_interval,
    const optimizer::TrajectoryNode& current_pose) {
  if (trajectory.nodes.empty()) return;
  const std::size_t first_boundary = previous_interval + 1;
  const std::size_t last_boundary = std::min(
      current_interval, trajectory.nodes.size() - 1);
  for (std::size_t node_index = first_boundary;
       node_index <= last_boundary; ++node_index) {
    Update(trajectory.nodes[node_index]);
  }
  Update(current_pose);
}

void SweptAreaVisualizer::Reset(
    const optimizer::TrajectoryNode& start_pose) {
  Clear();
  Update(start_pose);
}

void SweptAreaVisualizer::Clear() {
  if (published_chunk_count_ > 0) {
    VisualizationPlot::ClearMarkers(
        marker_namespace_, 0,
        static_cast<int>(published_chunk_count_ - 1));
  }
  occupied_cells_.clear();
  marker_chunks_.clear();
  first_dirty_chunk_ = std::numeric_limits<std::size_t>::max();
  published_chunk_count_ = 0;
  raster_sample_count_ = 0;
  has_previous_pose_ = false;
}

void SweptAreaVisualizer::AddInterpolatedSweep(
    const optimizer::TrajectoryNode& first,
    const optimizer::TrajectoryNode& second) {
  const double translation = std::hypot(second.x - first.x,
                                        second.y - first.y);
  const double angle = common::math::NormalizeAngle(
      second.theta - first.theta);
  const double maximum_point_motion =
      translation + vehicle_radius_ * std::abs(angle);
  const double sample_spacing = resolution_ * 0.5;
  const std::size_t sample_count = std::max<std::size_t>(
      1, static_cast<std::size_t>(
          std::ceil(maximum_point_motion / sample_spacing)));
  for (std::size_t sample = 1; sample <= sample_count; ++sample) {
    const double ratio = static_cast<double>(sample) /
                         static_cast<double>(sample_count);
    optimizer::TrajectoryNode pose;
    pose.x = first.x + ratio * (second.x - first.x);
    pose.y = first.y + ratio * (second.y - first.y);
    pose.theta = common::math::NormalizeAngle(first.theta + ratio * angle);
    RasterizeVehicle(pose);
  }
}

void SweptAreaVisualizer::RasterizeVehicle(
    const optimizer::TrajectoryNode& pose) {
  ++raster_sample_count_;
  const auto box = vehicle_.GenerateBox(
      common::math::Pose(pose.x, pose.y, pose.theta));
  const std::int32_t min_x = static_cast<std::int32_t>(
      std::floor(box.min_x() / resolution_));
  const std::int32_t max_x = static_cast<std::int32_t>(
      std::floor(box.max_x() / resolution_));
  const std::int32_t min_y = static_cast<std::int32_t>(
      std::floor(box.min_y() / resolution_));
  const std::int32_t max_y = static_cast<std::int32_t>(
      std::floor(box.max_y() / resolution_));
  for (std::int32_t x_index = min_x; x_index <= max_x; ++x_index) {
    for (std::int32_t y_index = min_y; y_index <= max_y; ++y_index) {
      const common::math::Vec2d center(
          (static_cast<double>(x_index) + 0.5) * resolution_,
          (static_cast<double>(y_index) + 0.5) * resolution_);
      if (box.IsPointIn(center)) AddCell(x_index, y_index);
    }
  }
}

void SweptAreaVisualizer::AddCell(std::int32_t x_index,
                                  std::int32_t y_index) {
  if (!occupied_cells_.insert(CellKey(x_index, y_index)).second) return;
  const std::size_t chunk_index =
      (occupied_cells_.size() - 1) / kCellsPerMarker;
  if (chunk_index >= marker_chunks_.size()) marker_chunks_.emplace_back();
  marker_chunks_[chunk_index].emplace_back(
      (static_cast<double>(x_index) + 0.5) * resolution_,
      (static_cast<double>(y_index) + 0.5) * resolution_);
  first_dirty_chunk_ = std::min(first_dirty_chunk_, chunk_index);
}

std::uint64_t SweptAreaVisualizer::CellKey(std::int32_t x_index,
                                           std::int32_t y_index) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x_index))
          << 32U) |
         static_cast<std::uint32_t>(y_index);
}

void SweptAreaVisualizer::QueueDirtyMarkers() {
  if (first_dirty_chunk_ == std::numeric_limits<std::size_t>::max()) return;
  Color fill = Color::fromRGB(0.35f, 0.78f, 1.0f);
  fill.set_a(0.30);
  for (std::size_t index = first_dirty_chunk_;
       index < marker_chunks_.size(); ++index) {
    std::vector<double> x;
    std::vector<double> y;
    x.reserve(marker_chunks_[index].size());
    y.reserve(marker_chunks_[index].size());
    for (const auto& cell : marker_chunks_[index]) {
      x.push_back(cell.x());
      y.push_back(cell.y());
    }
    VisualizationPlot::PlotGridCells(
        x, y, resolution_, fill, static_cast<int>(index),
        marker_namespace_, 0.008, 0.01);
  }
  published_chunk_count_ = marker_chunks_.size();
  first_dirty_chunk_ = std::numeric_limits<std::size_t>::max();
}

}  // namespace visualization
