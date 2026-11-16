#include "visualize/swept_area_visualizer.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/math/math_utils.h"
#include "common/math/pose.h"
#include "common/util/color.h"
#include "visualize/visualization_palette.h"
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
    const optimizer::TrajectoryNode& pose, bool forward) {
  if (!has_previous_pose_) {
    RasterizeVehicle(pose, forward);
    previous_pose_ = pose;
    has_previous_pose_ = true;
    return;
  }
  AddInterpolatedSweep(previous_pose_, pose, forward);
  previous_pose_ = pose;
}

void SweptAreaVisualizer::UpdateAlongTrajectory(
    const optimizer::Trajectory& trajectory,
    std::size_t previous_interval, std::size_t current_interval,
    const optimizer::TrajectoryNode& current_pose) {
  if (trajectory.nodes.empty() || trajectory.intervals.empty()) return;
  const std::size_t first_boundary = previous_interval + 1;
  const std::size_t last_boundary = std::min(
      current_interval, trajectory.nodes.size() - 1);
  for (std::size_t node_index = first_boundary;
       node_index <= last_boundary; ++node_index) {
    const std::size_t interval_index = std::min(
        node_index - 1, trajectory.intervals.size() - 1);
    Update(trajectory.nodes[node_index],
           trajectory.intervals[interval_index].gear);
  }
  const std::size_t interval_index = std::min(
      current_interval, trajectory.intervals.size() - 1);
  Update(current_pose, trajectory.intervals[interval_index].gear);
}

void SweptAreaVisualizer::Reset(
    const optimizer::TrajectoryNode& start_pose, bool forward) {
  Clear();
  Update(start_pose, forward);
}

void SweptAreaVisualizer::Clear() {
  auto clear_layer = [&](DirectionLayer* layer, const std::string& suffix) {
    if (layer->published_chunk_count > 0) {
      VisualizationPlot::ClearMarkers(
          marker_namespace_ + suffix, 0,
          static_cast<int>(layer->published_chunk_count - 1));
    }
    layer->occupied_cells.clear();
    layer->marker_chunks.clear();
    layer->first_dirty_chunk = std::numeric_limits<std::size_t>::max();
    layer->published_chunk_count = 0;
  };
  clear_layer(&forward_layer_, "_forward");
  clear_layer(&reverse_layer_, "_reverse");
  raster_sample_count_ = 0;
  has_previous_pose_ = false;
}

void SweptAreaVisualizer::AddInterpolatedSweep(
    const optimizer::TrajectoryNode& first,
    const optimizer::TrajectoryNode& second, bool forward) {
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
    RasterizeVehicle(pose, forward);
  }
}

void SweptAreaVisualizer::RasterizeVehicle(
    const optimizer::TrajectoryNode& pose, bool forward) {
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
      if (box.IsPointIn(center)) AddCell(x_index, y_index, forward);
    }
  }
}

void SweptAreaVisualizer::AddCell(std::int32_t x_index,
                                  std::int32_t y_index, bool forward) {
  DirectionLayer& layer = forward ? forward_layer_ : reverse_layer_;
  if (!layer.occupied_cells.insert(CellKey(x_index, y_index)).second) return;
  const std::size_t chunk_index =
      (layer.occupied_cells.size() - 1) / kCellsPerMarker;
  if (chunk_index >= layer.marker_chunks.size()) layer.marker_chunks.emplace_back();
  layer.marker_chunks[chunk_index].emplace_back(
      (static_cast<double>(x_index) + 0.5) * resolution_,
      (static_cast<double>(y_index) + 0.5) * resolution_);
  layer.first_dirty_chunk = std::min(layer.first_dirty_chunk, chunk_index);
}

std::uint64_t SweptAreaVisualizer::CellKey(std::int32_t x_index,
                                           std::int32_t y_index) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x_index))
          << 32U) |
         static_cast<std::uint32_t>(y_index);
}

void SweptAreaVisualizer::QueueDirtyMarkers() {
  auto queue_layer = [&](DirectionLayer* layer, bool forward,
                         const std::string& suffix) {
    if (layer->first_dirty_chunk ==
        std::numeric_limits<std::size_t>::max()) return;
    const Color fill = palette::WithAlpha(
        forward ? palette::Forward() : palette::Reverse(),
        palette::kSweptFillAlpha);
    for (std::size_t index = layer->first_dirty_chunk;
         index < layer->marker_chunks.size(); ++index) {
      std::vector<double> x;
      std::vector<double> y;
      x.reserve(layer->marker_chunks[index].size());
      y.reserve(layer->marker_chunks[index].size());
      for (const auto& cell : layer->marker_chunks[index]) {
        x.push_back(cell.x());
        y.push_back(cell.y());
      }
      VisualizationPlot::PlotGridCells(
          x, y, resolution_, fill, static_cast<int>(index),
          marker_namespace_ + suffix, 0.008, 0.01);
    }
    layer->published_chunk_count = layer->marker_chunks.size();
    layer->first_dirty_chunk = std::numeric_limits<std::size_t>::max();
  };
  queue_layer(&forward_layer_, true, "_forward");
  queue_layer(&reverse_layer_, false, "_reverse");
}

}  // namespace visualization
