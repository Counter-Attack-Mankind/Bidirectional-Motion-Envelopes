#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include "common/math/vec2d.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "optimizer/trajectory.h"

namespace visualization {

class SweptAreaVisualizer {
 public:
  bool Initialize(const VehicleParameter& vehicle, double resolution,
                  std::string* error);
  void Update(const optimizer::TrajectoryNode& pose);
  void Reset(const optimizer::TrajectoryNode& start_pose);
  void QueueDirtyMarkers();

  std::size_t cell_count() const { return occupied_cells_.size(); }
  std::size_t raster_sample_count() const { return raster_sample_count_; }

 private:
  void AddInterpolatedSweep(const optimizer::TrajectoryNode& first,
                            const optimizer::TrajectoryNode& second);
  void RasterizeVehicle(const optimizer::TrajectoryNode& pose);
  void AddCell(std::int32_t x_index, std::int32_t y_index);
  static std::uint64_t CellKey(std::int32_t x_index,
                               std::int32_t y_index);

  static constexpr std::size_t kCellsPerMarker = 4096;
  VehicleParameter vehicle_;
  double resolution_ = 0.005;
  double vehicle_radius_ = 0.0;
  optimizer::TrajectoryNode previous_pose_;
  bool has_previous_pose_ = false;
  std::unordered_set<std::uint64_t> occupied_cells_;
  std::vector<std::vector<common::math::Vec2d>> marker_chunks_;
  std::size_t first_dirty_chunk_ = std::numeric_limits<std::size_t>::max();
  std::size_t published_chunk_count_ = 0;
  std::size_t raster_sample_count_ = 0;
};

}  // namespace visualization
