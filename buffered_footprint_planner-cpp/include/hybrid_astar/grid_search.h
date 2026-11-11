/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>

#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>
#include <array>
#include <deque>

#include "common/util/string_utils.h"
#include "common/math/circle_2d.h"
#include "common/math/line_segment2d.h"
#include "hybrid_astar/parking_planner_config.h"

namespace planning {

using common::math::Circle2d;

class Node2d {
public:
  Node2d(double x, double y, double xy_resolution,
         const std::array<double, 4>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    x_ = x;
    y_ = y;
    grid_x_ = ToGridCoordinate(x, XYbounds[0], xy_resolution);
    grid_y_ = ToGridCoordinate(y, XYbounds[2], xy_resolution);
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }
  void SetPathCost(double path_cost) {
    path_cost_ = path_cost;
    cost_ = path_cost_ + heuristic_;
  }
  void SetHeuristic(double heuristic) {
    heuristic_ = heuristic;
    cost_ = path_cost_ + heuristic_;
  }
  void SetCost(double cost) { cost_ = cost; }
  void SetPreNode(std::shared_ptr<Node2d> pre_node) { pre_node_ = pre_node; }
  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetGridX() const { return grid_x_; }
  double GetGridY() const { return grid_y_; }
  double GetPathCost() const { return path_cost_; }
  double GetHeuCost() const { return heuristic_; }
  double GetCost() const { return cost_; }
  const int64_t& GetIndex() const { return index_; }
  std::shared_ptr<Node2d> GetPreNode() const { return pre_node_; }
  static int64_t CalcIndex(double x, double y,
                               double xy_resolution,
                               const std::array<double, 4>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    const int grid_x = ToGridCoordinate(x, XYbounds[0], xy_resolution);
    const int grid_y = ToGridCoordinate(y, XYbounds[2], xy_resolution);
    return ComputeStringIndex(grid_x, grid_y);
  }
  bool operator==(const Node2d& right) const {
    return right.GetIndex() == index_;
  }

private:
  static int ToGridCoordinate(double value, double lower_bound,
                              double resolution) {
    return static_cast<int>(
        std::floor((value - lower_bound) / resolution + 1e-9));
  }

  static int64_t ComputeStringIndex(int x_grid, int y_grid) {
    const auto x_bits = static_cast<std::uint32_t>(x_grid);
    const auto y_bits = static_cast<std::uint32_t>(y_grid);
    return static_cast<std::int64_t>(
        (static_cast<std::uint64_t>(x_bits) << 32U) | y_bits);
  }

private:
  double x_ = 0.0;
  double y_ = 0.0;
  int grid_x_ = 0;
  int grid_y_ = 0;
  double path_cost_ = 0.0;
  double heuristic_ = 0.0;
  double cost_ = 0.0;
  int64_t index_;
  std::shared_ptr<Node2d> pre_node_ = nullptr;
};

struct GridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  double path_cost = 0.0;
};

struct GridSearchStatistics {
  std::size_t total_grid_cells = 0;
  std::size_t settled_cells = 0;
  std::size_t enqueue_count = 0;
  std::size_t queue_size = 0;
  std::size_t queue_peak = 0;
  std::size_t obstacle_checks = 0;
  double elapsed_seconds = 0.0;
};

struct GridSearchProgress {
  std::vector<double> settled_x;
  std::vector<double> settled_y;
  std::vector<double> wavefront_x;
  std::vector<double> wavefront_y;
  double resolution = 0.0;
  GridSearchStatistics statistics;
};

class GridSearch {
public:
  using ProgressCallback = std::function<void(GridSearchProgress)>;

  explicit GridSearch(const ParkingPlannerConfig &config, const std::vector<common::math::Polygon2d> &obstacles);
  virtual ~GridSearch() = default;
  bool GenerateAStarPath(
      double sx, double sy, double ex, double ey,
      const std::array<double, 4>& XYbounds, GridAStartResult* result);
  bool GenerateDpMap(double ex, double ey, const std::array<double, 4>& XYbounds,
                     const std::function<bool()>& should_stop = {},
                     ProgressCallback progress_callback = {});
  double CheckDpMap(double sx, double sy);
  const GridSearchStatistics& statistics() const { return statistics_; }
  GridSearchProgress progress_snapshot() const;

private:
  static double EuclidDistance(double x1, double y1, double x2,
                        double y2);
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      const std::shared_ptr<Node2d>& node);
  bool CheckConstraints(const std::shared_ptr<Node2d>& node,
                        const std::function<bool()>& should_stop = {});
  void LoadGridAStarResult(GridAStartResult* result);
  void PublishProgress(const ProgressCallback& callback, bool force = false);

private:
  double xy_grid_resolution_ = 0.0;
  std::array<double, 4> XYbounds_;
  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;
  std::shared_ptr<Node2d> start_node_;
  std::shared_ptr<Node2d> end_node_;
  std::shared_ptr<Node2d> final_node_;
  std::vector<common::math::Polygon2d> obstacles_;

  struct cmp {
    bool operator()(const std::pair<int64_t, double>& left,
                    const std::pair<int64_t, double>& right) const {
      return left.second > right.second;
    }
  };
  std::unordered_map<int64_t, std::shared_ptr<Node2d>> dp_map_;
  std::unordered_map<int64_t, bool> constraint_cache_;
  GridSearchStatistics statistics_;
  std::chrono::steady_clock::time_point dp_begin_;
  std::chrono::steady_clock::time_point last_progress_publish_;
  std::vector<double> settled_visualization_x_;
  std::vector<double> settled_visualization_y_;
  std::deque<std::pair<double, double>> recent_wavefront_;
  std::size_t visualization_stride_ = 1;
  static constexpr double kProgressPeriodSeconds = 0.15;
  static constexpr std::size_t kMaxVisualizedSettledCells = 8000;
  static constexpr std::size_t kMaxVisualizedWavefrontCells = 128;
};
}  // namespace planning
