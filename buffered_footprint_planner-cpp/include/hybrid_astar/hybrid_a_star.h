/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <algorithm>
#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <array>
#include <functional>

#include "common/math/math_utils.h"
#include "hybrid_astar/grid_search.h"
#include "hybrid_astar/node3d.h"
#include "hybrid_astar/reeds_shepp_path.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "hybrid_astar/parking_planner_config.h"

namespace planning {

struct HybridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<bool> gear;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
};

enum class HybridSearchStatus {
  kSearching,
  kSucceeded,
  kFailed,
  kTimedOut,
  kCancelled,
};

enum class HybridSearchStage {
  kBuildingDpMap,
  kHybridSearch,
};

// Bounded, presentation-only snapshot. It never participates in search costs
// or node ordering.
struct HybridSearchProgress {
  HybridSearchStage stage = HybridSearchStage::kHybridSearch;
  std::vector<double> expanded_x;
  std::vector<double> expanded_y;
  std::vector<double> backtrack_x;
  std::vector<double> backtrack_y;
  double current_x = 0.0;
  double current_y = 0.0;
  double current_phi = 0.0;
  double elapsed_seconds = 0.0;
  std::size_t expanded_count = 0;
  std::vector<double> dp_settled_x;
  std::vector<double> dp_settled_y;
  std::vector<double> dp_wavefront_x;
  std::vector<double> dp_wavefront_y;
  double dp_grid_resolution = 0.0;
  GridSearchStatistics dp_statistics;
  bool has_current = false;
  HybridSearchStatus status = HybridSearchStatus::kSearching;
};

class HybridAStar {
public:
  using ProgressCallback = std::function<void(HybridSearchProgress)>;
  using CancellationCallback = std::function<bool()>;

  HybridAStar(const ParkingPlannerConfig &config, 
              const VehicleParameter &vehicle, 
              const std::vector<common::math::Polygon2d> &obstacles,
              bool save_debug_data = false);
  virtual ~HybridAStar() = default;
  bool Plan(double sx, double sy, double sphi, double ex, double ey, double ephi,
            HybridAStartResult* result,
            ProgressCallback progress_callback = {},
            CancellationCallback cancellation_callback = {});
  bool timed_out() const { return timed_out_; }
  double timeout_seconds() const {
    return planner_open_space_config_.hybrid_astar_timeout;
  }
  double elapsed_seconds() const;
  bool TrajectoryPartition(const HybridAStartResult& result,
                           std::vector<HybridAStartResult>* partitioned_result);

private:
  bool AnalyticExpansion(const std::shared_ptr<Node3d>& current_node);
  // check collision and validity
  bool ValidityCheck(const std::shared_ptr<Node3d>& node);
  // check Reeds Shepp path collision and validity
  bool RSPCheck(const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end);
  // load the whole RSP as nodes and add to the close set
  std::shared_ptr<Node3d> LoadRSPinCS(
      const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
      std::shared_ptr<Node3d> current_node);
  std::shared_ptr<Node3d> Next_node_generator(
      const std::shared_ptr<Node3d>& current_node, size_t next_node_index);
  void CalculateNodeCost(const std::shared_ptr<Node3d>& current_node,
                         const std::shared_ptr<Node3d>& next_node);
  double TrajCost(const std::shared_ptr<Node3d>& current_node,
                  const std::shared_ptr<Node3d>& next_node);
  double HoloObstacleHeuristic(const std::shared_ptr<Node3d>& next_node);
  bool GetResult(HybridAStartResult* result);
  bool GetTemporalProfile(HybridAStartResult* result);
  bool GenerateSpeedAcceleration(HybridAStartResult* result);
  bool DeadlineExceeded();
  bool StopRequested();
  void PublishProgress(const std::shared_ptr<Node3d>& current_node,
                       HybridSearchStatus status, bool force);
  void PublishDpProgress(const GridSearchProgress& grid_progress,
                         HybridSearchStatus status);

private:
  ParkingPlannerConfig planner_open_space_config_;
  VehicleParameter vehicle_param_;
  size_t next_node_num_ = 0;
  double max_steer_angle_ = 0.0;
  double step_size_ = 0.0;
  double xy_grid_resolution_ = 0.0;
  double delta_t_ = 0.0;
  double traj_forward_penalty_ = 0.0;
  double traj_back_penalty_ = 0.0;
  double traj_gear_switch_penalty_ = 0.0;
  double traj_steer_penalty_ = 0.0;
  double traj_steer_change_penalty_ = 0.0;
  double heu_rs_forward_penalty_ = 0.0;
  double heu_rs_back_penalty_ = 0.0;
  double heu_rs_gear_switch_penalty_ = 0.0;
  double heu_rs_steer_penalty_ = 0.0;
  double heu_rs_steer_change_penalty_ = 0.0;
  std::array<double, 4> XYbounds_;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> final_node_;
  std::vector<common::math::Polygon2d> obstacles_;

  struct cmp {
    bool operator()(const std::pair<int64_t, double>& left,
                    const std::pair<int64_t, double>& right) const {
      return left.second > right.second;
    }
  };
  std::priority_queue<std::pair<int64_t, double>,
      std::vector<std::pair<int64_t, double>>, cmp>
      open_pq_;
  std::unordered_map<int64_t, std::shared_ptr<Node3d>> open_set_;
  std::unordered_map<int64_t, std::shared_ptr<Node3d>> close_set_;
  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;
  std::chrono::steady_clock::time_point search_begin_;
  std::chrono::steady_clock::time_point search_deadline_;
  bool timed_out_ = false;
  bool cancelled_ = false;
  bool save_debug_data_ = false;
  ProgressCallback progress_callback_;
  CancellationCallback cancellation_callback_;
  std::chrono::steady_clock::time_point last_progress_publish_;
  std::vector<double> expanded_visualization_x_;
  std::vector<double> expanded_visualization_y_;
  std::size_t expanded_node_count_ = 0;
  static constexpr double kProgressPeriodSeconds = 0.15;
  static constexpr std::size_t kMaxVisualizedExpandedNodes = 10000;
  static constexpr std::size_t kMaxBacktrackNodes = 1000;
};

}  // namespace planning
