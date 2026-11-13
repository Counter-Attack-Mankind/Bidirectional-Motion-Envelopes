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

#include "planning/hybrid_astar/hybrid_a_star.h"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "common/util/csv_logger.h"

namespace planning {

using common::util::HybridAStarLogger;

using common::math::Box2d;
using common::math::Vec2d;

double HybridAStar::elapsed_seconds() const {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       search_begin_).count();
}

bool HybridAStar::DeadlineExceeded() {
  if (std::chrono::steady_clock::now() < search_deadline_) return false;
  timed_out_ = true;
  return true;
}

bool HybridAStar::StopRequested() {
  if (cancellation_callback_ && cancellation_callback_()) {
    cancelled_ = true;
    return true;
  }
  return DeadlineExceeded();
}

HybridAStar::HybridAStar(const ParkingPlannerConfig &config, 
                         const VehicleParameter &vehicle, 
                         const std::vector<common::math::Polygon2d> &obstacles,
                         bool save_debug_data)
    : obstacles_(obstacles), save_debug_data_(save_debug_data) {
  planner_open_space_config_ = config;
  vehicle_param_ = vehicle;
  XYbounds_ = config.xy_bounds;
  reed_shepp_generator_ = std::unique_ptr<ReedShepp>(
      new ReedShepp(vehicle_param_, planner_open_space_config_));
  next_node_num_ =
      planner_open_space_config_.next_node_num;
  max_steer_angle_ =
      vehicle_param_.delta_max / vehicle_param_.steer_ratio;
  maximum_curvature_ =
      std::tan(max_steer_angle_) / vehicle_param_.wheel_base;
  search_clearance_ = planner_open_space_config_.search_clearance;
  step_size_ = planner_open_space_config_.step_size;
  xy_grid_resolution_ =
      planner_open_space_config_.xy_grid_resolution;
  delta_t_ = planner_open_space_config_.delta_t;
  traj_forward_penalty_ =
      planner_open_space_config_.traj_forward_penalty;
  traj_back_penalty_ =
      planner_open_space_config_.traj_back_penalty;
  traj_gear_switch_penalty_ =
      planner_open_space_config_.traj_gear_switch_penalty;
  traj_steer_penalty_ =
      planner_open_space_config_.traj_steer_penalty;
  traj_steer_change_penalty_ = planner_open_space_config_.traj_steer_change_penalty;
}

void HybridAStar::PublishProgress(
    const std::shared_ptr<Node3d>& current_node, HybridSearchStatus status,
    bool force) {
  if (!progress_callback_) return;
  const auto now = std::chrono::steady_clock::now();
  if (!force && std::chrono::duration<double>(now - last_progress_publish_)
                        .count() < kProgressPeriodSeconds) {
    return;
  }
  last_progress_publish_ = now;
  HybridSearchProgress progress;
  progress.expanded_x = expanded_visualization_x_;
  progress.expanded_y = expanded_visualization_y_;
  progress.elapsed_seconds = elapsed_seconds();
  progress.expanded_count = expanded_node_count_;
  progress.status = status;
  if (current_node) {
    progress.has_current = true;
    progress.current_x = current_node->GetX();
    progress.current_y = current_node->GetY();
    progress.current_phi = current_node->GetPhi();
    std::shared_ptr<Node3d> path_node = current_node;
    while (path_node && progress.backtrack_x.size() < kMaxBacktrackNodes) {
      progress.backtrack_x.push_back(path_node->GetX());
      progress.backtrack_y.push_back(path_node->GetY());
      path_node = path_node->GetPreNode();
    }
    std::reverse(progress.backtrack_x.begin(), progress.backtrack_x.end());
    std::reverse(progress.backtrack_y.begin(), progress.backtrack_y.end());
  }
  progress_callback_(std::move(progress));
}

bool HybridAStar::AnalyticExpansion(const std::shared_ptr<Node3d>& current_node) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  if (!reed_shepp_generator_->ShortestFeasibleRSP(
          current_node, end_node_,
          [this](const ReedSheppPath& path) {
            return RSPCheck(std::make_shared<ReedSheppPath>(path));
          }, reeds_shepp_to_check,
          [this, current_node]() {
            PublishProgress(current_node, HybridSearchStatus::kSearching,
                            false);
            return StopRequested();
          })) {
    return false;
  }

  // 记录Reed-Shepp路径
  HybridAStarLogger::LogReedSheppPath(reeds_shepp_to_check->x,
                                      reeds_shepp_to_check->y,
                                      reeds_shepp_to_check->phi,
                                      reeds_shepp_to_check->gear,
                                      reeds_shepp_to_check->total_length);

  // load the whole RSP as nodes and add to the close set
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::make_shared<Node3d>(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_);
  return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(const std::shared_ptr<Node3d>& node) {
  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (StopRequested()) return false;
    common::math::Pose pose(traversed_x[i], traversed_y[i], traversed_phi[i]);
    auto vehicle_box = vehicle_param_.GenerateBox(pose);
    for (const auto& corner : vehicle_box.corners()) {
      if (corner.x() > XYbounds_[1] || corner.x() < XYbounds_[0] ||
          corner.y() > XYbounds_[3] || corner.y() < XYbounds_[2]) {
        return false;
      }
    }
    common::math::Polygon2d vehicle_polygon(vehicle_box);
    for (const auto &ob : obstacles_) {
      if (vehicle_polygon.DistanceTo(ob) + 1e-12 < search_clearance_) {
        return false;
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::make_shared<Node3d>(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_);
  if (!reeds_shepp_to_end->gear.empty()) {
    end_node->SetDirec(reeds_shepp_to_end->gear.back());
    std::vector<bool> edge_gears;
    for (size_t i = 1; i < reeds_shepp_to_end->gear.size(); ++i) {
      edge_gears.push_back(reeds_shepp_to_end->gear[i]);
    }
    end_node->SetEdgeGears(std::move(edge_gears));
  }
  end_node->SetPre(std::move(current_node));
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    const std::shared_ptr<Node3d>& current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;
  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
        static_cast<double>(next_node_index);
    traveled_distance = step_size_;
  } else {
    size_t index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
        static_cast<double>(index);
    traveled_distance = -step_size_;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  for (size_t i = 0; i < arc / step_size_; ++i) {
    if ((i & 63U) == 0U && StopRequested()) return nullptr;
    const double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base * std::tan(steering));
    const double midpoint_phi = last_phi + 0.5 *
        common::math::NormalizeAngle(next_phi - last_phi);
    const double next_x = last_x + traveled_distance * std::cos(midpoint_phi);
    const double next_y = last_y + traveled_distance * std::sin(midpoint_phi);
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  std::shared_ptr<Node3d> next_node = std::make_shared<Node3d>(
      intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 planner_open_space_config_);
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetEdgeGears(std::vector<bool>(intermediate_x.size() - 1,
                                           traveled_distance > 0.0));
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(const std::shared_ptr<Node3d>& current_node,
                                    const std::shared_ptr<Node3d>& next_node) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
  // evaluate heuristic cost
  next_node->SetHeuCost(DistanceHeadingHeuristic(next_node));
}

double HybridAStar::TrajCost(const std::shared_ptr<Node3d>& current_node,
                             const std::shared_ptr<Node3d>& next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::DistanceHeadingHeuristic(
    const std::shared_ptr<Node3d>& next_node) {
  const auto begin = std::chrono::steady_clock::now();
  const double distance = std::hypot(next_node->GetX() - end_node_->GetX(),
                                     next_node->GetY() - end_node_->GetY());
  const double heading = std::abs(common::math::NormalizeAngle(
      next_node->GetPhi() - end_node_->GetPhi()));
  const double value = std::max(distance, heading / maximum_curvature_);
  statistics_.heuristic_elapsed_seconds +=
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
          .count();
  ++statistics_.heuristic_call_count;
  return value;
}

double HybridAStar::CalculatePathMinimumClearance(
    const HybridAStartResult& result) const {
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < result.x.size(); ++i) {
    const common::math::Polygon2d body(vehicle_param_.GenerateBox(
        common::math::Pose(result.x[i], result.y[i], result.phi[i])));
    for (const auto& obstacle : obstacles_) {
      minimum = std::min(minimum, body.DistanceTo(obstacle));
    }
  }
  return minimum;
}

bool HybridAStar::GetResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  std::vector<bool> hybrid_a_gear;
  while (current_node->GetPreNode() != nullptr) {
    if (StopRequested()) return false;
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      printf("result size check failed");
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      printf("states sizes are not equal");
      return false;
    }
    const auto& edge_gears = current_node->GetEdgeGears();
    if (edge_gears.size() + 1 != x.size()) {
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    hybrid_a_gear.insert(hybrid_a_gear.end(), edge_gears.rbegin(),
                         edge_gears.rend());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  std::reverse(hybrid_a_gear.begin(), hybrid_a_gear.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;
  if (hybrid_a_gear.size() + 1 != result->x.size()) return false;
  result->gear = std::move(hybrid_a_gear);
  result->gear.push_back(result->gear.empty() ? true : result->gear.back());

  if (!GetTemporalProfile(result)) {
    printf("GetSpeedProfile from Hybrid Astar path fails");
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    printf("state sizes not equal");
    return false;
  }
  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1) {
    printf("control sizes not equal or not right");
    return false;
  }
  return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    printf("result size check when generating speed and acceleration fail");
    return false;
  }
  const size_t x_size = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zeros
  if (result->gear.size() != x_size) {
    return false;
  }
  result->v.clear();
  result->a.clear();
  result->steer.clear();
  result->v.push_back(0.0);
  for (size_t i = 1; i + 1 < x_size; ++i) {
    if ((i & 255U) == 0U && StopRequested()) return false;
    const double distance = std::hypot(result->x[i + 1] - result->x[i - 1],
                                       result->y[i + 1] - result->y[i - 1]);
    const double discrete_v = (result->gear[i] ? 1.0 : -1.0) *
                              distance / (2.0 * delta_t_);
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (size_t i = 0; i + 1 < x_size; ++i) {
    if ((i & 255U) == 0U && StopRequested()) return false;
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  // load steering from phi
  for (size_t i = 0; i + 1 < x_size; ++i) {
    if ((i & 255U) == 0U && StopRequested()) return false;
    const double distance = std::hypot(result->x[i + 1] - result->x[i],
                                       result->y[i + 1] - result->y[i]);
    const double signed_distance = result->gear[i] ? distance : -distance;
    const double discrete_steer = distance < 1e-12 ? 0.0 : std::atan(
        common::math::NormalizeAngle(result->phi[i + 1] - result->phi[i]) *
        vehicle_param_.wheel_base / signed_distance);
    result->steer.push_back(discrete_steer);
  }
  return true;
}

bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() < 2 || x.size() != y.size() || x.size() != phi.size() ||
      x.size() != result.gear.size()) {
    printf("states sizes are not equal when do trajectory partitioning of Hybrid A Star result");
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  bool current_gear = result.gear.front();
  current_traj->x.push_back(x.front());
  current_traj->y.push_back(y.front());
  current_traj->phi.push_back(phi.front());
  current_traj->gear.push_back(current_gear);
  for (size_t i = 0; i < horizon - 1; ++i) {
    if ((i & 255U) == 0U && StopRequested()) return false;
    const bool gear = result.gear[i];
    if (gear != current_gear) {
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      current_traj->gear.push_back(gear);
    }
    current_traj->x.push_back(x[i + 1]);
    current_traj->y.push_back(y[i + 1]);
    current_traj->phi.push_back(phi[i + 1]);
    current_traj->gear.push_back(gear);
  }


  // Retrieve v, a and steer from path
  for (auto& result : *partitioned_result) {
    if (StopRequested()) return false;
    if (!GenerateSpeedAcceleration(&result)) {
      printf("GenerateSpeedAcceleration fail");
      return false;
    }
  }

  return true;
}

bool HybridAStar::GetTemporalProfile(HybridAStartResult* result) {
  std::vector<HybridAStartResult> partitioned_results;
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    return false;
  }
  HybridAStartResult stitched_result;
  for (const auto& result : partitioned_results) {
    if (StopRequested()) return false;
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(result.gear.begin(), result.gear.end() - 1,
              std::back_inserter(stitched_result.gear));
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    std::copy(result.a.begin(), result.a.end(),
              std::back_inserter(stitched_result.a));
    std::copy(result.steer.begin(), result.steer.end(),
              std::back_inserter(stitched_result.steer));
  }
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.gear.push_back(partitioned_results.back().gear.back());
  stitched_result.v.push_back(partitioned_results.back().v.back());
  stitched_result.accumulated_s.reserve(stitched_result.x.size());
  stitched_result.accumulated_s.push_back(0.0);
  for (size_t i = 1; i < stitched_result.x.size(); ++i) {
    if ((i & 255U) == 0U && StopRequested()) return false;
    stitched_result.accumulated_s.push_back(
        stitched_result.accumulated_s.back() +
        std::hypot(stitched_result.x[i] - stitched_result.x[i - 1],
                   stitched_result.y[i] - stitched_result.y[i - 1]));
  }
  *result = stitched_result;
  return true;
}

bool HybridAStar::Plan(
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    HybridAStartResult* result, ProgressCallback progress_callback,
    CancellationCallback cancellation_callback) {
  // clear containers
  search_begin_ = std::chrono::steady_clock::now();
  search_deadline_ = search_begin_ +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              planner_open_space_config_.hybrid_astar_timeout));
  timed_out_ = false;
  cancelled_ = false;
  progress_callback_ = std::move(progress_callback);
  cancellation_callback_ = std::move(cancellation_callback);
  last_progress_publish_ = search_begin_ - std::chrono::seconds(1);
  expanded_visualization_x_.clear();
  expanded_visualization_y_.clear();
  expanded_node_count_ = 0;
  statistics_ = HybridAStarStatistics{};
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;

  if (save_debug_data_) HybridAStarLogger::Initialize();
  printf("Hybrid A* search started (timeout %.1f s)\n",
         planner_open_space_config_.hybrid_astar_timeout);

  // load nodes and obstacles
  start_node_.reset(
      new Node3d(sx, sy, sphi, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d(ex, ey, ephi, XYbounds_, planner_open_space_config_));
  if (!ValidityCheck(start_node_)) {
    printf(timed_out_ ? "Hybrid A* timed out while checking start node\n"
                      : "start_node violates bounds or search clearance\n");
    PublishProgress(start_node_,
                    timed_out_ ? HybridSearchStatus::kTimedOut
                               : HybridSearchStatus::kFailed,
                    true);
    HybridAStarLogger::Finalize();
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    printf(timed_out_ ? "Hybrid A* timed out while checking goal node\n"
                      : "end_node violates bounds or search clearance\n");
    PublishProgress(start_node_,
                    timed_out_ ? HybridSearchStatus::kTimedOut
                               : HybridSearchStatus::kFailed,
                    true);
    HybridAStarLogger::Finalize();
    return false;
  }
  PublishProgress(start_node_, HybridSearchStatus::kSearching, true);

  // load open set, pq
  start_node_->SetHeuCost(DistanceHeadingHeuristic(start_node_));
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

  // Hybrid A* begins
  std::shared_ptr<Node3d> last_expanded = start_node_;
  size_t admitted_node_count = 0;
  while (!open_pq_.empty()) {
    if (StopRequested()) break;
    // CRITICAL FIX: Add exploration limit to prevent infinite search
    if (admitted_node_count >= planner_open_space_config_.explore_node_num) {
      printf("Hybrid A* reached exploration limit: %zu nodes explored\n",
             admitted_node_count);
      break;
    }

    // take out the lowest cost neighboring node
    const int64_t current_id = open_pq_.top().first;
    const double queued_cost = open_pq_.top().second;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    if (close_set_.count(current_id) ||
        queued_cost > current_node->GetCost() + 1e-12) continue;

    last_expanded = current_node;
    ++expanded_node_count_;
    statistics_.expanded_node_count = expanded_node_count_;
    if (expanded_visualization_x_.size() < kMaxVisualizedExpandedNodes) {
      expanded_visualization_x_.push_back(current_node->GetX());
      expanded_visualization_y_.push_back(current_node->GetY());
    }
    PublishProgress(current_node, HybridSearchStatus::kSearching, false);

    // OPTIMIZATION: Check distance to goal - try analytic expansion more frequently when close
    double dx = current_node->GetX() - end_node_->GetX();
    double dy = current_node->GetY() - end_node_->GetY();
    double distance_to_goal = std::sqrt(dx * dx + dy * dy);

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends.
    if (AnalyticExpansion(current_node)) {
      printf("Hybrid A* found solution at distance %.2f from goal\n", distance_to_goal);
      break;
    }
    if (StopRequested()) break;
    close_set_.emplace(current_node->GetIndex(), current_node);
    for (size_t i = 0; i < next_node_num_; ++i) {
      if (StopRequested()) break;
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      CalculateNodeCost(current_node, next_node);
      const auto existing = open_set_.find(next_node->GetIndex());
      if (existing == open_set_.end() ||
          next_node->GetCost() + 1e-12 < existing->second->GetCost()) {
        if (existing == open_set_.end()) ++admitted_node_count;
        open_set_.emplace(next_node->GetIndex(), next_node);
        if (existing != open_set_.end()) existing->second = next_node;
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());

        // 记录搜索节点
        if (save_debug_data_) {
          HybridAStarLogger::LogSearchNode(
              next_node->GetX(), next_node->GetY(), next_node->GetPhi(),
              next_node->GetTrajCost(), next_node->GetHeuCost(),
              next_node->GetDirec(), next_node->GetSteer(),
              expanded_node_count_);
        }
      }
    }
  }
  if (final_node_ == nullptr) {
    printf("Hybrid A* search %s after %.3f s; expanded=%zu; "
           "heuristic_calls=%zu; heuristic_time=%.6f s\n",
           timed_out_ ? "timed out" : (cancelled_ ? "cancelled" : "failed"),
           elapsed_seconds(), expanded_node_count_,
           statistics_.heuristic_call_count,
           statistics_.heuristic_elapsed_seconds);
    PublishProgress(last_expanded, timed_out_ ? HybridSearchStatus::kTimedOut :
                    (cancelled_ ? HybridSearchStatus::kCancelled :
                                  HybridSearchStatus::kFailed), true);
    HybridAStarLogger::Finalize();
    return false;
  }
  if (!GetResult(result)) {
    PublishProgress(last_expanded, timed_out_ ? HybridSearchStatus::kTimedOut :
                    (cancelled_ ? HybridSearchStatus::kCancelled :
                                  HybridSearchStatus::kFailed), true);
    HybridAStarLogger::Finalize();
    return false;
  }

  // Calculate path statistics
  double path_length = 0.0;
  for (size_t i = 1; i < result->x.size(); ++i) {
    if ((i & 255U) == 0U && StopRequested()) {
      PublishProgress(last_expanded,
                      timed_out_ ? HybridSearchStatus::kTimedOut
                                 : HybridSearchStatus::kCancelled,
                      true);
      HybridAStarLogger::Finalize();
      return false;
    }
    double dx = result->x[i] - result->x[i-1];
    double dy = result->y[i] - result->y[i-1];
    path_length += std::sqrt(dx * dx + dy * dy);
  }
  statistics_.minimum_path_clearance = CalculatePathMinimumClearance(*result);

  printf("Hybrid A* search succeeded in %.3f s; expanded=%zu; points=%zu; "
         "length=%.2f m; heuristic_calls=%zu; heuristic_time=%.6f s; "
         "minimum_clearance=%.4f m\n", elapsed_seconds(),
         expanded_node_count_, result->x.size(), path_length,
         statistics_.heuristic_call_count,
         statistics_.heuristic_elapsed_seconds,
         statistics_.minimum_path_clearance);
  PublishProgress(last_expanded, HybridSearchStatus::kSucceeded, true);

  HybridAStarLogger::Finalize();
  return true;
}
}  // namespace planning
