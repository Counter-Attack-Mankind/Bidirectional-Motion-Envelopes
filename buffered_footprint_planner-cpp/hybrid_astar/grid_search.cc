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

#include "hybrid_astar/grid_search.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace planning {

GridSearch::GridSearch(const ParkingPlannerConfig &config, const std::vector<common::math::Polygon2d> &obstacles): obstacles_(obstacles) {
  xy_grid_resolution_ = config.grid_a_star_xy_resolution;
  XYbounds_ = config.xy_bounds;
}

double GridSearch::EuclidDistance(const double x1, const double y1,
                                  const double x2, const double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool GridSearch::CheckConstraints(
    const std::shared_ptr<Node2d> &node,
    const std::function<bool()>& should_stop) {
  const double node_grid_x = node->GetGridX();
  const double node_grid_y = node->GetGridY();
  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }
  if (obstacles_.empty()) {
    return true;
  }
  const auto cached = constraint_cache_.find(node->GetIndex());
  if (cached != constraint_cache_.end()) return cached->second;
  for (const auto &ob : obstacles_) {
    if (should_stop && should_stop()) return false;
    ++statistics_.obstacle_checks;
    if(ob.IsPointIn({ node->GetX(), node->GetY() })) {
      constraint_cache_.emplace(node->GetIndex(), false);
      return false;
    }
  }
  constraint_cache_.emplace(node->GetIndex(), true);
  return true;
}

GridSearchProgress GridSearch::progress_snapshot() const {
  GridSearchProgress progress;
  progress.settled_x = settled_visualization_x_;
  progress.settled_y = settled_visualization_y_;
  progress.wavefront_x.reserve(recent_wavefront_.size());
  progress.wavefront_y.reserve(recent_wavefront_.size());
  for (const auto& point : recent_wavefront_) {
    progress.wavefront_x.push_back(point.first);
    progress.wavefront_y.push_back(point.second);
  }
  progress.resolution = xy_grid_resolution_;
  progress.statistics = statistics_;
  progress.statistics.elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    dp_begin_).count();
  return progress;
}

void GridSearch::PublishProgress(const ProgressCallback& callback,
                                 bool force) {
  if (!callback) return;
  const auto now = std::chrono::steady_clock::now();
  if (!force && std::chrono::duration<double>(now - last_progress_publish_)
                        .count() < kProgressPeriodSeconds) {
    return;
  }
  last_progress_publish_ = now;
  callback(progress_snapshot());
}

std::vector<std::shared_ptr<Node2d>> GridSearch::GenerateNextNodes(
    const std::shared_ptr<Node2d> &current_node) {
  double current_node_x = current_node->GetX();
  double current_node_y = current_node->GetY();
  double current_node_path_cost = current_node->GetPathCost();
  double diagonal_distance = std::sqrt(2.0);
  std::vector<std::shared_ptr<Node2d>> next_nodes;
  std::shared_ptr<Node2d> up = std::make_shared<Node2d>(
      current_node_x, current_node_y + xy_grid_resolution_, xy_grid_resolution_,
      XYbounds_);
  up->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(
      current_node_x + xy_grid_resolution_,
      current_node_y + xy_grid_resolution_, xy_grid_resolution_, XYbounds_);
  up_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> right =
      std::make_shared<Node2d>(current_node_x + xy_grid_resolution_,
                               current_node_y, xy_grid_resolution_, XYbounds_);
  right->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(
      current_node_x + xy_grid_resolution_,
      current_node_y - xy_grid_resolution_, xy_grid_resolution_, XYbounds_);
  down_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> down = std::make_shared<Node2d>(
      current_node_x, current_node_y - xy_grid_resolution_, xy_grid_resolution_,
      XYbounds_);
  down->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(
      current_node_x - xy_grid_resolution_,
      current_node_y - xy_grid_resolution_, xy_grid_resolution_, XYbounds_);
  down_left->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> left =
      std::make_shared<Node2d>(current_node_x - xy_grid_resolution_,
                               current_node_y, xy_grid_resolution_, XYbounds_);
  left->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(
      current_node_x - xy_grid_resolution_,
      current_node_y + xy_grid_resolution_, xy_grid_resolution_, XYbounds_);
  up_left->SetPathCost(current_node_path_cost + diagonal_distance);

  next_nodes.emplace_back(up);
  next_nodes.emplace_back(up_right);
  next_nodes.emplace_back(right);
  next_nodes.emplace_back(down_right);
  next_nodes.emplace_back(down);
  next_nodes.emplace_back(down_left);
  next_nodes.emplace_back(left);
  next_nodes.emplace_back(up_left);
  return next_nodes;
}

bool GridSearch::GenerateDpMap(
    const double ex, const double ey, const std::array<double, 4> &XYbounds,
    const std::function<bool()>& should_stop,
    ProgressCallback progress_callback) {
  dp_begin_ = std::chrono::steady_clock::now();
  last_progress_publish_ = dp_begin_ - std::chrono::seconds(1);
  statistics_ = GridSearchStatistics();
  settled_visualization_x_.clear();
  settled_visualization_y_.clear();
  recent_wavefront_.clear();
  constraint_cache_.clear();
  std::priority_queue<std::pair<int64_t, double>,
      std::vector<std::pair<int64_t, double>>, cmp>
      open_pq;
  std::unordered_map<int64_t, std::shared_ptr<Node2d>> open_set;
  dp_map_ = decltype(dp_map_)();
  XYbounds_ = XYbounds;
  // XYbounds with xmin, xmax, ymin, ymax
  max_grid_y_ = std::round((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_);
  max_grid_x_ = std::round((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_);
  const long double grid_cell_count =
      (static_cast<long double>(max_grid_x_) + 1.0L) *
      (static_cast<long double>(max_grid_y_) + 1.0L);
  if (grid_cell_count > 0.0L &&
      grid_cell_count <= static_cast<long double>(
          std::numeric_limits<std::size_t>::max())) {
    statistics_.total_grid_cells = static_cast<std::size_t>(grid_cell_count);
  }
  visualization_stride_ = std::max<std::size_t>(
      1, (statistics_.total_grid_cells + kMaxVisualizedSettledCells - 1) /
             kMaxVisualizedSettledCells);
  // Avoid a multi-second unordered_map rehash crossing the hard deadline.
  // The cap prevents malformed bounds from causing an unbounded reservation.
  if (grid_cell_count > 0.0L && grid_cell_count <= 10000000.0L) {
    const auto reserve_count = static_cast<std::size_t>(grid_cell_count);
    dp_map_.reserve(reserve_count);
    open_set.reserve(reserve_count);
    constraint_cache_.reserve(reserve_count);
  }
  PublishProgress(progress_callback, true);
  if (should_stop && should_stop()) return false;
  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_);
  open_set.emplace(end_node->GetIndex(), end_node);
  open_pq.emplace(end_node->GetIndex(), end_node->GetCost());
  statistics_.enqueue_count = 1;
  statistics_.queue_size = 1;
  statistics_.queue_peak = 1;

  // Grid a star begins
  while (!open_pq.empty()) {
    if (should_stop && should_stop()) return false;
    const int64_t current_id = open_pq.top().first;
    const double queued_cost = open_pq.top().second;
    open_pq.pop();
    statistics_.queue_size = open_pq.size();
    std::shared_ptr<Node2d> current_node = open_set[current_id];
    if (dp_map_.count(current_id) ||
        queued_cost > current_node->GetCost() + 1e-12) continue;
    if (should_stop && should_stop()) return false;
    dp_map_.emplace(current_node->GetIndex(), current_node);
    ++statistics_.settled_cells;
    if (progress_callback &&
        (statistics_.settled_cells - 1) % visualization_stride_ == 0 &&
        settled_visualization_x_.size() < kMaxVisualizedSettledCells) {
      settled_visualization_x_.push_back(current_node->GetX());
      settled_visualization_y_.push_back(current_node->GetY());
    }
    if (progress_callback) {
      recent_wavefront_.emplace_back(current_node->GetX(),
                                     current_node->GetY());
      if (recent_wavefront_.size() > kMaxVisualizedWavefrontCells) {
        recent_wavefront_.pop_front();
      }
    }
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(current_node));
    for (auto &next_node : next_nodes) {
      if (should_stop && should_stop()) return false;
      if (dp_map_.find(next_node->GetIndex()) != dp_map_.end()) {
        continue;
      }
      if (!CheckConstraints(next_node, should_stop)) {
        continue;
      }
      const auto open_node = open_set.find(next_node->GetIndex());
      if (open_node == open_set.end()) {
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
        ++statistics_.enqueue_count;
        statistics_.queue_size = open_pq.size();
        statistics_.queue_peak =
            std::max(statistics_.queue_peak, statistics_.queue_size);
      } else if (open_node->second->GetCost() >
                 next_node->GetCost() + 1e-12) {
        next_node->SetPreNode(current_node);
        open_node->second = next_node;
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
        ++statistics_.enqueue_count;
        statistics_.queue_size = open_pq.size();
        statistics_.queue_peak =
            std::max(statistics_.queue_peak, statistics_.queue_size);
      }
    }
    statistics_.queue_size = open_pq.size();
    statistics_.queue_peak =
        std::max(statistics_.queue_peak, statistics_.queue_size);
    PublishProgress(progress_callback);
  }
  statistics_.queue_size = 0;
  statistics_.elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    dp_begin_).count();
  PublishProgress(progress_callback, true);
  return true;
}

double GridSearch::CheckDpMap(const double sx, const double sy) {
  int64_t index = Node2d::CalcIndex(sx, sy, xy_grid_resolution_, XYbounds_);
  if (dp_map_.find(index) != dp_map_.end()) {
    return dp_map_[index]->GetCost() * xy_grid_resolution_;
  } else {
    return std::numeric_limits<double>::infinity();
  }
}

void GridSearch::LoadGridAStarResult(GridAStartResult *result) {
  (*result).path_cost = final_node_->GetPathCost() * xy_grid_resolution_;
  std::shared_ptr<Node2d> current_node = final_node_;
  std::vector<double> grid_a_x;
  std::vector<double> grid_a_y;
  while (current_node->GetPreNode() != nullptr) {
    grid_a_x.push_back(current_node->GetGridX() * xy_grid_resolution_ +
                       XYbounds_[0]);
    grid_a_y.push_back(current_node->GetGridY() * xy_grid_resolution_ +
                       XYbounds_[2]);
    current_node = current_node->GetPreNode();
  }
  std::reverse(grid_a_x.begin(), grid_a_x.end());
  std::reverse(grid_a_y.begin(), grid_a_y.end());
  (*result).x = std::move(grid_a_x);
  (*result).y = std::move(grid_a_y);
}
}  // namespace planning
