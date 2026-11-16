#include "planning/hybrid_search_adapter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "common/math/math_utils.h"

namespace planning {
namespace {

constexpr double kEndpointPositionTolerance = 1e-6;
constexpr double kEndpointHeadingTolerance = 1e-6;
constexpr double kSteeringConsistencyTolerance = 1e-6;
constexpr double kSteeringBoundTolerance = 1e-4;

bool Finite(double value) { return std::isfinite(value); }

bool NormalizeResult(const HybridAStartResult& raw, bool reverse_search,
                     const scene::Pose2d& actual_start,
                     const scene::Pose2d& actual_goal,
                     const VehicleParameter& vehicle, double delta_t,
                     HybridAStartResult* normalized,
                     bool* steering_recomputed, std::string* error) {
  const std::size_t node_count = raw.x.size();
  if (!normalized || !steering_recomputed || node_count < 2 ||
      raw.y.size() != node_count || raw.phi.size() != node_count ||
      raw.gear.size() != node_count || raw.v.size() != node_count ||
      raw.accumulated_s.size() != node_count ||
      raw.a.size() + 1 != node_count ||
      raw.steer.size() + 1 != node_count || !Finite(delta_t) ||
      delta_t <= 0.0) {
    if (error) *error = "Hybrid A* result dimensions are inconsistent";
    return false;
  }

  *normalized = HybridAStartResult{};
  *steering_recomputed = false;
  normalized->x.resize(node_count);
  normalized->y.resize(node_count);
  normalized->phi.resize(node_count);
  for (std::size_t i = 0; i < node_count; ++i) {
    const std::size_t source = reverse_search ? node_count - 1 - i : i;
    if (!Finite(raw.x[source]) || !Finite(raw.y[source]) ||
        !Finite(raw.phi[source])) {
      if (error) *error = "Hybrid A* result contains non-finite poses";
      return false;
    }
    normalized->x[i] = raw.x[source];
    normalized->y[i] = raw.y[source];
    if (i == 0) {
      if (std::abs(common::math::NormalizeAngle(
              raw.phi[source] - actual_start.yaw)) >
          kEndpointHeadingTolerance) {
        if (error) *error = "Hybrid A* normalized start heading mismatch";
        return false;
      }
      normalized->phi[i] = actual_start.yaw;
    } else {
      normalized->phi[i] = normalized->phi[i - 1] +
          common::math::NormalizeAngle(raw.phi[source] -
                                       normalized->phi[i - 1]);
    }
  }

  if (std::hypot(normalized->x.front() - actual_start.x,
                 normalized->y.front() - actual_start.y) >
          kEndpointPositionTolerance ||
      std::hypot(normalized->x.back() - actual_goal.x,
                 normalized->y.back() - actual_goal.y) >
          kEndpointPositionTolerance ||
      std::abs(common::math::NormalizeAngle(
          normalized->phi.back() - actual_goal.yaw)) >
          kEndpointHeadingTolerance) {
    if (error) *error = "Hybrid A* normalized path endpoint mismatch";
    return false;
  }

  normalized->gear.reserve(node_count);
  for (std::size_t edge = 0; edge + 1 < node_count; ++edge) {
    const std::size_t source_edge =
        reverse_search ? node_count - 2 - edge : edge;
    normalized->gear.push_back(
        reverse_search ? !raw.gear[source_edge] : raw.gear[source_edge]);
  }
  normalized->gear.push_back(normalized->gear.back());

  normalized->v.assign(node_count, 0.0);
  std::size_t segment_begin = 0;
  while (segment_begin + 1 < node_count) {
    std::size_t segment_end = segment_begin + 1;
    while (segment_end + 1 < node_count &&
           normalized->gear[segment_end] ==
               normalized->gear[segment_begin]) {
      ++segment_end;
    }
    const double sign = normalized->gear[segment_begin] ? 1.0 : -1.0;
    for (std::size_t node = segment_begin + 1; node < segment_end; ++node) {
      normalized->v[node] = sign *
          std::hypot(normalized->x[node + 1] - normalized->x[node - 1],
                     normalized->y[node + 1] - normalized->y[node - 1]) /
          (2.0 * delta_t);
    }
    segment_begin = segment_end;
  }

  normalized->a.reserve(node_count - 1);
  normalized->steer.reserve(node_count - 1);
  normalized->accumulated_s.reserve(node_count);
  normalized->accumulated_s.push_back(0.0);
  for (std::size_t edge = 0; edge + 1 < node_count; ++edge) {
    normalized->a.push_back(
        (normalized->v[edge + 1] - normalized->v[edge]) / delta_t);
    const double distance = std::hypot(
        normalized->x[edge + 1] - normalized->x[edge],
        normalized->y[edge + 1] - normalized->y[edge]);
    const double signed_distance =
        normalized->gear[edge] ? distance : -distance;
    const double expected_steer = distance < 1e-12 ? 0.0 : std::atan(
        common::math::NormalizeAngle(normalized->phi[edge + 1] -
                                     normalized->phi[edge]) *
        vehicle.wheel_base / signed_distance);
    const std::size_t source_edge =
        reverse_search ? node_count - 2 - edge : edge;
    double steer = raw.steer[source_edge];
    if (!Finite(steer) ||
        std::abs(steer - expected_steer) >
            kSteeringConsistencyTolerance) {
      steer = expected_steer;
      *steering_recomputed = true;
    }
    if (!Finite(steer) ||
        std::abs(steer - expected_steer) >
            kSteeringConsistencyTolerance ||
        std::abs(steer) > vehicle.delta_max +
                               kSteeringBoundTolerance) {
      if (error) *error = "Hybrid A* steering is inconsistent with path geometry";
      return false;
    }
    normalized->steer.push_back(steer);
    normalized->accumulated_s.push_back(
        normalized->accumulated_s.back() + distance);
  }
  return true;
}

std::size_t CountGearSwitches(const HybridAStartResult& path) {
  if (path.x.size() < 2 || path.gear.size() != path.x.size()) return 0;
  std::size_t switches = 0;
  for (std::size_t edge = 1; edge + 1 < path.x.size(); ++edge) {
    if (path.gear[edge] != path.gear[edge - 1]) ++switches;
  }
  return switches;
}

}  // namespace

bool ParseHybridSearchDirection(const std::string& value,
                                HybridSearchDirection* direction) {
  if (!direction) return false;
  if (value == "start_to_goal") {
    *direction = HybridSearchDirection::kStartToGoal;
    return true;
  }
  if (value == "goal_to_start") {
    *direction = HybridSearchDirection::kGoalToStart;
    return true;
  }
  return false;
}

const char* HybridSearchDirectionName(HybridSearchDirection direction) {
  return direction == HybridSearchDirection::kGoalToStart
             ? "goal_to_start"
             : "start_to_goal";
}

ParkingPlannerConfig AdaptHybridSearchConfig(
    const ParkingPlannerConfig& configured,
    HybridSearchDirection direction) {
  ParkingPlannerConfig effective = configured;
  if (direction == HybridSearchDirection::kGoalToStart) {
    std::swap(effective.traj_forward_penalty,
              effective.traj_back_penalty);
  }
  return effective;
}

HybridSearchAdapter::HybridSearchAdapter(
    HybridSearchDirection direction, const ParkingPlannerConfig& configured,
    const VehicleParameter& vehicle,
    const std::vector<common::math::Polygon2d>& obstacles,
    bool save_debug_data)
    : direction_(direction),
      configured_(configured),
      vehicle_(vehicle),
      obstacles_(obstacles),
      save_debug_data_(save_debug_data) {}

HybridSearchAdapterResult HybridSearchAdapter::Search(
    const scene::Pose2d& actual_start, const scene::Pose2d& actual_goal,
    HybridAStar::ProgressCallback progress_callback,
    HybridAStar::CancellationCallback cancellation_callback) const {
  HybridSearchAdapterResult result;
  result.effective_config = AdaptHybridSearchConfig(configured_, direction_);
  HybridAStar planner(result.effective_config, vehicle_, obstacles_,
                      save_debug_data_);
  const bool reverse_search =
      direction_ == HybridSearchDirection::kGoalToStart;
  const scene::Pose2d& search_start =
      reverse_search ? actual_goal : actual_start;
  const scene::Pose2d& search_goal =
      reverse_search ? actual_start : actual_goal;
  HybridAStartResult raw;
  const bool search_succeeded = planner.Plan(
      search_start.x, search_start.y, search_start.yaw,
      search_goal.x, search_goal.y, search_goal.yaw, &raw,
      std::move(progress_callback), std::move(cancellation_callback));
  result.timed_out = planner.timed_out();
  result.timeout_seconds = planner.timeout_seconds();
  result.statistics = planner.statistics();
  if (!search_succeeded) {
    result.error = result.timed_out ? "Hybrid A* timed out"
                                    : "Hybrid A* search failed";
    return result;
  }
  if (!NormalizeResult(raw, reverse_search, actual_start, actual_goal,
                       vehicle_, result.effective_config.delta_t,
                       &result.path, &result.steering_recomputed,
                       &result.error)) {
    return result;
  }
  result.gear_switch_count = CountGearSwitches(result.path);
  result.succeeded = true;
  return result;
}

}  // namespace planning
