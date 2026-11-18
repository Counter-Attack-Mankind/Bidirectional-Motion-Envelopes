#include "execution/trajectory_adapter.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace execution {
namespace {

constexpr double kMotionEpsilon = 1e-4;
constexpr double kVelocityEvidence = 0.01;

bool NormalizeIntervalGears(const optimizer::Trajectory& trajectory,
                            std::vector<bool>* gears, std::string* error) {
  gears->clear();
  gears->reserve(trajectory.intervals.size());
  std::vector<bool> stationary(trajectory.intervals.size(), false);
  std::vector<double> interval_distances;
  interval_distances.reserve(trajectory.intervals.size());
  for (std::size_t i = 0; i < trajectory.intervals.size(); ++i) {
    const auto& first = trajectory.nodes[i];
    const auto& second = trajectory.nodes[i + 1];
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double distance = std::hypot(dx, dy);
    const bool gear = trajectory.intervals[i].gear;
    if (distance > kMotionEpsilon) {
      const double tangent = std::atan2(dy, dx);
      const double midpoint_yaw = first.theta + 0.5 *
          std::remainder(second.theta - first.theta, 2.0 * M_PI);
      const double alignment = std::cos(tangent - midpoint_yaw);
      if (std::abs(alignment) < 0.25 || (alignment > 0.0) != gear) {
        if (error) *error = "trajectory interval " + std::to_string(i) +
                            " displacement conflicts with gear";
        return false;
      }
    }
    if (std::abs(first.v) > kVelocityEvidence && (first.v > 0.0) != gear) {
      if (error) *error = "trajectory interval " + std::to_string(i) +
                          " velocity sign conflicts with gear";
      return false;
    }
    // Forward Euler drives interval i with node i. A non-zero velocity at
    // node i+1 therefore belongs to the following interval and must not turn
    // a geometrically stationary interval into a spurious motion segment.
    stationary[i] = distance <= kMotionEpsilon &&
                    std::abs(first.v) <= kVelocityEvidence;
    gears->push_back(gear);
    interval_distances.push_back(distance);
  }
  for (std::size_t i = 0; i < gears->size(); ++i) {
    if (!stationary[i]) continue;
    std::size_t previous = i;
    while (previous > 0 && stationary[previous - 1]) --previous;
    std::size_t next = i + 1;
    while (next < gears->size() && stationary[next]) ++next;
    if (previous > 0) {
      (*gears)[i] = (*gears)[previous - 1];
    } else if (next < gears->size()) {
      (*gears)[i] = (*gears)[next];
    }
  }
  double segment_distance = 0.0;
  for (std::size_t i = 0; i < gears->size(); ++i) {
    if (i > 0 && (*gears)[i] != (*gears)[i - 1]) {
      if (segment_distance < kMotionEpsilon) {
        if (error) *error = "trajectory contains a zero-length motion segment";
        return false;
      }
      segment_distance = 0.0;
    }
    segment_distance += interval_distances[i];
  }
  if (segment_distance < kMotionEpsilon) {
    if (error) *error = "trajectory contains no executable motion";
    return false;
  }
  return true;
}

}  // namespace

bool TrajectoryAdapter::Validate(const optimizer::Trajectory& trajectory,
                                 std::string* error) {
  if (trajectory.nodes.size() < 2) {
    if (error) *error = "trajectory requires at least two nodes";
    return false;
  }
  if (trajectory.intervals.size() + 1 != trajectory.nodes.size()) {
    if (error) *error = "interval count must be node count minus one";
    return false;
  }
  for (std::size_t i = 0; i < trajectory.nodes.size(); ++i) {
    const auto& node = trajectory.nodes[i];
    if (!std::isfinite(node.x) || !std::isfinite(node.y) ||
        !std::isfinite(node.theta) || !std::isfinite(node.v) ||
        !std::isfinite(node.a)) {
      if (error) *error = "trajectory node " + std::to_string(i) +
                          " contains a non-finite value";
      return false;
    }
  }
  for (std::size_t i = 0; i < trajectory.intervals.size(); ++i) {
    if (!std::isfinite(trajectory.intervals[i].dt) ||
        trajectory.intervals[i].dt <= 0.0) {
      if (error) *error = "trajectory interval " + std::to_string(i) +
                          " has a non-positive time step";
      return false;
    }
  }
  std::vector<bool> normalized_gears;
  return NormalizeIntervalGears(trajectory, &normalized_gears, error);
}

bool TrajectoryAdapter::Convert(const optimizer::Trajectory& trajectory,
                                int vehicle_id, std::uint32_t task_id,
                                const std::string& result_directory,
                                int obstacle_count,
                                sandbox_msgs::Trajectory* message,
                                std::string* error) {
  if (!message) {
    if (error) *error = "trajectory output message is null";
    return false;
  }
  if (vehicle_id < 0 || !Validate(trajectory, error)) return false;
  message->header.stamp = ros::Time::now();
  message->header.seq = 0;  // Never use transport metadata as business identity.
  message->header.frame_id = "map";
  message->target = vehicle_id;
  message->task_id = task_id;
  message->result_directory = result_directory;
  message->obstacle_count = obstacle_count;
  message->points.clear();
  message->points.reserve(trajectory.nodes.size());
  std::vector<bool> interval_gears;
  if (!NormalizeIntervalGears(trajectory, &interval_gears, error)) return false;
  double time = 0.0;
  for (std::size_t i = 0; i < trajectory.nodes.size(); ++i) {
    if (i > 0) time += trajectory.intervals[i - 1].dt;
    sandbox_msgs::TrajectoryPoint point;
    point.time = time;
    point.x = trajectory.nodes[i].x;
    point.y = trajectory.nodes[i].y;
    point.yaw = trajectory.nodes[i].theta;
    point.velocity = trajectory.nodes[i].v;
    point.acceleration = trajectory.nodes[i].a;
    const std::size_t interval =
        std::min(i, trajectory.intervals.size() - 1);
    point.gear = interval_gears[interval] ? 1 : -1;
    message->points.push_back(point);
  }
  return true;
}

}  // namespace execution
