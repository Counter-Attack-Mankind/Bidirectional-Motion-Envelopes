#include "execution/trajectory_adapter.h"

#include <cmath>

namespace execution {

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
  return true;
}

bool TrajectoryAdapter::Convert(const optimizer::Trajectory& trajectory,
                                int vehicle_id, std::uint32_t task_id,
                                sandbox_msgs::Trajectory* message,
                                std::string* error) {
  if (!message) {
    if (error) *error = "trajectory output message is null";
    return false;
  }
  if (vehicle_id < 0 || !Validate(trajectory, error)) return false;
  message->header.stamp = ros::Time::now();
  message->header.frame_id = "map";
  message->header.seq = task_id;
  message->target = vehicle_id;
  message->points.clear();
  message->points.reserve(trajectory.nodes.size());
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
    message->points.push_back(point);
  }
  return true;
}

}  // namespace execution
