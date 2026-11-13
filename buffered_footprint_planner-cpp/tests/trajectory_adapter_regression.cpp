#include <cassert>
#include <cmath>
#include <string>

#include <ros/time.h>

#include "execution/trajectory_adapter.h"

int main() {
  ros::Time::init();
  optimizer::Trajectory trajectory;
  trajectory.nodes.resize(3);
  trajectory.intervals.resize(2);
  trajectory.nodes[0].x = 1.0;
  trajectory.nodes[0].v = 0.2;
  trajectory.nodes[1].x = 1.1;
  trajectory.nodes[1].v = 0.0;
  trajectory.nodes[2].x = 1.0;
  trajectory.nodes[2].v = -0.1;
  trajectory.intervals[0].dt = 0.07;
  trajectory.intervals[1].dt = 0.13;

  sandbox_msgs::Trajectory message;
  std::string error;
  assert(execution::TrajectoryAdapter::Convert(
      trajectory, 7, 42, &message, &error));
  assert(message.target == 7);
  assert(message.header.seq == 42);
  assert(message.points.size() == 3);
  assert(std::abs(message.points[0].time) < 1e-12);
  assert(std::abs(message.points[1].time - 0.07) < 1e-12);
  assert(std::abs(message.points[2].time - 0.20) < 1e-12);
  assert(message.points[2].velocity < 0.0);

  trajectory.intervals[1].dt = 0.0;
  assert(!execution::TrajectoryAdapter::Validate(trajectory, &error));
  return 0;
}
