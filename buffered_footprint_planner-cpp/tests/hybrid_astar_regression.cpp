// Regression checks for the Apollo-derived ROS adapter. Build standalone with
// hybrid_astar and common_math, or add to a local catkin test target.
#include <cassert>
#include <cmath>
#include <memory>

#include "hybrid_astar/hybrid_a_star.h"

int main() {
  VehicleParameter vehicle;
  vehicle.GenerateDisc();
  ParkingPlannerConfig config;
  config.xy_bounds = {0.0, 1.5, 0.0, 1.5};
  config.xy_grid_resolution = 0.02;
  config.phi_grid_resolution = 0.05;
  config.step_size = 0.03;
  planning::ReedShepp rs(vehicle, config);
  auto start = std::make_shared<planning::Node3d>(0.65, 0.65, 0.0);
  auto goal = std::make_shared<planning::Node3d>(0.85, 0.85, 1.2);
  int candidates = 0;
  auto selected = std::make_shared<planning::ReedSheppPath>();
  assert(rs.ShortestFeasibleRSP(start, goal,
      [&](const planning::ReedSheppPath&) { return ++candidates > 1; },
      selected));
  assert(candidates > 1 && selected->x.size() == selected->gear.size());

  planning::HybridAStar planner(config, vehicle, {});
  planning::HybridAStartResult forward;
  assert(planner.Plan(0.45, 0.75, 0.0, 1.05, 0.75, 0.0, &forward));
  assert(planner.statistics().heuristic_call_count >= 1);
  assert(planner.statistics().expanded_node_count >= 1);
  assert(forward.x.size() == forward.gear.size());
  for (size_t i = 0; i + 1 < forward.gear.size(); ++i) {
    assert(forward.gear[i]);
  }

  planning::HybridAStartResult reverse;
  assert(planner.Plan(1.05, 0.75, 0.0, 0.45, 0.75, 0.0, &reverse));
  assert(reverse.x.size() == reverse.gear.size());
  for (size_t i = 0; i + 1 < reverse.gear.size(); ++i) {
    assert(!reverse.gear[i]);
  }

  planning::HybridAStartResult turn;
  assert(planner.Plan(0.55, 0.75, 0.0, 0.95, 0.75, M_PI, &turn));
  bool switched = false;
  for (size_t i = 1; i + 1 < turn.gear.size(); ++i) {
    if (turn.gear[i] != turn.gear[i - 1]) {
      switched = true;
      assert(std::abs(turn.v[i]) < 1e-9);
    }
  }
  assert(switched);
  for (size_t i = 0; i + 1 < turn.x.size(); ++i) {
    const double dx = turn.x[i + 1] - turn.x[i];
    const double dy = turn.y[i + 1] - turn.y[i];
    const double projection = dx * std::cos(turn.phi[i]) +
                              dy * std::sin(turn.phi[i]);
    assert((turn.gear[i] ? projection : -projection) >= -1e-9);
  }

  planning::HybridAStartResult outside;
  assert(!planner.Plan(0.02, 0.75, 0.0, 1.0, 0.75, 0.0, &outside));

  const common::math::Box2d nearby_box(
      common::math::Vec2d(0.8235, 0.8765), 0.0, 0.10, 0.05);
  const common::math::Polygon2d nearby_obstacle(nearby_box);
  const common::math::Polygon2d start_body(vehicle.GenerateBox(
      common::math::Pose(0.75, 0.75, 0.0)));
  assert(!start_body.HasOverlap(nearby_obstacle));
  assert(start_body.DistanceTo(nearby_obstacle) < config.search_clearance);
  planning::HybridAStar clearance_planner(
      config, vehicle, {nearby_obstacle});
  planning::HybridAStartResult clearance_rejected;
  assert(!clearance_planner.Plan(0.75, 0.75, 0.0,
                                1.20, 0.75, 0.0,
                                &clearance_rejected));
}
