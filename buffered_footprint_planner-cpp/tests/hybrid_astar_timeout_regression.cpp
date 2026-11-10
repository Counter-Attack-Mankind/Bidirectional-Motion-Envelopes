#include <chrono>
#include <cstdlib>
#include <iostream>

#include "hybrid_astar/hybrid_a_star.h"

int main() {
  ParkingPlannerConfig config;
  config.xy_bounds = {0.0, 30.0, 0.0, 30.0};
  config.grid_a_star_xy_resolution = 0.01;
  config.xy_grid_resolution = 0.02;
  config.explore_node_num = 2000000;

  VehicleParameter vehicle;
  vehicle.GenerateDisc();
  planning::HybridAStar planner(config, vehicle, {});
  planning::HybridAStartResult result;
  const auto begin = std::chrono::steady_clock::now();
  const bool succeeded = planner.Plan(1.0, 1.0, 0.0,
                                      29.0, 29.0, 0.0, &result);
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - begin).count();
  std::cout << "succeeded=" << succeeded
            << " timed_out=" << planner.timed_out()
            << " elapsed=" << elapsed << '\n';
  if (succeeded || !planner.timed_out()) return EXIT_FAILURE;
  // Scheduling jitter is allowed, but the algorithm must return near its
  // internal 8 s deadline rather than run to completion in the background.
  return elapsed >= 7.4 && elapsed <= 8.0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
