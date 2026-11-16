#include <cassert>
#include <string>

#include "visualize/body_history_visualizer.h"
#include "visualize/swept_area_visualizer.h"

int main() {
  VehicleParameter vehicle;
  vehicle.wheel_base = 0.143;
  vehicle.front_hang = 0.036;
  vehicle.rear_hang = 0.032;
  vehicle.width = 0.191;
  vehicle.GenerateDisc();
  std::string error;

  visualization::BodyHistoryVisualizer history;
  assert(history.Initialize(vehicle, 0.005, &error));
  optimizer::TrajectoryNode first;
  history.Reset(first, true);
  optimizer::TrajectoryNode second = first;
  second.x = 0.010;
  history.Update(second, true);
  assert(history.sample_count() == 3);
  optimizer::TrajectoryNode rotated = second;
  rotated.theta = 0.1;
  history.Update(rotated, false);
  assert(history.sample_count() > 3);

  visualization::SweptAreaVisualizer sweep;
  assert(sweep.Initialize(vehicle, 0.02, &error));
  sweep.Reset(first, true);
  const std::size_t one_direction = sweep.cell_count();
  assert(one_direction > 0);
  sweep.Update(first, true);
  assert(sweep.cell_count() == one_direction);
  sweep.Update(first, false);
  assert(sweep.cell_count() == 2 * one_direction);
  return 0;
}
