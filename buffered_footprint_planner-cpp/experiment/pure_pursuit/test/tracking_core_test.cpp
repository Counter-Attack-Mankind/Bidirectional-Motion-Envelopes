#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "pure_pursuit/tracking_core.h"

namespace {

int Fail(const std::string& message) {
  std::cerr << "tracking_core_test: " << message << "\n";
  return 1;
}

pure_pursuit::TrackingPoint P(double t, double x, double y, double yaw,
                              double velocity, int gear = 1) {
  pure_pursuit::TrackingPoint point;
  point.time = t;
  point.x = x;
  point.y = y;
  point.yaw = yaw;
  point.velocity = velocity;
  point.gear = gear;
  return point;
}

}  // namespace

int main() {
  using pure_pursuit::TrackingPath;
  std::string error;

  TrackingPath forward;
  if (!forward.Build({P(0, 0, 0, 0, 0), P(1, 1, 0, 0, 0.2)}, &error) ||
      forward.segments().size() != 1 || forward.segment(0).gear != 1)
    return Fail("pure forward trajectory classification failed: " + error);

  TrackingPath reverse;
  if (!reverse.Build({P(0, 1, 0, 0, 0, -1), P(1, 0, 0, 0, -0.2, -1)}, &error) ||
      reverse.segment(0).gear != -1)
    return Fail("pure reverse trajectory classification failed: " + error);

  TrackingPath cusp;
  if (!cusp.Build({P(0, 0, 0, 0, 0.1), P(1, 1, 0, 0, 0),
                   P(2, 1, 0, 0, 0, -1), P(3, 0, 0, 0, -0.1, -1)}, &error) ||
      cusp.segments().size() != 2 || cusp.segment(0).gear != 1 ||
      cusp.segment(1).gear != -1)
    return Fail("forward-to-reverse cusp classification failed: " + error);

  TrackingPath reverse_forward;
  if (!reverse_forward.Build(
          {P(0, 1, 0, 0, -0.1, -1), P(1, 0, 0, 0, 0, -1),
           P(2, 0, 0, 0, 0), P(3, 1, 0, 0, 0.1)}, &error) ||
      reverse_forward.segments().size() != 2 ||
      reverse_forward.segment(0).gear != -1 ||
      reverse_forward.segment(1).gear != 1)
    return Fail("reverse-to-forward cusp classification failed: " + error);

  TrackingPath multiple_cusps;
  if (!multiple_cusps.Build(
          {P(0, 0, 0, 0, 0.1), P(1, 1, 0, 0, 0, -1),
           P(2, 0, 0, 0, 0, 1), P(3, 1, 0, 0, 0.1)}, &error) ||
      multiple_cusps.segments().size() != 3 ||
      multiple_cusps.segment(0).gear != 1 ||
      multiple_cusps.segment(1).gear != -1 ||
      multiple_cusps.segment(2).gear != 1)
    return Fail("multiple cusp classification failed: " + error);

  TrackingPath conflict;
  if (conflict.Build({P(0, 0, 0, 0, -0.2), P(1, 1, 0, 0, -0.2)},
                     &error))
    return Fail("velocity/geometry direction conflict was accepted");

  // Actual field failure: forward-Euler interval starts positive, ends
  // negative, but explicit interval gear and geometric displacement are
  // forward. The end velocity belongs to the next Euler interval.
  TrackingPath crossing_zero;
  if (!crossing_zero.Build(
          {P(0, 0, 0, 0, 0.003649, 1),
           P(0.1, 0.0003649, 0, 0, -0.011802, -1),
           P(0.2, -0.0008153, 0, 0, -0.02, -1)}, &error) ||
      crossing_zero.segment(0).gear != 1 ||
      crossing_zero.segment(1).gear != -1)
    return Fail("forward-Euler cross-zero interval regression failed: " + error);

  TrackingPath near_zero_motion;
  if (!near_zero_motion.Build(
          {P(0, 0, 0, 0, 0, 1), P(0.1, 1e-6, 0, 0, 0, 1),
           P(0.2, 0.01, 0, 0, 0.05, 1)}, &error))
    return Fail("near-zero displacement interval was rejected: " + error);

  // Overlapping return branch is geometrically closer, but projection is
  // constrained to the current segment and a local arc-length window.
  const auto projection = cusp.Project(0.48, 0.01, 0.0, 0.40, 0.1, 0.1,
                                       0, 0.02, 0.30, true, 0.0);
  if (!projection.valid || projection.s < 0.38 || projection.s > 0.70)
    return Fail("local continuous projection jumped or regressed");

  TrackingPath wrapped_yaw;
  if (!wrapped_yaw.Build(
          {P(0, 0, 0, 3.13, 0.1), P(1, -1, 0, -3.13, 0.1)}, &error))
    return Fail("wrapped-yaw path was rejected: " + error);
  const auto yaw_wrap = wrapped_yaw.SampleAtTime(0.5, 0);
  if (std::abs(yaw_wrap.yaw) < 3.0)
    return Fail("yaw interpolation did not use the shortest angle");

  const auto vehicle = pure_pursuit::VehicleFootprint(
      0.0, 0.0, 0.0, 0.0735, 0.211, 0.191);
  const std::vector<pure_pursuit::Point2d> near_obstacle{
      {0.18, -0.05}, {0.28, -0.05}, {0.28, 0.05}, {0.18, 0.05}};
  const std::vector<pure_pursuit::Point2d> far_obstacle{
      {1.0, -0.05}, {1.1, -0.05}, {1.1, 0.05}, {1.0, 0.05}};
  if (pure_pursuit::PolygonDistance(vehicle, near_obstacle) > 0.05 ||
      pure_pursuit::PolygonDistance(vehicle, far_obstacle) < 0.5)
    return Fail("vehicle/obstacle clearance calculation failed");

  std::cout << "tracking_core_test: PASS\n";
  return 0;
}
