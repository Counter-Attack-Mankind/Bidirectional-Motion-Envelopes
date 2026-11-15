#pragma once

#include <limits>
#include <string>
#include <vector>

namespace pure_pursuit {

struct TrackingPoint {
  double time = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  int gear = 0;
  double s = 0.0;
};

struct MotionSegment {
  int point_begin = 0;
  int point_end = 0;  // Inclusive; adjacent segments share the cusp point.
  int gear = 1;
  double s_begin = 0.0;
  double s_end = 0.0;
  double time_begin = 0.0;
  double time_end = 0.0;
};

struct ProjectionResult {
  bool valid = false;
  double s = 0.0;
  double distance = std::numeric_limits<double>::infinity();
  double heading_error = std::numeric_limits<double>::infinity();
  double continuity_error = std::numeric_limits<double>::infinity();
  int interval = -1;
};

class TrackingPath {
 public:
  bool Build(std::vector<TrackingPoint> points, std::string* error);

  bool empty() const { return points_.empty(); }
  const std::vector<TrackingPoint>& points() const { return points_; }
  const std::vector<MotionSegment>& segments() const { return segments_; }
  const MotionSegment& segment(int index) const { return segments_.at(index); }

  TrackingPoint SampleAtS(double s, int segment_index) const;
  TrackingPoint SampleAtTime(double time, int segment_index) const;
  ProjectionResult Project(double x, double y, double body_yaw,
                           double previous_s, double speed, double dt,
                           int segment_index, double backward_window,
                           double forward_window,
                           bool motion_heading_valid = false,
                           double motion_heading = 0.0) const;

 private:
  std::vector<TrackingPoint> points_;
  std::vector<int> interval_gears_;
  std::vector<MotionSegment> segments_;
};

struct Point2d {
  Point2d() = default;
  Point2d(double x_value, double y_value) : x(x_value), y(y_value) {}
  double x = 0.0;
  double y = 0.0;
};

std::vector<Point2d> VehicleFootprint(double rear_x, double rear_y,
                                      double yaw, double rear_to_center,
                                      double length, double width);
double PolygonDistance(const std::vector<Point2d>& lhs,
                       const std::vector<Point2d>& rhs);
double NormalizeAngle(double angle);

}  // namespace pure_pursuit
