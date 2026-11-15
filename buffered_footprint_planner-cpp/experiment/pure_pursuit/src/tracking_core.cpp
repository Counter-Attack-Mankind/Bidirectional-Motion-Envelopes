#include "pure_pursuit/tracking_core.h"

#include <algorithm>
#include <cmath>

namespace pure_pursuit {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMotionEpsilon = 1e-4;
constexpr double kVelocityEvidence = 0.01;

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

TrackingPoint Interpolate(const TrackingPoint& a, const TrackingPoint& b,
                          double weight) {
  weight = Clamp(weight, 0.0, 1.0);
  TrackingPoint result;
  result.time = a.time + weight * (b.time - a.time);
  result.x = a.x + weight * (b.x - a.x);
  result.y = a.y + weight * (b.y - a.y);
  result.yaw = NormalizeAngle(
      a.yaw + weight * NormalizeAngle(b.yaw - a.yaw));
  result.velocity = a.velocity + weight * (b.velocity - a.velocity);
  result.acceleration =
      a.acceleration + weight * (b.acceleration - a.acceleration);
  result.s = a.s + weight * (b.s - a.s);
  return result;
}

double Cross(const Point2d& a, const Point2d& b, const Point2d& c) {
  return (b.x - a.x) * (c.y - a.y) -
         (b.y - a.y) * (c.x - a.x);
}

bool OnSegment(const Point2d& a, const Point2d& b, const Point2d& p) {
  return std::abs(Cross(a, b, p)) < 1e-10 &&
         p.x >= std::min(a.x, b.x) - 1e-10 &&
         p.x <= std::max(a.x, b.x) + 1e-10 &&
         p.y >= std::min(a.y, b.y) - 1e-10 &&
         p.y <= std::max(a.y, b.y) + 1e-10;
}

bool SegmentsIntersect(const Point2d& a, const Point2d& b,
                       const Point2d& c, const Point2d& d) {
  const double c1 = Cross(a, b, c);
  const double c2 = Cross(a, b, d);
  const double c3 = Cross(c, d, a);
  const double c4 = Cross(c, d, b);
  if (((c1 > 0.0 && c2 < 0.0) || (c1 < 0.0 && c2 > 0.0)) &&
      ((c3 > 0.0 && c4 < 0.0) || (c3 < 0.0 && c4 > 0.0)))
    return true;
  return OnSegment(a, b, c) || OnSegment(a, b, d) ||
         OnSegment(c, d, a) || OnSegment(c, d, b);
}

double PointSegmentDistance(const Point2d& p, const Point2d& a,
                            const Point2d& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double norm = dx * dx + dy * dy;
  if (norm < 1e-16) return std::hypot(p.x - a.x, p.y - a.y);
  const double t = Clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / norm,
                         0.0, 1.0);
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

bool PointInPolygon(const Point2d& p, const std::vector<Point2d>& polygon) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size();
       j = i++) {
    const Point2d& a = polygon[i];
    const Point2d& b = polygon[j];
    if (OnSegment(a, b, p)) return true;
    const bool crosses = ((a.y > p.y) != (b.y > p.y)) &&
        p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x;
    if (crosses) inside = !inside;
  }
  return inside;
}

}  // namespace

double NormalizeAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

bool TrackingPath::Build(std::vector<TrackingPoint> points,
                         std::string* error) {
  points_.clear();
  interval_gears_.clear();
  segments_.clear();
  if (points.size() < 2) {
    if (error) *error = "trajectory requires at least two points";
    return false;
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    const TrackingPoint& p = points[i];
    if (!std::isfinite(p.time) || !std::isfinite(p.x) ||
        !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
        !std::isfinite(p.velocity) || !std::isfinite(p.acceleration) ||
        (i == 0 && p.time < 0.0) ||
        (i > 0 && p.time <= points[i - 1].time)) {
      if (error) *error = "trajectory contains non-finite or non-monotonic data";
      return false;
    }
  }

  points[0].s = 0.0;
  interval_gears_.assign(points.size() - 1, 0);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double dx = points[i + 1].x - points[i].x;
    const double dy = points[i + 1].y - points[i].y;
    const double ds = std::hypot(dx, dy);
    points[i + 1].s = points[i].s + ds;

    const int original_gear = points[i].gear;
    if (original_gear != 1 && original_gear != -1) {
      if (error) *error = "interval " + std::to_string(i) +
          " has no valid original gear";
      return false;
    }
    const int start_velocity_gear =
        std::abs(points[i].velocity) > kVelocityEvidence
            ? (points[i].velocity > 0.0 ? 1 : -1) : 0;
    int geometry_gear = 0;

    if (ds > kMotionEpsilon) {
      const double tangent = std::atan2(dy, dx);
      const double midpoint_yaw = NormalizeAngle(
          points[i].yaw + 0.5 * NormalizeAngle(
              points[i + 1].yaw - points[i].yaw));
      const double alignment = std::cos(
          tangent - midpoint_yaw);
      if (std::abs(alignment) < 0.25) {
        if (error) *error = "path tangent is inconsistent with vehicle heading";
        return false;
      }
      geometry_gear = alignment > 0.0 ? 1 : -1;
    }
    if ((geometry_gear != 0 && geometry_gear != original_gear) ||
        (start_velocity_gear != 0 && start_velocity_gear != original_gear)) {
      if (error) {
        *error = "interval " + std::to_string(i) +
            " direction conflict: start_v=" +
            std::to_string(points[i].velocity) + " end_v=" +
            std::to_string(points[i + 1].velocity) + " original_gear=" +
            std::to_string(original_gear) + " geometry_gear=" +
            std::to_string(geometry_gear) +
            (geometry_gear != 0 && geometry_gear != original_gear
                 ? " geometric displacement disagrees with original gear"
                 : " Euler interval start velocity disagrees with original gear");
      }
      return false;
    }
    // BME uses forward Euler: interval i is driven by node i and its explicit
    // optimized interval gear. Node i+1 may cross zero before the next
    // interval and must never reclassify interval i.
    interval_gears_[i] = original_gear;
  }

  int begin = 0;
  int gear = interval_gears_[0];
  for (int i = 1; i < static_cast<int>(interval_gears_.size()); ++i) {
    if (interval_gears_[i] == gear) continue;
    MotionSegment segment;
    segment.point_begin = begin;
    segment.point_end = i;
    segment.gear = gear;
    segment.s_begin = points[begin].s;
    segment.s_end = points[i].s;
    segment.time_begin = points[begin].time;
    segment.time_end = points[i].time;
    segments_.push_back(segment);
    begin = i;
    gear = interval_gears_[i];
  }
  MotionSegment segment;
  segment.point_begin = begin;
  segment.point_end = static_cast<int>(points.size()) - 1;
  segment.gear = gear;
  segment.s_begin = points[begin].s;
  segment.s_end = points.back().s;
  segment.time_begin = points[begin].time;
  segment.time_end = points.back().time;
  segments_.push_back(segment);

  for (const MotionSegment& item : segments_) {
    if (item.s_end - item.s_begin < kMotionEpsilon) {
      if (error) *error = "trajectory contains a zero-length motion segment";
      segments_.clear();
      return false;
    }
  }
  points_ = std::move(points);
  return true;
}

TrackingPoint TrackingPath::SampleAtS(double s, int segment_index) const {
  const MotionSegment& segment = segments_.at(segment_index);
  s = Clamp(s, segment.s_begin, segment.s_end);
  for (int i = segment.point_begin; i < segment.point_end; ++i) {
    const TrackingPoint& a = points_[i];
    const TrackingPoint& b = points_[i + 1];
    if (b.s - a.s < kMotionEpsilon) continue;
    if (s <= b.s + 1e-9)
      return Interpolate(a, b, (s - a.s) / (b.s - a.s));
  }
  return points_[segment.point_end];
}

TrackingPoint TrackingPath::SampleAtTime(double time,
                                         int segment_index) const {
  const MotionSegment& segment = segments_.at(segment_index);
  time = Clamp(time, segment.time_begin, segment.time_end);
  for (int i = segment.point_begin; i < segment.point_end; ++i) {
    const TrackingPoint& a = points_[i];
    const TrackingPoint& b = points_[i + 1];
    if (time <= b.time + 1e-9)
      return Interpolate(a, b, (time - a.time) / (b.time - a.time));
  }
  return points_[segment.point_end];
}

ProjectionResult TrackingPath::Project(
    double x, double y, double body_yaw, double previous_s, double speed,
    double dt, int segment_index, double backward_window,
    double forward_window, bool motion_heading_valid,
    double motion_heading) const {
  ProjectionResult result;
  const MotionSegment& segment = segments_.at(segment_index);
  const double lo = std::max(segment.s_begin, previous_s - backward_window);
  const double hi = std::min(segment.s_end, previous_s + forward_window);
  const double expected = Clamp(previous_s + std::max(0.0, speed) * dt,
                                lo, hi);
  double best_score = std::numeric_limits<double>::infinity();
  for (int i = segment.point_begin; i < segment.point_end; ++i) {
    const TrackingPoint& a = points_[i];
    const TrackingPoint& b = points_[i + 1];
    const double ds = b.s - a.s;
    if (ds < kMotionEpsilon || b.s < lo || a.s > hi) continue;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double norm = dx * dx + dy * dy;
    double weight = ((x - a.x) * dx + (y - a.y) * dy) / norm;
    weight = Clamp(weight, 0.0, 1.0);
    double candidate_s = a.s + weight * ds;
    candidate_s = Clamp(candidate_s, lo, hi);
    const TrackingPoint candidate = SampleAtS(candidate_s, segment_index);
    const double distance = std::hypot(candidate.x - x, candidate.y - y);
    const double tangent = std::atan2(dy, dx);
    const double expected_motion_heading = motion_heading_valid
        ? motion_heading
        : body_yaw + (segment.gear < 0 ? kPi : 0.0);
    const double heading = std::abs(NormalizeAngle(
        expected_motion_heading - tangent));
    const double continuity = std::abs(candidate_s - expected);
    const double score = distance + 0.08 * heading / kPi +
                         0.50 * continuity;
    if (score < best_score) {
      best_score = score;
      result.valid = true;
      result.s = candidate_s;
      result.distance = distance;
      result.heading_error = heading;
      result.continuity_error = continuity;
      result.interval = i;
    }
  }
  return result;
}

std::vector<Point2d> VehicleFootprint(double rear_x, double rear_y,
                                      double yaw, double rear_to_center,
                                      double length, double width) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double center_x = rear_x + rear_to_center * c;
  const double center_y = rear_y + rear_to_center * s;
  const double hx = 0.5 * length;
  const double hy = 0.5 * width;
  std::vector<Point2d> result;
  for (const Point2d& local :
       std::vector<Point2d>{{hx, hy}, {hx, -hy}, {-hx, -hy}, {-hx, hy}}) {
    result.push_back({center_x + c * local.x - s * local.y,
                      center_y + s * local.x + c * local.y});
  }
  return result;
}

double PolygonDistance(const std::vector<Point2d>& lhs,
                       const std::vector<Point2d>& rhs) {
  if (lhs.size() < 3 || rhs.size() < 3)
    return std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    const Point2d& a = lhs[i];
    const Point2d& b = lhs[(i + 1) % lhs.size()];
    for (std::size_t j = 0; j < rhs.size(); ++j) {
      const Point2d& c = rhs[j];
      const Point2d& d = rhs[(j + 1) % rhs.size()];
      if (SegmentsIntersect(a, b, c, d)) return 0.0;
    }
  }
  if (PointInPolygon(lhs.front(), rhs) || PointInPolygon(rhs.front(), lhs))
    return 0.0;
  double distance = std::numeric_limits<double>::infinity();
  for (const Point2d& p : lhs)
    for (std::size_t j = 0; j < rhs.size(); ++j)
      distance = std::min(distance, PointSegmentDistance(
          p, rhs[j], rhs[(j + 1) % rhs.size()]));
  for (const Point2d& p : rhs)
    for (std::size_t j = 0; j < lhs.size(); ++j)
      distance = std::min(distance, PointSegmentDistance(
          p, lhs[j], lhs[(j + 1) % lhs.size()]));
  return distance;
}

}  // namespace pure_pursuit
