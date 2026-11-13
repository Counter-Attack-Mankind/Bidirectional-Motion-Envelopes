#include "scene/mocap_scene_provider.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <regex>
#include <set>
#include <utility>

#include <ros/master.h>
#include <boost/bind/bind.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "common/math/math_utils.h"

namespace scene {
namespace {
using boost::placeholders::_1;

bool FinitePose(const geometry_msgs::PoseStamped& pose) {
  const auto& p = pose.pose.position;
  const auto& q = pose.pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(q.x) &&
         std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

SceneObstacle MinimumAreaObb(const std::string& id,
                             const std::vector<common::math::Vec2d>& points,
                             double padding) {
  SceneObstacle best;
  best.id = id;
  double best_area = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& a = points[i];
    const auto& b = points[(i + 1) % points.size()];
    const double angle = std::atan2(b.y() - a.y(), b.x() - a.x());
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -min_u;
    double min_v = min_u;
    double max_v = -min_u;
    for (const auto& point : points) {
      const double u = c * point.x() + s * point.y();
      const double v = -s * point.x() + c * point.y();
      min_u = std::min(min_u, u);
      max_u = std::max(max_u, u);
      min_v = std::min(min_v, v);
      max_v = std::max(max_v, v);
    }
    const double length = max_u - min_u + 2.0 * padding;
    const double width = max_v - min_v + 2.0 * padding;
    const double area = length * width;
    if (area >= best_area) continue;
    best_area = area;
    const double center_u = 0.5 * (min_u + max_u);
    const double center_v = 0.5 * (min_v + max_v);
    best.x = c * center_u - s * center_v;
    best.y = s * center_u + c * center_v;
    best.yaw = common::math::NormalizeAngle(angle);
    best.length = length;
    best.width = width;
    if (best.width > best.length) {
      std::swap(best.width, best.length);
      best.yaw = common::math::NormalizeAngle(
          best.yaw + 1.57079632679489661923);
    }
    while (best.yaw >= 1.57079632679489661923)
      best.yaw -= 3.14159265358979323846;
    while (best.yaw < -1.57079632679489661923)
      best.yaw += 3.14159265358979323846;
  }
  return best;
}

std::vector<common::math::Vec2d> ConvexHull(
    std::vector<common::math::Vec2d> points) {
  std::sort(points.begin(), points.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.x() == rhs.x() ? lhs.y() < rhs.y() : lhs.x() < rhs.x();
  });
  points.erase(std::unique(points.begin(), points.end(), [](const auto& lhs,
                                                            const auto& rhs) {
                 return std::hypot(lhs.x() - rhs.x(), lhs.y() - rhs.y()) < 1e-9;
               }), points.end());
  if (points.size() < 3) return {};
  auto cross = [](const auto& o, const auto& a, const auto& b) {
    return (a.x() - o.x()) * (b.y() - o.y()) -
           (a.y() - o.y()) * (b.x() - o.x());
  };
  std::vector<common::math::Vec2d> hull(2 * points.size());
  std::size_t k = 0;
  for (const auto& point : points) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], point) <= 0.0) --k;
    hull[k++] = point;
  }
  const std::size_t lower = k + 1;
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (k >= lower && cross(hull[k - 2], hull[k - 1], *it) <= 0.0) --k;
    hull[k++] = *it;
  }
  hull.resize(k - 1);
  return hull;
}

}  // namespace

bool MocapSceneProvider::Initialize(ros::NodeHandle& node,
                                    const SceneSnapshot& static_scene) {
  static_scene_ = static_scene;
  static_scene_.obstacles.clear();
  node.param("mocap/vrpn_namespace", vrpn_namespace_, vrpn_namespace_);
  node.param("mocap/position_scale", position_scale_, position_scale_);
  node.param("mocap/map_rotation", rotation_, rotation_);
  node.param("mocap/map_translation_x", translation_x_, translation_x_);
  node.param("mocap/map_translation_y", translation_y_, translation_y_);
  node.param("mocap/vehicle_yaw_offset", yaw_offset_, yaw_offset_);
  node.param("mocap/body_to_rear_axle_x", body_to_rear_x_, body_to_rear_x_);
  node.param("mocap/body_to_rear_axle_y", body_to_rear_y_, body_to_rear_y_);
  node.param("mocap/data_timeout", data_timeout_, data_timeout_);
  node.param("mocap/geometry_retention", geometry_retention_,
             geometry_retention_);
  node.param("mocap/obstacle_padding", obstacle_padding_, obstacle_padding_);
  node.param("mocap/minimum_obstacle_markers", minimum_obstacle_markers_,
             minimum_obstacle_markers_);
  if (!(position_scale_ > 0.0) || !(data_timeout_ > 0.0) ||
      minimum_obstacle_markers_ < 3) {
    ROS_ERROR("Invalid mocap scale, timeout, or minimum marker count");
    return false;
  }
  discovery_timer_ = node.createTimer(ros::Duration(0.5),
      &MocapSceneProvider::DiscoverTopics, this);
  geometry_timer_ = node.createTimer(ros::Duration(0.02),
      [this](const ros::TimerEvent&) { RefreshGeometry(ros::Time::now()); });
  return true;
}

void MocapSceneProvider::DiscoverTopics(const ros::TimerEvent&) {
  ros::master::V_TopicInfo topics;
  if (!ros::master::getTopics(topics)) {
    ROS_WARN_THROTTLE(2.0, "Cannot query ROS master for VRPN topics");
    return;
  }
  const std::regex vehicle_pattern(".*/Vehicle([0-9]+)/pose$");
  const std::regex obstacle_pattern(
      ".*/Obstacle([0-9]+)_Marker([0-9]+)/pose$");
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& topic : topics) {
    if (topic.datatype != "geometry_msgs/PoseStamped" ||
        topic.name.rfind(vrpn_namespace_ + "/", 0) != 0 ||
        subscribers_.count(topic.name)) continue;
    std::smatch match;
    if (std::regex_match(topic.name, match, vehicle_pattern)) {
      const int id = std::stoi(match[1]);
      vehicles_[id];
      subscribers_[topic.name] = ros::NodeHandle().subscribe<geometry_msgs::PoseStamped>(
          topic.name, 1, boost::bind(&MocapSceneProvider::VehicleCallback,
                                    this, id, _1));
      ++revision_;
      ROS_INFO("Discovered mocap vehicle %d on %s", id, topic.name.c_str());
    } else if (std::regex_match(topic.name, match, obstacle_pattern)) {
      const int obstacle_id = std::stoi(match[1]);
      const int marker_id = std::stoi(match[2]);
      obstacles_[obstacle_id].markers[marker_id];
      subscribers_[topic.name] = ros::NodeHandle().subscribe<geometry_msgs::PoseStamped>(
          topic.name, 1, boost::bind(&MocapSceneProvider::MarkerCallback,
                                    this, obstacle_id, marker_id, _1));
      ++revision_;
      ROS_INFO("Discovered obstacle %d marker %d on %s", obstacle_id,
               marker_id, topic.name.c_str());
    }
  }
}

void MocapSceneProvider::VehicleCallback(
    int id, const geometry_msgs::PoseStamped::ConstPtr& message) {
  if (!message || !FinitePose(*message)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  auto& sample = vehicles_[id];
  sample.pose = *message;
  sample.received = ros::Time::now();
  sample.seen = true;
}

void MocapSceneProvider::MarkerCallback(
    int obstacle_id, int marker_id,
    const geometry_msgs::PoseStamped::ConstPtr& message) {
  if (!message || !FinitePose(*message)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  auto& sample = obstacles_[obstacle_id].markers[marker_id];
  sample.pose = *message;
  sample.received = ros::Time::now();
  sample.seen = true;
}

common::math::Vec2d MocapSceneProvider::TransformPoint(double x,
                                                       double y) const {
  x *= position_scale_;
  y *= position_scale_;
  const double c = std::cos(rotation_);
  const double s = std::sin(rotation_);
  return {c * x - s * y + translation_x_,
          s * x + c * y + translation_y_};
}

Pose2d MocapSceneProvider::TransformVehicle(
    const geometry_msgs::PoseStamped& pose) const {
  const auto point = TransformPoint(pose.pose.position.x, pose.pose.position.y);
  const double yaw = common::math::NormalizeAngle(
      tf2::getYaw(pose.pose.orientation) + rotation_ + yaw_offset_);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {point.x() + c * body_to_rear_x_ - s * body_to_rear_y_,
          point.y() + s * body_to_rear_x_ + c * body_to_rear_y_, yaw};
}

void MocapSceneProvider::RefreshGeometry(const ros::Time& now) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& entry : obstacles_) {
    std::vector<common::math::Vec2d> points;
    for (const auto& marker : entry.second.markers) {
      const auto& sample = marker.second;
      if (!sample.seen || (now - sample.received).toSec() > data_timeout_) continue;
      points.push_back(TransformPoint(sample.pose.pose.position.x,
                                      sample.pose.pose.position.y));
    }
    if (points.size() < static_cast<std::size_t>(minimum_obstacle_markers_))
      continue;
    const auto hull = ConvexHull(std::move(points));
    if (hull.size() < 3) continue;
    SceneObstacle geometry = MinimumAreaObb(
        "obstacle_" + std::to_string(entry.first), hull, obstacle_padding_);
    if (geometry.length <= 1e-4 || geometry.width <= 1e-4) continue;
    const bool changed = !entry.second.has_geometry ||
        std::hypot(geometry.x - entry.second.last_geometry.x,
                   geometry.y - entry.second.last_geometry.y) > 1e-4 ||
        std::abs(common::math::NormalizeAngle(
            geometry.yaw - entry.second.last_geometry.yaw)) > 1e-3 ||
        std::abs(geometry.length - entry.second.last_geometry.length) > 1e-4 ||
        std::abs(geometry.width - entry.second.last_geometry.width) > 1e-4;
    entry.second.last_geometry = geometry;
    entry.second.last_geometry_time = now;
    entry.second.has_geometry = true;
    if (changed) ++revision_;
  }
}

MocapSceneProvider::Status MocapSceneProvider::StatusLocked(
    const ros::Time& now) const {
  Status status;
  status.revision = revision_;
  int vehicle_id = -1;
  int fresh_vehicle_count = 0;
  for (const auto& entry : vehicles_) {
    if (entry.second.seen &&
        (now - entry.second.received).toSec() <= data_timeout_) {
      vehicle_id = entry.first;
      ++fresh_vehicle_count;
    }
  }
  if (fresh_vehicle_count != 1) vehicle_id = -1;
  status.vehicle_id = vehicle_id;
  const auto vehicle = vehicles_.find(vehicle_id);
  status.vehicle_fresh = vehicle != vehicles_.end() && vehicle->second.seen &&
      (now - vehicle->second.received).toSec() <= data_timeout_;
  bool all_fresh = !obstacles_.empty();
  for (const auto& obstacle : obstacles_) {
    all_fresh = all_fresh && obstacle.second.has_geometry &&
        (now - obstacle.second.last_geometry_time).toSec() <= data_timeout_;
  }
  status.obstacle_count = obstacles_.size();
  status.obstacles_fresh = all_fresh;
  status.ready = status.vehicle_fresh && status.obstacles_fresh;
  if (vehicles_.empty()) status.message = "waiting for VehicleN/pose topic";
  else if (fresh_vehicle_count > 1) status.message =
      "multiple fresh VehicleN topics found; exactly one vehicle is required";
  else if (vehicle_id < 0) status.message = "vehicle pose is stale";
  else if (!status.vehicle_fresh) status.message = "vehicle pose is stale";
  else if (obstacles_.empty()) status.message =
      "waiting for ObstacleN_MarkerM topics";
  else if (!status.obstacles_fresh) {
    bool beyond_retention = false;
    for (const auto& obstacle : obstacles_) {
      beyond_retention = beyond_retention ||
          (obstacle.second.has_geometry &&
           (now - obstacle.second.last_geometry_time).toSec() >
               geometry_retention_);
    }
    status.message = beyond_retention
        ? "obstacle stale; last geometry shown for safety only"
        : "obstacle markers are incomplete or stale";
  }
  else status.message = "mocap scene ready";
  return status;
}

MocapSceneProvider::Status MocapSceneProvider::GetStatus() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return StatusLocked(ros::Time::now());
}

bool MocapSceneProvider::Snapshot(SceneSnapshot* snapshot,
                                  Status* status) const {
  if (!snapshot) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  const ros::Time now = ros::Time::now();
  const Status current = StatusLocked(now);
  if (status) *status = current;
  *snapshot = static_scene_;
  const auto vehicle = vehicles_.find(current.vehicle_id);
  if (vehicle != vehicles_.end() && vehicle->second.seen)
    snapshot->start = TransformVehicle(vehicle->second.pose);
  snapshot->obstacles.clear();
  for (const auto& obstacle : obstacles_) {
    if (!obstacle.second.has_geometry) continue;
    // Retain the last known geometry through short occlusions.  Readiness is
    // false while stale, so retained obstacles remain visible but motion is
    // inhibited instead of silently treating them as absent.
    snapshot->obstacles.push_back(obstacle.second.last_geometry);
  }
  return current.ready;
}

}  // namespace scene
