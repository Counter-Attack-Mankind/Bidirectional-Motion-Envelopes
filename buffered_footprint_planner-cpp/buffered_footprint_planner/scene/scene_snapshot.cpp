#include "scene/scene_snapshot.h"

#include <cmath>
#include <unordered_set>

#include "common/math/box2d.h"
#include "common/math/pose.h"

namespace scene {
namespace {

bool FinitePose(const Pose2d& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

bool PolygonInside(const common::math::Polygon2d& polygon,
                   const std::array<double, 4>& bounds) {
  return polygon.min_x() >= bounds[0] && polygon.max_x() <= bounds[1] &&
         polygon.min_y() >= bounds[2] && polygon.max_y() <= bounds[3];
}

bool ValidateVehiclePose(const char* name, const Pose2d& pose,
                         const VehicleParameter& vehicle,
                         const std::array<double, 4>& bounds,
                         const std::vector<SceneObstacle>& obstacles,
                         std::string* error) {
  if (!FinitePose(pose)) {
    if (error) *error = std::string(name) + " pose contains non-finite values";
    return false;
  }
  const common::math::Polygon2d footprint(vehicle.GenerateBox(
      common::math::Pose(pose.x, pose.y, pose.yaw)));
  if (!PolygonInside(footprint, bounds)) {
    if (error) *error = std::string(name) + " vehicle footprint leaves map bounds";
    return false;
  }
  for (const auto& obstacle : obstacles) {
    if (footprint.HasOverlap(obstacle.polygon())) {
      if (error) *error = std::string(name) + " vehicle collides with obstacle '" +
                          obstacle.id + "'";
      return false;
    }
  }
  return true;
}

}  // namespace

common::math::Polygon2d SceneObstacle::polygon() const {
  return common::math::Polygon2d(
      common::math::Box2d({x, y}, yaw, length, width));
}

std::vector<common::math::Vec2d> SceneSnapshot::boundary() const {
  return {{bounds[0], bounds[2]}, {bounds[1], bounds[2]},
          {bounds[1], bounds[3]}, {bounds[0], bounds[3]}};
}

std::vector<common::math::Polygon2d> SceneSnapshot::obstacle_polygons() const {
  std::vector<common::math::Polygon2d> result;
  result.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) result.push_back(obstacle.polygon());
  return result;
}

bool SceneSnapshot::Validate(const VehicleParameter& vehicle,
                             std::string* error) const {
  for (double value : bounds) {
    if (!std::isfinite(value)) {
      if (error) *error = "Map bounds contain non-finite values";
      return false;
    }
  }
  if (frame_id.empty() || bounds[0] >= bounds[1] || bounds[2] >= bounds[3]) {
    if (error) *error = "Frame ID or map bounds are invalid";
    return false;
  }
  if (!(vehicle.wheel_base > 0.0) || vehicle.front_hang < 0.0 ||
      vehicle.rear_hang < 0.0 || !(vehicle.width > 0.0)) {
    if (error) *error = "Vehicle geometry is invalid";
    return false;
  }
  std::unordered_set<std::string> ids;
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const auto& obstacle = obstacles[i];
    if (obstacle.id.empty() || !ids.insert(obstacle.id).second) {
      if (error) *error = "Obstacle IDs must be non-empty and unique";
      return false;
    }
    if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
        !std::isfinite(obstacle.yaw) || !std::isfinite(obstacle.length) ||
        !std::isfinite(obstacle.width) || obstacle.length <= 0.0 ||
        obstacle.width <= 0.0 || !PolygonInside(obstacle.polygon(), bounds)) {
      if (error) *error = "Obstacle '" + obstacle.id + "' is invalid or outside map bounds";
      return false;
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (obstacle.polygon().HasOverlap(obstacles[j].polygon())) {
        if (error) *error = "Obstacles '" + obstacle.id + "' and '" +
                            obstacles[j].id + "' overlap";
        return false;
      }
    }
  }
  return ValidateVehiclePose("Start", start, vehicle, bounds, obstacles, error) &&
         ValidateVehiclePose("Goal", goal, vehicle, bounds, obstacles, error);
}

}  // namespace scene
