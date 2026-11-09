#pragma once

#include <array>
#include <string>
#include <vector>

#include "common/math/polygon2d.h"
#include "hybrid_astar/vehicle_parameter.h"

namespace scene {

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct SceneObstacle {
  std::string id;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double length = 0.0;
  double width = 0.0;

  common::math::Polygon2d polygon() const;
};

struct SceneSnapshot {
  int task_id = 0;
  std::string frame_id = "map";
  std::array<double, 4> bounds{{0.0, 0.0, 0.0, 0.0}};
  Pose2d start;
  Pose2d goal;
  std::vector<SceneObstacle> obstacles;

  std::vector<common::math::Vec2d> boundary() const;
  std::vector<common::math::Polygon2d> obstacle_polygons() const;
  bool Validate(const VehicleParameter& vehicle, std::string* error) const;
};

}  // namespace scene
