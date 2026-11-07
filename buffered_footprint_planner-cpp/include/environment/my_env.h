#pragma once

#include <string>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/math/vec2d.h"

using json = nlohmann::json;

namespace common {
namespace math {

inline void to_json(json& j, const Vec2d& point) {
  j = json{point.x(), point.y()};
}

inline void from_json(const json& j, Vec2d& point) {
  if (!j.is_array() || j.size() != 2) {
    throw std::runtime_error("a 2D point must be [x, y]");
  }
  point.set_x(j.at(0).get<double>());
  point.set_y(j.at(1).get<double>());
}

inline void to_json(json& j, const Pose& pose) {
  j = json{pose.x(), pose.y(), pose.theta()};
}

inline void from_json(const json& j, Pose& pose) {
  if (!j.is_array() || j.size() != 3) {
    throw std::runtime_error("a pose must be [x, y, theta]");
  }
  pose.set_x(j.at(0).get<double>());
  pose.set_y(j.at(1).get<double>());
  pose.set_theta(j.at(2).get<double>());
}

inline void to_json(json& j, const Polygon2d& polygon) {
  j = polygon.points();
}

inline void from_json(const json& j, Polygon2d& polygon) {
  const auto points = j.get<std::vector<Vec2d>>();
  if (points.size() < 3) {
    throw std::runtime_error("an obstacle needs at least three vertices");
  }
  polygon = Polygon2d(points);
}

}  // namespace math
}  // namespace common

struct MyEnvironment {
  std::vector<common::math::Vec2d> border;
  std::vector<common::math::Polygon2d> obstacles;
  common::math::Pose start;
  common::math::Pose goal;
  bool has_start = false;
  bool has_goal = false;

  bool Read(const std::string& env_file);
  bool Save(const std::string& env_file) const;
};

inline void to_json(json& j, const MyEnvironment& environment) {
  j = json{{"border", environment.border}, {"obstacles", environment.obstacles}};
  if (environment.has_start) j["start"] = environment.start;
  if (environment.has_goal) j["goal"] = environment.goal;
}

inline void from_json(const json& j, MyEnvironment& environment) {
  j.at("border").get_to(environment.border);
  j.at("obstacles").get_to(environment.obstacles);
  if (environment.border.size() < 3) {
    throw std::runtime_error("border needs at least three points");
  }

  environment.has_start = j.contains("start");
  environment.has_goal = j.contains("goal");
  if (environment.has_start) j.at("start").get_to(environment.start);
  if (environment.has_goal) j.at("goal").get_to(environment.goal);
}
