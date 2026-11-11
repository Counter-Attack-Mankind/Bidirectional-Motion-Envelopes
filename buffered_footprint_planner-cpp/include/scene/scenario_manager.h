#pragma once

#include <array>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "common/math/polygon2d.h"
#include "scene/scene_snapshot.h"

class ScenarioManager {
 public:
  using Obstacle = scene::SceneObstacle;

  bool Load(const ros::NodeHandle& node);
  bool SetObstaclePose(const std::string& id, double x, double y, double yaw);

  const std::string& frame_id() const { return frame_id_; }
  const std::array<double, 4>& bounds() const { return bounds_; }
  const std::vector<common::math::Vec2d>& boundary() const { return boundary_; }
  const scene::Pose2d& goal() const { return goal_; }
  const std::vector<Obstacle>& obstacles() const { return obstacles_; }
  std::vector<common::math::Polygon2d> obstacle_polygons() const;
  scene::SceneSnapshot Snapshot(const scene::Pose2d& start,
                                int task_id = 0) const;

 private:
  std::string frame_id_ = "map";
  std::array<double, 4> bounds_{{0.0, 4.5, 0.0, 2.5}};
  std::vector<common::math::Vec2d> boundary_;
  scene::Pose2d goal_;
  std::vector<Obstacle> obstacles_;
};
