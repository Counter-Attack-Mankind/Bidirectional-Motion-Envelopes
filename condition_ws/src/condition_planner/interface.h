#pragma once

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "common/math/pose.h"
#include "hybrid_astar/hybrid_a_star.h"
#include "trajectory_nlp/trajectory_optimizer.h"

#include "common/trajectory_point.h"
#include "my_env.h"

using namespace common::math;

inline std::vector<Vec2d> PointsToVec(const std::vector<geometry_msgs::Point> &points) {
  std::vector<Vec2d> tmp(points.size());
  for(size_t i = 0; i < points.size(); i++) {
    tmp[i].set_x(points[i].x);
    tmp[i].set_y(points[i].y);
  }
  return tmp;
}

class ConditionPlanner {
public:
  std::mutex trajectory_mutex;
  ConditionPlanner();

  void ReadConfig();

  void set_start_pose(double x, double y, double theta) {
    start_pose_ = Pose(x, y, theta);
  }

  const Pose &start_pose() const { return start_pose_; }

  void set_goal_pose(double x, double y, double theta) {
    goal_pose_ = Pose(x, y, theta);
  }

  const Pose &goal_pose() const { return goal_pose_; }

  Env env() { return env_; }

  // 加载环境 (border + obstacles)
  void LoadEnvironment(const MyEnvironment &my_env);

  // 规划: Hybrid A*(仅前进) → LIOM优化（版本1）
  bool Plan(Trajectory *result);

  // 规划: Hybrid A*(仅前进) → DP曲率限速配速 → ECC一次性NLP（版本2，条件分支约束）
  bool PlanECC(Trajectory *result);

private:
  bool LoadLiomTrajectoryFromCsv(const std::string &csv_file, trajectory_nlp::States &states) const;

  Env env_;
  ParkingPlannerConfig planning_config_;
  TrajectoryNLPConfig nlp_config_;
  trajectory_nlp::DpSVConfig dp_sv_config_;
  std::shared_ptr<planning::HybridAStar> planner_;
  std::shared_ptr<trajectory_nlp::TrajectoryOptimizer> optimizer_;

  ros::NodeHandle nh_;
  Pose start_pose_, goal_pose_;
  std::string liom_csv_file_;
};
