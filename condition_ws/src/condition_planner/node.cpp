//
// Condition planner node
// 从env.json加载场景，自动规划 Hybrid A*(仅前进) → LIOM
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

#include "common/math/pose.h"
#include "common/math/vec2d.h"
#include "common/math/polygon2d.h"
#include "common/util/time.h"
#include "visualization_plot.h"
#include "interface.h"
#include "my_env.h"

using namespace common::math;

class ConditionPlannerNode {
public:
  ConditionPlannerNode(): nh_("~") {
    VisualizationPlot::Init(nh_, "world", "/condition_planner_markers");

    // 加载环境文件
    env_file_ = nh_.param<std::string>("env_file", "");
    ROS_INFO("Environment file: '%s'", env_file_.c_str());

    if (env_.Read(env_file_)) {
      planner_.LoadEnvironment(env_);

      // 从env.json读取start/goal (Pose类型)
      if (std::abs(env_.start.x()) > 1e-9 || std::abs(env_.start.y()) > 1e-9) {
        planner_.set_start_pose(env_.start.x(), env_.start.y(), env_.start.theta());
        has_start_ = true;
        ROS_INFO("Start from JSON: (%.3f, %.3f, %.3f rad)",
                 env_.start.x(), env_.start.y(), env_.start.theta());
      }
      if (std::abs(env_.goal.x()) > 1e-9 || std::abs(env_.goal.y()) > 1e-9) {
        planner_.set_goal_pose(env_.goal.x(), env_.goal.y(), env_.goal.theta());
        has_goal_ = true;
        ROS_INFO("Goal from JSON: (%.3f, %.3f, %.3f rad)",
                 env_.goal.x(), env_.goal.y(), env_.goal.theta());
      }

      ROS_INFO("Environment loaded: %zu border points, %zu obstacles",
               env_.points.size(), env_.obstacles.size());
    } else {
      ROS_ERROR("Failed to load environment from '%s'", env_file_.c_str());
    }

    // RViz交互 (可选: 覆盖start/goal)
    pose_sub_ = nh_.subscribe("/initialpose", 1, &ConditionPlannerNode::StartPoseCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ConditionPlannerNode::GoalPoseCallback, this);

    // 延迟2秒等RViz订阅marker topic后再可视化和规划
    startup_timer_ = nh_.createTimer(ros::Duration(2.0),
        &ConditionPlannerNode::OnStartup, this, /*oneshot=*/true);

    ROS_INFO("Condition Planner Node Ready, waiting 2s for RViz...");
  }

  void OnStartup(const ros::TimerEvent&) {
    VisualizeEnvironment();
    if (has_start_ && has_goal_) {
      ROS_INFO("Auto-planning...");
      PlanAndVisualize();
    } else {
      ROS_INFO("Use RViz to set start (2D Pose Estimate) and goal (2D Nav Goal)");
    }
  }

  void StartPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = tf::getYaw(msg->pose.pose.orientation);

    planner_.set_start_pose(x, y, theta);
    has_start_ = true;
    ROS_INFO("Start pose: (%.3f, %.3f, %.3f rad)", x, y, theta);

    auto box = planner_.env()->vehicle.GenerateBox(Pose(x, y, theta));
    VisualizationPlot::PlotPolygon(Polygon2d(box), 0.02, Color::Blue, 1000, "Start");
    VisualizationPlot::Trigger();

    if (has_goal_) PlanAndVisualize();
  }

  void GoalPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double theta = tf::getYaw(msg->pose.orientation);

    planner_.set_goal_pose(x, y, theta);
    has_goal_ = true;
    ROS_INFO("Goal pose: (%.3f, %.3f, %.3f rad)", x, y, theta);

    auto box = planner_.env()->vehicle.GenerateBox(Pose(x, y, theta));
    VisualizationPlot::PlotPolygon(Polygon2d(box), 0.02, Color::Cyan, 1001, "Goal");
    VisualizationPlot::Trigger();

    if (has_start_) PlanAndVisualize();
  }

  void PlanAndVisualize() {
    bool use_ecc = nh_.param<bool>("use_ecc", false);

    Trajectory result;
    double t0 = common::util::GetCurrentTimestamp();
    bool success = false;

    if (use_ecc) {
      ROS_INFO("Planning Mode: ECC (Version 2: DP curvature speed + one-shot NLP with pair1/pair2)");
      success = planner_.PlanECC(&result);
    } else {
      ROS_INFO("Planning Mode: LIOM (Version 1: Hybrid A* -> iterative LIOM)");
      success = planner_.Plan(&result);
    }

    if (success) {
      double elapsed = common::util::GetCurrentTimestamp() - t0;
      ROS_INFO("SUCCESS! %zu points, %.2f s duration, %.3f s compute",
               result.size(),
               result.empty() ? 0 : result.back().t,
               elapsed);

      // 可视化最终轨迹
      std::vector<double> x, y;
      for (const auto& pt : result) {
        x.push_back(pt.x);
        y.push_back(pt.y);
      }
      VisualizationPlot::Plot(x, y, 0.02, Color::Green, 2, "Optimized Trajectory");

      // 可视化起终点
      auto start_box = planner_.env()->vehicle.GenerateBox(planner_.start_pose());
      auto goal_box  = planner_.env()->vehicle.GenerateBox(planner_.goal_pose());
      VisualizationPlot::PlotPolygon(Polygon2d(start_box), 0.02, Color::Blue, 1000, "Start");
      VisualizationPlot::PlotPolygon(Polygon2d(goal_box),  0.02, Color::Cyan, 1001, "Goal");
      VisualizationPlot::Trigger();

      // 保存轨迹（版本1 → trajectory_v1.csv，版本2 → trajectory_v2.csv）
      SaveTrajectory(result, use_ecc ? 2 : 1);
    } else {
      ROS_ERROR("Planning FAILED!");
    }
  }

  void VisualizeEnvironment() {
    // 可视化border
    if (!env_.points.empty()) {
      std::vector<double> bx, by;
      for (const auto& p : env_.points) {
        bx.push_back(p.x());
        by.push_back(p.y());
      }
      VisualizationPlot::Plot(bx, by, 0.01, Color::White, 100, "Border");
    }
    // 可视化obstacles(实心填充)
    for (size_t i = 0; i < env_.obstacles.size(); i++) {
      VisualizationPlot::PlotFilledPolygon(env_.obstacles[i], Color::Red,
                                            200 + i, "Obstacle_" + std::to_string(i));
    }
    // 起终点车身 + 朝向箭头
    if (has_start_) {
      auto sp = planner_.start_pose();
      auto box = planner_.env()->vehicle.GenerateBox(sp);
      VisualizationPlot::PlotPolygon(Polygon2d(box), 0.005, Color::Blue, 1000, "Start");
      // 朝向箭头
      double arr = 0.15;
      VisualizationPlot::Plot(
          {sp.x(), sp.x() + arr * cos(sp.theta())},
          {sp.y(), sp.y() + arr * sin(sp.theta())},
          0.008, Color::Blue, 1002, "StartHeading");
    }
    if (has_goal_) {
      auto gp = planner_.goal_pose();
      auto box = planner_.env()->vehicle.GenerateBox(gp);
      VisualizationPlot::PlotPolygon(Polygon2d(box), 0.005, Color::Cyan, 1001, "Goal");
      double arr = 0.15;
      VisualizationPlot::Plot(
          {gp.x(), gp.x() + arr * cos(gp.theta())},
          {gp.y(), gp.y() + arr * sin(gp.theta())},
          0.008, Color::Cyan, 1003, "GoalHeading");
    }
    VisualizationPlot::Trigger();
  }

  void SaveTrajectory(const Trajectory& traj, int version = 1) {
    std::string filename = "trajectory_v" + std::to_string(version) + ".csv";
    std::string out = env_file_;
    auto pos = out.rfind("env.json");
    if (pos != std::string::npos) {
      out.replace(pos, 8, filename);
    } else {
      out = "/tmp/" + filename;
    }

    std::ofstream ofs(out);
    if (!ofs.is_open()) return;
    ofs << "t,x,y,theta,v,phi,a,omega,s,kappa\n";
    for (const auto& pt : traj) {
      ofs << pt.t << "," << pt.x << "," << pt.y << "," << pt.theta << ","
          << pt.v << "," << pt.phi << "," << pt.a << "," << pt.omega << ","
          << pt.s << "," << pt.kappa << "\n";
    }
    ROS_INFO("Trajectory v%d saved to: %s (%zu points)", version, out.c_str(), traj.size());
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_, goal_sub_;
  ros::Timer startup_timer_;
  ConditionPlanner planner_;
  MyEnvironment env_;
  std::string env_file_;
  bool has_start_ = false;
  bool has_goal_ = false;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "condition_planner_node");
  ConditionPlannerNode node;
  ros::spin();
  return 0;
}
