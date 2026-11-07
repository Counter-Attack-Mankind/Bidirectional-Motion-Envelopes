#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <fstream>
#include <sstream>

#include "common/math/vec2d.h"
#include "common/math/circle_2d.h"
#include "common/util/time.h"
#include "hybrid_astar/vehicle_parameter.h"

#include "visualization_plot.h"

#include "interface.h"

ConditionPlanner::ConditionPlanner(): nh_("~/condition_planner") {
  env_ = std::make_shared<Environment>();
  ReadConfig();
}

void ConditionPlanner::ReadConfig() {
  // vehicle parameters (从YAML读取，默认值为小车参数)
  nh_.param("back_edge_to_center", env_->vehicle.back_edge_to_center, 0.032);
  nh_.param("rear_hang", env_->vehicle.back_edge_to_center, env_->vehicle.back_edge_to_center);
  nh_.param("wheel_base", env_->vehicle.wheel_base, 0.143);
  nh_.param("length", env_->vehicle.length, 0.211);
  nh_.param("width", env_->vehicle.width, 0.191);
  nh_.param("max_acceleration", env_->vehicle.max_acceleration, 0.1);
  nh_.param("max_deceleration", env_->vehicle.max_deceleration, -0.1);
  nh_.param("max_velocity", env_->vehicle.max_velocity, 0.25);
  env_->vehicle.max_reverse_velocity = 0.0;  // 禁止后退
  nh_.param("max_steer_angle", env_->vehicle.max_steer_angle, 0.3491);
  nh_.param("max_phi", env_->vehicle.max_steer_angle, env_->vehicle.max_steer_angle);
  nh_.param("max_steer_angle_rate", env_->vehicle.max_steer_angle_rate, 0.1);
  nh_.param("max_omega", env_->vehicle.max_steer_angle_rate, env_->vehicle.max_steer_angle_rate);
  env_->vehicle.steer_ratio = 1.0;
  env_->vehicle.front_edge_to_center = env_->vehicle.length - env_->vehicle.back_edge_to_center;
  env_->vehicle.center_to_rear = env_->vehicle.front_edge_to_center - env_->vehicle.length * 0.5;
  env_->vehicle.GenerateDiscs();

  // hybrid A* parameters
  nh_.param("xy_grid_resolution", planning_config_.xy_grid_resolution, 0.02);
  nh_.param("phi_grid_resolution", planning_config_.phi_grid_resolution, 0.05);
  planning_config_.next_node_num = nh_.param("next_node_num", 20);
  nh_.param("step_size", planning_config_.step_size, 0.03);
  nh_.param("traj_forward_penalty", planning_config_.traj_forward_penalty, 1.0);
  planning_config_.traj_back_penalty = 1e6;       // 禁止后退
  planning_config_.traj_gear_switch_penalty = 1e6; // 禁止换挡
  nh_.param("traj_steer_penalty", planning_config_.traj_steer_penalty, 1.0);
  nh_.param("traj_steer_change_penalty", planning_config_.traj_steer_change_penalty, 5.0);
  nh_.param("grid_a_star_xy_resolution", planning_config_.grid_a_star_xy_resolution, 0.03);
  nh_.param("analytic_expansion_cost", planning_config_.analytic_expansion_cost, 20.0);
  planning_config_.explore_node_num = nh_.param("explore_node_num", 2000000);

  // LIOM optimization parameters
  // ECC 条件分支约束参数（转角-车速耦合约束，来自 NLP4.mod）
  nh_.param("ecc_vhalf",   nlp_config_.ecc_vhalf,   0.125);    // v <= vhalf 当 |phi| > phimax/2
  nh_.param("ecc_NATAN",   nlp_config_.ecc_NATAN,   10000.0);  // smooth step 斜率
  nh_.param("ecc_NF",      nlp_config_.ecc_NF,      1.0);
  nh_.param("ecc_epsl",    nlp_config_.ecc_epsl,    0.10);
  nh_.param("ecc_epsl_sq", nlp_config_.ecc_epsl_sq, 0.01);
  nh_.param("ecc_NG",      nlp_config_.ecc_NG,      10.0);

  nh_.param("nfe", nlp_config_.nfe, 100);
  nh_.param("tf_max", nlp_config_.tf_max, 500.0);
  nh_.param("corridor_max_iter", nlp_config_.corridor_max_iter, 1000);
  nh_.param("corridor_incremental_limit", nlp_config_.corridor_incremental_limit, 20.0);
  nh_.param("opti_omega", nlp_config_.opti_omega, 5.0);
  nh_.param("opti_a", nlp_config_.opti_a, 1.0);
  nh_.param("opti_phi", nlp_config_.opti_phi, 1.0);
  nh_.param("opti_ref_x", nlp_config_.opti_ref_x, 0.5);
  nh_.param("opti_ref_y", nlp_config_.opti_ref_y, 0.5);
  nh_.param("opti_ref_theta", nlp_config_.opti_ref_theta, 0.1);
  nh_.param("opti_ref_v", nlp_config_.opti_ref_v, 0.5);
  nh_.param("opti_smooth_a", nlp_config_.opti_smooth_a, 5.0);
  nh_.param("opti_smooth_omega", nlp_config_.opti_smooth_omega, 5.0);
  nh_.param("opti_iter_max", nlp_config_.opti_iter_max, 100);
  nh_.param("opti_w_penalty0", nlp_config_.opti_w_penalty0, 1e3);
  nh_.param("opti_alpha", nlp_config_.opti_alpha, 3.0);
  nh_.param("opti_w_penalty_max", nlp_config_.opti_w_penalty_max, 1e7);
  nh_.param("opti_varepsilon_tol", nlp_config_.opti_varepsilon_tol, 1e-6);
  nh_.param("opti_solution_norm_tol", nlp_config_.opti_solution_norm_tol, 1e-1);

  // DP S-V parameters for ECC initial guess
  nh_.param("dp_sv_tf", dp_sv_config_.tf, 60.0);
  dp_sv_config_.nfe = nlp_config_.nfe;  // must stay aligned with the NLP discretization
  dp_sv_config_.num_s_nodes = nh_.param("dp_sv_num_s_nodes", 80);
  dp_sv_config_.num_v_nodes = nh_.param("dp_sv_num_v_nodes", 25);
  nh_.param("dp_sv_w_nominal_velocity", dp_sv_config_.w_nominal_velocity, 5.0);
  nh_.param("dp_sv_w_acceleration", dp_sv_config_.w_acceleration, 1.0);
  nh_.param("dp_sv_w_jerk", dp_sv_config_.w_jerk, 2.0);
  nh_.param("dp_sv_v_min", dp_sv_config_.v_min, 1e-4);
  nh_.param("dp_sv_cost_max", dp_sv_config_.dp_cost_max, 1e8);
  nh_.param("dp_sv_v_ratio", dp_sv_config_.v_ratio, 0.9);
  nh_.param("dp_sv_a_ratio", dp_sv_config_.a_ratio, 1.0);
  nh_.param("dp_sv_v_reduction_rate", dp_sv_config_.v_reduction_rate, 0.5);
  nh_.param("dp_sv_dt_resolution", dp_sv_config_.dt_resolution, 1e-3);

  auto package_path = ros::package::getPath("condition_planner");
  std::string default_liom_csv =
      package_path.empty() ? std::string() : package_path + "/config/trajectory_v1.csv";
  nh_.param<std::string>("liom_csv_file", liom_csv_file_, default_liom_csv);
}

void ConditionPlanner::LoadEnvironment(const MyEnvironment &my_env) {
  env_->points.SetPoints(my_env.points);
  env_->UpdateBounds();
  env_->obstacles = my_env.obstacles;
}

bool ConditionPlanner::LoadLiomTrajectoryFromCsv(const std::string &csv_file, trajectory_nlp::States &states) const {
  std::ifstream ifs(csv_file);
  if (!ifs.is_open()) {
    ROS_ERROR("[PlanECC] Cannot open LIOM CSV: %s", csv_file.c_str());
    return false;
  }

  std::string line;
  if (!std::getline(ifs, line)) {
    ROS_ERROR("[PlanECC] Empty LIOM CSV: %s", csv_file.c_str());
    return false;
  }

  states = trajectory_nlp::States{};
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;
    while (std::getline(ss, token, ',')) {
      if (!token.empty()) {
        values.push_back(std::stod(token));
      }
    }

    if (values.size() < 8) {
      ROS_WARN("[PlanECC] Skip malformed CSV row with %zu values", values.size());
      continue;
    }

    states.t.push_back(values[0]);
    states.x.push_back(values[1]);
    states.y.push_back(values[2]);
    states.theta.push_back(values[3]);
    states.v.push_back(values[4]);
    states.phi.push_back(values[5]);
    states.a.push_back(values[6]);
    states.omega.push_back(values[7]);
  }

  if (states.x.size() < 2) {
    ROS_ERROR("[PlanECC] LIOM CSV has too few valid points: %zu", states.x.size());
    return false;
  }

  while (states.x.size() > 2) {
    const size_t last = states.x.size() - 1;
    const double dx = states.x[last] - states.x[last - 1];
    const double dy = states.y[last] - states.y[last - 1];
    if (std::hypot(dx, dy) > 1e-6 || std::abs(states.v[last]) > 1e-6) {
      break;
    }
    states.t.pop_back();
    states.x.pop_back();
    states.y.pop_back();
    states.theta.pop_back();
    states.v.pop_back();
    states.phi.pop_back();
    states.a.pop_back();
    states.omega.pop_back();
  }

  states.theta = ToContinuousAngle(states.theta);
  states.tf = states.t.empty() ? 0.0 : states.t.back();

  ROS_INFO("[PlanECC] Loaded LIOM CSV: %s (%zu points, tf=%.3f s)",
           csv_file.c_str(), states.x.size(), states.tf);
  return true;
}

bool ConditionPlanner::Plan(Trajectory *result) {
  TrajectoryPoint start(start_pose_);
  TrajectoryPoint goal(goal_pose_);
  // 起止状态: 从静止到静止
  start.v = 0.0; start.a = 0.0; start.phi = 0.0; start.omega = 0.0;
  goal.v = 0.0;  goal.a = 0.0;  goal.phi = 0.0;  goal.omega = 0.0;

  // 转换 VehicleParam → VehicleParameter (Hybrid A*用)
  VehicleParameter vehicle_param;
  vehicle_param.wheel_base = env_->vehicle.wheel_base;
  vehicle_param.front_hang = env_->vehicle.front_edge_to_center - env_->vehicle.wheel_base;
  vehicle_param.rear_hang = env_->vehicle.back_edge_to_center;
  vehicle_param.width = env_->vehicle.width;
  vehicle_param.v_max = env_->vehicle.max_velocity;
  vehicle_param.a_max = env_->vehicle.max_acceleration;
  vehicle_param.delta_max = env_->vehicle.max_steer_angle;
  vehicle_param.omega_max = env_->vehicle.max_steer_angle_rate;
  vehicle_param.steer_ratio = env_->vehicle.steer_ratio;
  vehicle_param.GenerateDisc();

  // 设置搜索边界
  auto bounds = env_->xy_bound();
  planning_config_.xy_bounds = bounds;
  ROS_INFO("Planning bounds: x=[%.2f, %.2f], y=[%.2f, %.2f]",
           bounds[0], bounds[1], bounds[2], bounds[3]);

  // 创建规划器
  planner_ = std::make_shared<planning::HybridAStar>(
      planning_config_, vehicle_param, env_->obstacles);
  optimizer_ = std::make_shared<trajectory_nlp::TrajectoryOptimizer>(nlp_config_, dp_sv_config_, env_);

  // ========== Step 1: Hybrid A* (仅前进) ==========
  double t0 = common::util::GetCurrentTimestamp();
  planning::HybridAStartResult ha_result;
  bool is_success = planner_->Plan(
      start.x, start.y, start.theta,
      goal.x, goal.y, goal.theta, &ha_result);
  double ha_time = common::util::GetCurrentTimestamp() - t0;

  if (!is_success) {
    ROS_ERROR("Hybrid A* failed!");
    return false;
  }
  ROS_INFO("Hybrid A* succeeded in %.3f s, %zu points", ha_time, ha_result.x.size());

  // 可视化 Hybrid A* 路径
  VisualizationPlot::Plot(ha_result.x, ha_result.y, 0.03, Color::Blue, 1, "Hybrid A* Path");
  VisualizationPlot::Trigger();

  // ========== Step 2: 构造粗解 ==========
  trajectory_nlp::CoarseDecision decision;
  decision.x = ha_result.x;
  decision.y = ha_result.y;
  decision.theta = ToContinuousAngle(ha_result.phi);

  // 所有gear都是前进(1)
  decision.gears.assign(ha_result.v.size(), 1);

  // 强制终点精确匹配
  if (!decision.x.empty()) {
    size_t last = decision.x.size() - 1;
    decision.x[last] = goal.x;
    decision.y[last] = goal.y;
    decision.theta[last] = goal.theta;
  }

  // ========== Step 3: LIOM 优化 ==========
  double t1 = common::util::GetCurrentTimestamp();
  trajectory_nlp::States states;
  is_success = optimizer_->Optimize(decision, start, goal, states);
  double opt_time = common::util::GetCurrentTimestamp() - t1;

  if (!is_success) {
    ROS_WARN("First LIOM attempt failed, retrying with 2x nfe...");
    nlp_config_.nfe *= 2;
    auto optimizer2 = std::make_shared<trajectory_nlp::TrajectoryOptimizer>(nlp_config_, dp_sv_config_, env_);
    is_success = optimizer2->Optimize(decision, start, goal, states);
    opt_time = common::util::GetCurrentTimestamp() - t1;
    nlp_config_.nfe /= 2;  // 恢复
  }

  ROS_INFO("LIOM optimization: %s in %.3f s", is_success ? "succeeded" : "FAILED", opt_time);
  ROS_INFO("Total planning time: %.3f s", ha_time + opt_time);

  if (!is_success || states.x.empty()) {
    ROS_ERROR("Optimization failed or returned empty states!");
    return false;
  }

  // ========== Step 4: 输出轨迹 ==========
  std::lock_guard<std::mutex> lock(trajectory_mutex);
  result->clear();

  for (size_t i = 0; i < states.x.size(); i++) {
    TrajectoryPoint tp{};
    tp.x = states.x[i];
    tp.y = states.y[i];
    tp.theta = states.theta[i];
    tp.v = states.v[i];
    tp.phi = states.phi[i];
    tp.a = states.a[i];
    tp.omega = states.omega[i];
    tp.kappa = tan(tp.phi) / env_->vehicle.wheel_base;
    result->push_back(tp);
  }

  ComputeTimeAndDistance(*result, states.tf, nlp_config_.nfe);

  return true;
}

// PlanECC: 版本2，带条件分支约束（转角大时限速）
// 流程: LIOM CSV 路径 -> DP S-V 配速 -> ECC 一次性 NLP
bool ConditionPlanner::PlanECC(Trajectory *result) {
  TrajectoryPoint start(start_pose_);
  TrajectoryPoint goal(goal_pose_);
  start.v = 0.0; start.a = 0.0; start.phi = 0.0; start.omega = 0.0;
  goal.v  = 0.0; goal.a  = 0.0; goal.phi  = 0.0; goal.omega  = 0.0;

  trajectory_nlp::States liom_path;
  if (!LoadLiomTrajectoryFromCsv(liom_csv_file_, liom_path)) {
    return false;
  }

  optimizer_ = std::make_shared<trajectory_nlp::TrajectoryOptimizer>(nlp_config_, dp_sv_config_, env_);

  const double t0 = common::util::GetCurrentTimestamp();
  trajectory_nlp::States states;
  const bool is_success = optimizer_->OptimizeECC(liom_path, start, goal, states);
  const double opt_time = common::util::GetCurrentTimestamp() - t0;

  ROS_INFO("[PlanECC] ECC optimization: %s in %.3f s", is_success ? "succeeded" : "FAILED", opt_time);

  if (!is_success || states.x.empty()) {
    ROS_ERROR("[PlanECC] ECC optimization failed or returned empty states!");
    return false;
  }

  // Step 4: 输出轨迹（与版本1相同）
  std::lock_guard<std::mutex> lock(trajectory_mutex);
  result->clear();

  for (size_t i = 0; i < states.x.size(); i++) {
    TrajectoryPoint tp{};
    tp.x     = states.x[i];
    tp.y     = states.y[i];
    tp.theta = states.theta[i];
    tp.v     = states.v[i];
    tp.phi   = states.phi[i];
    tp.a     = states.a[i];
    tp.omega = states.omega[i];
    tp.kappa = tan(tp.phi) / env_->vehicle.wheel_base;
    result->push_back(tp);
  }

  ComputeTimeAndDistance(*result, states.tf, nlp_config_.nfe);

  return true;
}
