#include <bitset>
#include <chrono>

#include "trajectory_optimizer.h"
#include "common/math/math_utils.h"
#include "common/math/trajectory1d.h"
#include "common/util/vector.h"
#include "common/util/csv_logger.h"
#include "visualization_plot.h"

namespace trajectory_nlp {

using common::math::Trajectory1d;
using common::util::NLPOptimizationLogger;

TrajectoryOptimizer::TrajectoryOptimizer(const TrajectoryNLPConfig &config, const DpSVConfig &dp_sv_config, Env env)
    : config_(config), dp_sv_config_(dp_sv_config), env_(std::move(env)), nlp_(config, env_) {
  vehicle_ = env_->vehicle;
}
// 标准 LIOM 优化流程 (参考 CartesianPlanner)
bool TrajectoryOptimizer::Optimize(const CoarseDecision &decision, const TrajectoryPoint &start, const TrajectoryPoint &goal, States &result) {
  // 初始化CSV记录器
  NLPOptimizationLogger::Initialize();

  // 重置可视化：清除之前的迭代轨迹、车身足迹和最终轨迹
  // Clear intermediate iteration trajectories (ids 0 to opti_iter_max)
  VisualizationPlot::ClearMarkers("Intermediate Iteration", 0, config_.opti_iter_max);
  // Clear final trajectory (id 1 from node.cpp, id 999 from optimizer)
  VisualizationPlot::ClearMarkers("Final Trajectory", 1, 1);
  VisualizationPlot::ClearMarkers("Final Trajectory", 999, 999);
  // Clear vehicle footprints (ids 1000 to 1000+nfe)
  VisualizationPlot::ClearMarkers("Vehicle Footprints", 1000, 1000 + config_.nfe);
  // Clear rear corridor (ids 0 to nfe)
  VisualizationPlot::ClearMarkers("Rear Corridor", 0, config_.nfe);
  // // Clear Hybrid A* path (id 1)
  // VisualizationPlot::ClearMarkers("Hybrid A* Path", 1, 1);
  VisualizationPlot::Trigger();

  // 1. 初始解生成 (Resample)
  auto coarse = decision;
  auto guess = ResampleCoarsePath(coarse);
  CalculateInitialGuess(guess);

  // ============ DIAGNOSTIC: Verify uniform time intervals ============
  ROS_INFO("========== Initial Guess Time Verification ==========");
  ROS_INFO("Total points (nfe): %zu", guess.t.size());
  ROS_INFO("Total time (tf): %.6f", guess.tf);
  double expected_dt = guess.tf / (guess.t.size() - 1);
  ROS_INFO("Expected uniform dt: %.6f", expected_dt);

  // Check first few intervals
  ROS_INFO("First 5 time intervals:");
  for (size_t i = 1; i < std::min(size_t(6), guess.t.size()); i++) {
    double dt = guess.t[i] - guess.t[i-1];
    ROS_INFO("  t[%zu] - t[%zu] = %.6f - %.6f = %.9f (error: %.2e)",
             i, i-1, guess.t[i], guess.t[i-1], dt, std::abs(dt - expected_dt));
  }

  // Check last few intervals
  ROS_INFO("Last 5 time intervals:");
  size_t start_idx = guess.t.size() >= 6 ? guess.t.size() - 5 : 1;
  for (size_t i = start_idx; i < guess.t.size(); i++) {
    double dt = guess.t[i] - guess.t[i-1];
    ROS_INFO("  t[%zu] - t[%zu] = %.6f - %.6f = %.9f (error: %.2e)",
             i, i-1, guess.t[i], guess.t[i-1], dt, std::abs(dt - expected_dt));
  }

  // Check terminal states
  ROS_INFO("Terminal state verification:");
  ROS_INFO("  goal: x=%.3f, y=%.3f, theta=%.3f, v=%.3f, a=%.3f, phi=%.3f",
           goal.x, goal.y, goal.theta, goal.v, goal.a, goal.phi);
  size_t last = guess.t.size() - 1;
  ROS_INFO("  guess[%zu]: x=%.3f, y=%.3f, theta=%.3f, v=%.3f, a=%.3f, phi=%.3f",
           last, guess.x[last], guess.y[last], guess.theta[last],
           guess.v[last], guess.a[last], guess.phi[last]);
  double terminal_dist = hypot(guess.x[last] - goal.x, guess.y[last] - goal.y);
  ROS_INFO("  Distance from guess to goal: %.6f m", terminal_dist);
  ROS_INFO("======================================================");
  // ====================================================================

  Constraints constraints;
  constraints.start = start;
  constraints.goal = goal;

  // 保存reference trajectory (coarse path重采样后的结果)
  States reference = guess;

  // 初始化迭代变量
  States current_states = guess;
  States previous_states;  // Store previous iteration's solution for convergence check

  // 标准LIOM权重策略：从较小值开始，每次迭代乘以alpha
  double w_penalty = config_.opti_w_penalty0;

  int iter = 0;

  // 论文 Algorithm 1 的标准实现
  while (iter < config_.opti_iter_max) {

      // Step 1: 基于当前轨迹生成走廊 (GenerateCorridors)
      constraints.gears = common::math::GetPathGears(current_states.x, current_states.y, current_states.theta);
      if (!FormulateCorridorConstraints(current_states, constraints)) {
          std::cout << "[LIOM] Failed to generate corridors at iter " << iter << std::endl;
          // 如果生成走廊失败（例如出界），可能需要回退或终止
          NLPOptimizationLogger::LogFinalResult(current_states.tf, current_states.x, current_states.y,
                                                current_states.theta, current_states.v, current_states.phi,
                                                current_states.a, current_states.omega, false);
          NLPOptimizationLogger::Finalize();
          return false;
      }

      // Step 2: 求解轻量级 OCP (SolveOCP) with reference trajectory tracking
      States next_states;
      double cur_infeasibility = 0.0;

      bool solver_success = nlp_.SolveIteratively(iter, w_penalty, constraints, current_states, reference, next_states, cur_infeasibility);

      // Intermediate iteration visualization: thin line (avoid gray due to gray background)
      VisualizationPlot::PlotTrajectory(next_states.x, next_states.y, next_states.v, vehicle_.max_velocity, 0.005, iter, "Intermediate Iteration");
      VisualizationPlot::Trigger();

      if (!solver_success) {
          std::cout << "[LIOM] Solver failed at iter " << iter << std::endl;
          NLPOptimizationLogger::LogFinalResult(current_states.tf, current_states.x, current_states.y,
                                                current_states.theta, current_states.v, current_states.phi,
                                                current_states.a, current_states.omega, false);
          NLPOptimizationLogger::Finalize();
          return false;
      }

      std::cout << "[LIOM] iter = " << iter << ", cur_infeasibility = " << cur_infeasibility << ", w_penalty = " << w_penalty << std::endl;

      // Step 3: 检查收敛性
      // 新的收敛判据：不仅要求运动学可行（infeasibility小），还要求前后两轮解向量一致
      bool kinematic_feasible = (cur_infeasibility < config_.opti_varepsilon_tol);
      bool solution_converged = false;
      double solution_norm_diff = 0.0;

      if (iter > 0) {
          // Compute norm difference between current and previous solution
          solution_norm_diff = ComputeSolutionNormDifference(next_states, previous_states);
          solution_converged = (solution_norm_diff < config_.opti_solution_norm_tol);
          std::cout << "[LIOM] Solution norm difference = " << solution_norm_diff << std::endl;
      }

      // Convergence criterion: kinematic feasibility AND solution consistency
      if (kinematic_feasible && (iter == 0 || solution_converged)) {
          std::cout << "[LIOM] Converged at iter " << iter
                    << " with infeasibility " << cur_infeasibility
                    << " and solution norm diff " << solution_norm_diff << std::endl;
          result = next_states;

          // Log final corridor constraints (only for converged solution)
          Constraints final_constraints;
          final_constraints.start = start;
          final_constraints.goal = goal;
          final_constraints.gears = common::math::GetPathGears(result.x, result.y, result.theta);
          FormulateCorridorConstraints(result, final_constraints, true);  // log_to_csv = true

          // 记录最终结果
          NLPOptimizationLogger::LogFinalResult(result.tf, result.x, result.y, result.theta,
                                                result.v, result.phi, result.a, result.omega, true);
          NLPOptimizationLogger::Finalize();

          // Final trajectory visualization: normal line width with vehicle footprints
          VisualizationPlot::PlotTrajectory(result.x, result.y, result.v, vehicle_.max_velocity, 0.01, 999, "Final Trajectory");
          VisualizationPlot::PlotVehicleFootprints(result.x, result.y, result.theta,
                                                    vehicle_.length, vehicle_.width, vehicle_.back_edge_to_center,
                                                    Color::Grey, 0.005, 1000, "Vehicle Footprints");
          VisualizationPlot::Trigger();
          return true;
      }

      // Step 4: 更新状态和权重进入下一次迭代
      previous_states = current_states;  // Save current state before updating
      current_states = next_states;

      // Update penalty weight with cap to prevent ill-conditioning
      if (w_penalty < config_.opti_w_penalty_max) {
        w_penalty *= config_.opti_alpha;
        // Ensure it doesn't exceed the maximum
        w_penalty = std::min(w_penalty, config_.opti_w_penalty_max);
      }

      iter++;
  }

  std::cout << "[LIOM] Reached max iterations without full convergence." << std::endl;

  // Log final corridor constraints (even for non-converged solution, for debugging)
  Constraints final_constraints;
  final_constraints.start = start;
  final_constraints.goal = goal;
  final_constraints.gears = common::math::GetPathGears(current_states.x, current_states.y, current_states.theta);
  FormulateCorridorConstraints(current_states, final_constraints, true);  // log_to_csv = true

  // 记录最终结果（未收敛）
  NLPOptimizationLogger::LogFinalResult(current_states.tf, current_states.x, current_states.y,
                                        current_states.theta, current_states.v, current_states.phi,
                                        current_states.a, current_states.omega, false);
  NLPOptimizationLogger::Finalize();

  result = current_states;
  // Final trajectory visualization: normal line width with vehicle footprints
  VisualizationPlot::PlotTrajectory(result.x, result.y, result.v, vehicle_.max_velocity, 0.01, 999, "Final Trajectory");
  VisualizationPlot::PlotVehicleFootprints(result.x, result.y, result.theta,
                                           vehicle_.length, vehicle_.width, vehicle_.back_edge_to_center,
                                           Color::Grey, 0.005, 1000, "Vehicle Footprints");
  return true;
}

// Function to resample the coarse decision into a finer path
States TrajectoryOptimizer::ResampleCoarsePath(CoarseDecision &coarse) {

  // Generate an optimal time profile for the coarse decision
  auto time_profile = GenerateOptimalTimeProfile(coarse);

  // Create linearly spaced time samples
  auto time_sampled = common::util::LinSpaced(0, time_profile.back(), config_.nfe);

  States result; // Declare an object to store the resulting resampled states
  result.tf = time_profile.back(); // Store the final time
  result.t = time_sampled; // Store the time samples

  // Resample x, y, and theta using 1D interpolation
  result.x = Trajectory1d(time_profile, coarse.x).Interpolate1d(time_sampled).GetY();
  result.y = Trajectory1d(time_profile, coarse.y).Interpolate1d(time_sampled).GetY();
  result.theta = common::math::ToContinuousAngle(Trajectory1d(time_profile, coarse.theta).Interpolate1d(time_sampled).GetY());

  // CRITICAL: Force exact match at start and end points to avoid interpolation errors
  // This ensures the resampled path exactly matches the coarse path endpoints
  result.x[0] = coarse.x.front();
  result.y[0] = coarse.y.front();
  result.theta[0] = coarse.theta.front();

  result.x[config_.nfe - 1] = coarse.x.back();
  result.y[config_.nfe - 1] = coarse.y.back();
  result.theta[config_.nfe - 1] = coarse.theta.back();

  ROS_INFO("[ResampleCoarsePath] Coarse points: %zu, Resampled to: %d, tf: %.3f s",
           coarse.x.size(), config_.nfe, result.tf);

  return result; // Return the resampled states
}

// Function to calculate an initial guess for the states (参考CartesianPlanner)
// IMPORTANT: Must be consistent with NLP's forward Euler discretization:
//   x[i+1] = x[i] + hi * v[i] * cos(theta[i])
//   theta[i+1] = theta[i] + hi * v[i] * tan(phi[i]) / L
//   v[i+1] = v[i] + hi * a[i]
//   phi[i+1] = phi[i] + hi * omega[i]
// So v[i] and phi[i] should be computed from (x[i+1]-x[i]) and (theta[i+1]-theta[i])
void TrajectoryOptimizer::CalculateInitialGuess(States &states) const {

  // Initialize vectors for velocity and steering angle
  states.v.resize(config_.nfe, 0.0);
  states.phi.resize(config_.nfe, 0.0);

  // Time step (assuming uniform time sampling)
  double dt = states.tf / (config_.nfe - 1);

  // Calculate v[i] and phi[i] from forward differences (x[i+1]-x[i], theta[i+1]-theta[i])
  // This matches NLP's forward Euler: x[i+1] = x[i] + dt * v[i] * cos(theta[i])
  for(size_t i = 0; i < config_.nfe - 1; i++) {
    // Calculate displacement from current to next point
    double dx = states.x[i+1] - states.x[i];
    double dy = states.y[i+1] - states.y[i];
    double dtheta = states.theta[i+1] - states.theta[i];

    // Calculate tracking angle between current and next points
    double tracking_angle = atan2(dy, dx);

    // Determine if the vehicle should move forward or backward
    bool gear = std::abs(common::math::NormalizeAngle(tracking_angle - states.theta[i])) < M_PI_2;

    // Calculate velocity: v[i] is used for x[i+1] - x[i]
    double velocity = hypot(dy, dx) / dt;
    states.v[i] = std::min(vehicle_.max_velocity, std::max(vehicle_.max_reverse_velocity, gear ? velocity : -velocity));

    // Calculate steering angle: phi[i] is used for theta[i+1] - theta[i]
    // From: theta[i+1] = theta[i] + dt * v[i] * tan(phi[i]) / L
    // => tan(phi[i]) = dtheta * L / (dt * v[i])
    if (std::abs(states.v[i]) < 1e-3) {
      // Velocity too small, cannot reliably compute steering from kinematics
      states.phi[i] = 0.0;
    } else {
      double raw_phi = atan(dtheta * vehicle_.wheel_base / (states.v[i] * dt));
      states.phi[i] = std::min(vehicle_.max_steer_angle, std::max(-vehicle_.max_steer_angle, raw_phi));
    }
  }

  // Terminal point: v and phi should be 0 (vehicle stopped)
  states.v[config_.nfe - 1] = 0.0;
  states.phi[config_.nfe - 1] = 0.0;

  // Initialize acceleration: a[i] is used for v[i+1] - v[i]
  states.a.resize(config_.nfe, 0.0);
  for(size_t i = 0; i < config_.nfe - 1; i++) {
    double a = (states.v[i+1] - states.v[i]) / dt;
    states.a[i] = std::min(vehicle_.max_acceleration,
                           std::max(vehicle_.max_deceleration, a));
  }
  states.a[config_.nfe - 1] = 0.0;

  // Initialize steering rate: omega[i] is used for phi[i+1] - phi[i]
  states.omega.resize(config_.nfe, 0.0);
  for(size_t i = 0; i < config_.nfe - 1; i++) {
    double omega = (states.phi[i+1] - states.phi[i]) / dt;
    states.omega[i] = std::min(vehicle_.max_steer_angle_rate,
                               std::max(-vehicle_.max_steer_angle_rate, omega));
  }
  states.omega[config_.nfe - 1] = 0.0;
}

// Generate optimal time profile for a given coarse trajectory decision
std::vector<double> TrajectoryOptimizer::GenerateOptimalTimeProfile(const CoarseDecision &coarse) {
  // Declare a vector 'gears' to store gear values for each point (either 1 for forward or -1 for reverse)
  std::vector<int> gears(coarse.x.size());

  // Use gear information from Hybrid A* if available, otherwise compute from trajectory
  bool use_ha_gears = !coarse.gears.empty() && coarse.gears.size() == coarse.x.size();

  if (use_ha_gears) {
    // Use the gear information directly from Hybrid A* (more reliable)
    gears = coarse.gears;
    ROS_INFO("[GenerateOptimalTimeProfile] Using Hybrid A* gear information (%zu points)", gears.size());
  } else {
    // Fallback: compute gear from trajectory direction (less reliable)
    ROS_WARN("[GenerateOptimalTimeProfile] No Hybrid A* gear info, computing from trajectory direction");
    for(size_t i = 0; i < gears.size(); i++) {
      if(i < gears.size()-1) {
        double tracking_angle = atan2(coarse.y[i+1] - coarse.y[i], coarse.x[i+1] - coarse.x[i]);
        bool gear = std::abs(common::math::NormalizeAngle(tracking_angle - coarse.theta[i])) < M_PI_2;
        gears[i] = gear ? 1 : -1;
      }
    }
    gears[gears.size()-1] = gears[gears.size()-2];
  }

  // Declare a vector 'stations' initialized to 0s to store accumulated distance at each point on the coarse trajectory
  std::vector<double> stations(coarse.x.size(), 0);
  // Calculate accumulated distance for each point
  for(size_t i = 1; i < coarse.x.size(); i++) {
    stations[i] = stations[i-1] + hypot(coarse.x[i] - coarse.x[i-1], coarse.y[i] - coarse.y[i-1]);
  }

  // Initialize a time profile vector to store time values for each point on the trajectory
  std::vector<double> time_profile(gears.size());
  size_t last_idx = 0;  // Index to track the start of the current segment
  double start_time = 0;
  // Loop through the gears vector to generate the optimal time profile for each segment
  for(size_t i = 0; i < gears.size(); i++) {
    // Split the trajectory at points where gear changes or at the last point
    if(i == gears.size() - 1 || gears[i+1] != gears[i]) {
      // Extract the current segment of stations (from last_idx to current i)
      std::vector<double> station_segment;
      std::copy_n(stations.begin() + last_idx, i - last_idx + 1, std::back_inserter(station_segment));
      // Generate the time profile for the current segment
      auto profile = GenerateOptimalTimeProfileSegment(gears[i], station_segment, start_time);
      // Update the main time profile with the calculated values for the current segment
      std::copy(profile.begin(), profile.end(), std::next(time_profile.begin(), last_idx));
      // Update the start time for the next segment
      start_time = profile.back();
      // Update the index to track the start of the next segment
      last_idx = i + 1;
    }
  }
  // Return the final time profile
  return time_profile;
}
// Generate an optimal time profile for a given segment based on gear, stations, and starting time
// Uses trapezoidal velocity profile: accelerate -> cruise -> decelerate
std::vector<double>
TrajectoryOptimizer::GenerateOptimalTimeProfileSegment(int gear, const std::vector<double> &stations, double start_time) const {
  // Initialize vehicle parameters
  double max_accel = std::abs(vehicle_.max_acceleration);
  double max_decel = std::abs(vehicle_.max_deceleration);
  double max_velocity = std::abs(gear > 0 ? vehicle_.max_velocity : vehicle_.max_reverse_velocity);

  // Total distance of this segment
  double total_dist = stations.back() - stations.front();
  if (total_dist < 1e-6) {
    // No movement, return constant time
    return std::vector<double>(stations.size(), start_time);
  }

  // Calculate trapezoidal velocity profile parameters
  // Distance to accelerate from 0 to max_velocity: d_accel = v^2 / (2*a)
  double d_accel = max_velocity * max_velocity / (2 * max_accel);
  // Distance to decelerate from max_velocity to 0: d_decel = v^2 / (2*a)
  double d_decel = max_velocity * max_velocity / (2 * max_decel);

  double v_peak = max_velocity;
  double d_cruise = 0.0;

  if (d_accel + d_decel > total_dist) {
    // Cannot reach max velocity, use triangular profile
    // v_peak^2 / (2*a_accel) + v_peak^2 / (2*a_decel) = total_dist
    // v_peak^2 * (1/(2*a_accel) + 1/(2*a_decel)) = total_dist
    v_peak = sqrt(2 * total_dist / (1/max_accel + 1/max_decel));
    d_accel = v_peak * v_peak / (2 * max_accel);
    d_decel = v_peak * v_peak / (2 * max_decel);
    d_cruise = 0.0;
  } else {
    // Trapezoidal profile with cruise phase
    d_cruise = total_dist - d_accel - d_decel;
  }

  // Calculate time for each phase
  double t_accel = v_peak / max_accel;           // Time to accelerate
  double t_cruise = d_cruise / max_velocity;      // Time at cruise (0 if triangular)
  double t_decel = v_peak / max_decel;           // Time to decelerate

  // Station where acceleration ends and cruise begins
  double s_accel_end = stations.front() + d_accel;
  // Station where cruise ends and deceleration begins
  double s_decel_start = stations.back() - d_decel;

  // Generate time profile for each station
  std::vector<double> time_profile(stations.size());
  time_profile[0] = start_time;

  for (size_t i = 1; i < stations.size(); i++) {
    double s = stations[i] - stations.front();  // Distance from segment start

    double t;
    if (s <= d_accel) {
      // In acceleration phase: s = 0.5 * a * t^2, so t = sqrt(2*s/a)
      t = sqrt(2 * s / max_accel);
    } else if (s <= d_accel + d_cruise) {
      // In cruise phase
      t = t_accel + (s - d_accel) / max_velocity;
    } else {
      // In deceleration phase
      // Distance into deceleration phase
      double s_in_decel = s - d_accel - d_cruise;
      // Time in deceleration: using v = v_peak - a*t and s = v_peak*t - 0.5*a*t^2
      // Solving: t = (v_peak - sqrt(v_peak^2 - 2*a*s)) / a
      double discriminant = v_peak * v_peak - 2 * max_decel * s_in_decel;
      if (discriminant < 0) discriminant = 0;
      double t_in_decel = (v_peak - sqrt(discriminant)) / max_decel;
      t = t_accel + t_cruise + t_in_decel;
    }

    time_profile[i] = start_time + t;
  }

  return time_profile;
}

bool TrajectoryOptimizer::FormulateCorridorConstraints(const States &states, Constraints &constraints, bool log_to_csv) {
  constraints.front_bound.resize(config_.nfe);
  constraints.rear_bound.resize(config_.nfe);

  for(size_t i = 0; i < config_.nfe; i++) {
    double xf, yf, xr, yr;
    std::tie(xf, yf, xr, yr) = vehicle_.GetDiscPositions(states.x[i], states.y[i], states.theta[i]);

    AABox2d front, rear;
    if(!GenerateAABox(xf, yf, vehicle_.radius, front)) {
      return false;
    }
    constraints.front_bound[i] = { front.min_x(), front.max_x(), front.min_y(), front.max_y() };

    // Only log corridor constraints if explicitly requested (e.g., at final convergence)
    if (log_to_csv) {
      NLPOptimizationLogger::LogCorridorConstraints(i,
                                                    front.min_x(), front.max_x(),
                                                    front.min_y(), front.max_y(),
                                                    true);
    }

    if(!GenerateAABox(xr, yr, vehicle_.radius, rear)) {
      return false;
    }
    constraints.rear_bound[i] = { rear.min_x(), rear.max_x(), rear.min_y(), rear.max_y() };

    // Only log corridor constraints if explicitly requested (e.g., at final convergence)
    if (log_to_csv) {
      NLPOptimizationLogger::LogCorridorConstraints(i,
                                                    rear.min_x(), rear.max_x(),
                                                    rear.min_y(), rear.max_y(),
                                                    false);
    }
  }

  // Plot Corridor - light color and thin line to avoid visual clutter
  for(size_t i = 0; i < constraints.rear_bound.size(); i++) {
    Polygon2d box = Polygon2d(Box2d(
        AABox2d({constraints.rear_bound[i][0], constraints.rear_bound[i][2]},
                {constraints.rear_bound[i][1], constraints.rear_bound[i][3]})));
    Color corridor_color(0.7, 0.7, 0.7);
    corridor_color.set_a(0.5);
    VisualizationPlot::PlotPolygon(box, 0.005, corridor_color, i, "Rear Corridor");
  }
  // for(size_t i = 0; i < constraints.front_bound.size(); i++) {
  //   Polygon2d box = Polygon2d(Box2d(
  //       AABox2d({constraints.front_bound[i][0], constraints.front_bound[i][2]},
  //               {constraints.front_bound[i][1], constraints.front_bound[i][3]})));
  //   VisualizationPlot::PlotPolygon(box, 0.01, common::util::Color::Red, i, "Front Corridor");
  // }
  VisualizationPlot::Trigger();

  return true;
}
// This function attempts to generate an axis-aligned bounding box (AABox) for a given
// (x, y) point with the specified radius. The bounding box is adjusted to avoid collisions.
bool TrajectoryOptimizer::GenerateAABox(double &x, double &y, double radius, AABox2d &box) const {
  using namespace std::chrono;
  static int call_count = 0;
  static duration<double> total_duration(0);

  auto start = high_resolution_clock::now(); // 开始计时
  // Create an initial bounding box using the given point and radius.
  double ri = radius;
  AABox2d bound({x-ri, y-ri}, {x+ri, y+ri});
  // Check if the initial bounding box collides with any obstacle in the environment.
  if(env_->CheckCollision(Box2d(bound))) {
    // initial condition not satisfied, involute to find feasible box
    // The initial box has a collision. Begin the process to adjust (involute) the box.
    int inc = 4;
    double real_x, real_y;
    // Repeat the adjustment process until a collision-free box is found or iteration limits are reached.
    do {
      // Determine which edge of the bounding box to adjust.
      int iter = inc / 4;
      uint8_t edge = inc % 4;
      // Initialize the adjusted position to the current (x, y).
      real_x = x;
      real_y = y;
      // Adjust the box's position based on which edge is being modified.
      if(edge == 0) {
        real_x = x - iter * 0.01;
      } else if(edge == 1) {
        real_x = x + iter * 0.01;
      } else if(edge == 2) {
        real_y = y - iter * 0.01;
      } else if(edge == 3) {
        real_y = y + iter * 0.01;
      }
      // Update the bounding box with the adjusted position.
      inc++;
      bound = AABox2d({real_x-ri, real_y-ri}, {real_x+ri, real_y+ri});
    } while(env_->CheckCollision(Box2d(bound)) && inc < config_.corridor_max_iter);
    // If the adjustment iterations exceed the allowed limit, return false.
    if(inc > config_.corridor_max_iter) {
      return false;
    }
    // Update the original x and y with the adjusted values.
    x = real_x;
    y = real_y;
  }
  // Begin the process to expand the bounding box edges as much as possible, while avoiding collisions.
  int inc = 4;
  std::bitset<4> blocked;  // Tracks which edges can no longer be expanded.
  double incremental[4] = {0.0};
  double step = radius * 0.01;//0.1;//radius * 0.005;
  // Repeat the expansion process until all edges are blocked or iteration limits are reached.
  do {
    int iter = inc / 4;
    uint8_t edge = inc % 4;
    inc++;
    // If the current edge is already blocked from further expansion, skip to the next iteration.
    if(blocked[edge]) continue;
    // Attempt to expand the current edge.
    incremental[edge] = iter * step;
    // Generate a test bounding box with the expanded edge.
    AABox2d test({x - ri - incremental[0], y - ri - incremental[2]},
                 { x + ri + incremental[1], y + ri + incremental[3]});
    // If the test bounding box collides or the expansion reaches a limit, block further expansion of the current edge.
    if(env_->CheckCollision(Box2d(test)) || incremental[edge] >= config_.corridor_incremental_limit) {
      incremental[edge] -= step;
      blocked[edge] = true;
    }
  } while(!blocked.all() && inc < config_.corridor_max_iter);
  // If the expansion iterations exceed the allowed limit, return false.
  if(inc > config_.corridor_max_iter) {
    return false;
  }
  // Finalize the bounding box using the expanded edges and assign it to the 'box' argument.
  box = {{x - incremental[0], y - incremental[2]},
          { x + incremental[1], y + incremental[3]}};

  auto end = high_resolution_clock::now(); // 结束计时
  duration<double> elapsed = end - start; // 计算经过的时间
  total_duration += elapsed; // 累加到总时间
  call_count++; // 调用次数增加
  // 输出单次调用时间，总调用次数和总时间
  // std::cout << "Call " << call_count << ": "
  //           << "Elapsed time: " << elapsed.count() << " seconds, "
  //           << "Total time: " << total_duration.count() << " seconds.\n";

  // Return true to indicate a successful generation of the bounding box.
  return true;
}

// ============================================================
// ECC 版本：基于曲率限速 DP 配速 + 一次性 NLP 优化
// ============================================================

// 计算满足转角-车速约束的时间序列
// 方法：从路径曲率推算各点 phi，高曲率点限速，前向-反向通行求速度剖面
std::vector<double> TrajectoryOptimizer::GenerateTimeProfileWithCurvatureLimit(const CoarseDecision &coarse) const {
  int n = coarse.x.size();
  if (n < 2) return std::vector<double>(n, 0.0);

  // 1. 计算弧长
  std::vector<double> s(n, 0.0);
  for (int i = 1; i < n; i++) {
    s[i] = s[i-1] + hypot(coarse.x[i] - coarse.x[i-1], coarse.y[i] - coarse.y[i-1]);
  }

  // 2. 从路径曲率推算各点 phi，并确定速度上限
  const double phimax    = vehicle_.max_steer_angle;      // 0.3491 rad
  const double threshold = 0.5 * phimax;                  // 0.17455 rad，|phi| 超过此值则限速
  const double vhalf     = config_.ecc_vhalf;             // 0.125 m/s
  const double vmax      = vehicle_.max_velocity;          // 0.25 m/s

  std::vector<double> v_limit(n, vmax);
  for (int i = 0; i < n - 1; i++) {
    double ds = s[i+1] - s[i];
    double dtheta = coarse.theta[i+1] - coarse.theta[i];
    // 归一化 dtheta 到 [-pi, pi]
    while (dtheta >  M_PI) dtheta -= 2*M_PI;
    while (dtheta < -M_PI) dtheta += 2*M_PI;
    double kappa = (ds > 1e-6) ? dtheta / ds : 0.0;
    double phi_i = atan(kappa * vehicle_.wheel_base);
    if (std::abs(phi_i) > threshold) {
      v_limit[i] = vhalf;
    }
  }
  v_limit[n-1] = v_limit[n-2];

  // 3. 前向-反向速度规划（梯形速度剖面 + 局部限速）
  const double a_max   = vehicle_.max_acceleration;
  const double a_decel = std::abs(vehicle_.max_deceleration);

  std::vector<double> v(n, 0.0);

  // 前向通行（加速受限）
  v[0] = 0.0;
  for (int i = 1; i < n; i++) {
    double ds = s[i] - s[i-1];
    double v_fwd = std::sqrt(std::max(0.0, v[i-1]*v[i-1] + 2.0*a_max*ds));
    v[i] = std::min(v_limit[i], v_fwd);
  }

  // 反向通行（减速受限）
  v[n-1] = 0.0;
  for (int i = n-2; i >= 0; i--) {
    double ds = s[i+1] - s[i];
    double v_bwd = std::sqrt(std::max(0.0, v[i+1]*v[i+1] + 2.0*a_decel*ds));
    v[i] = std::min(v[i], std::min(v_bwd, v_limit[i]));
  }

  // 4. 从速度剖面积分时间戳
  std::vector<double> t(n, 0.0);
  for (int i = 1; i < n; i++) {
    double ds    = s[i] - s[i-1];
    double v_avg = 0.5 * (v[i] + v[i-1]);
    if (v_avg < 1e-6) {
      // 避免除以零：用加速时间估算
      t[i] = t[i-1] + std::sqrt(2.0 * ds / (a_max + 1e-10));
    } else {
      t[i] = t[i-1] + ds / v_avg;
    }
  }

  ROS_INFO("[GenerateTimeProfileWithCurvatureLimit] tf=%.3f s, vhalf=%.3f m/s, threshold=%.4f rad",
           t.back(), vhalf, threshold);
  return t;
}

// ECC 版重采样（使用曲率限速时间序列代替梯形速度时间序列）
States TrajectoryOptimizer::ResampleCoarsePathECC(const CoarseDecision &coarse) const {
  auto time_profile = GenerateTimeProfileWithCurvatureLimit(coarse);
  auto time_sampled  = common::util::LinSpaced(0, time_profile.back(), config_.nfe);

  States result;
  result.tf    = time_profile.back();
  result.t     = time_sampled;
  result.x     = Trajectory1d(time_profile, coarse.x).Interpolate1d(time_sampled).GetY();
  result.y     = Trajectory1d(time_profile, coarse.y).Interpolate1d(time_sampled).GetY();
  result.theta = common::math::ToContinuousAngle(
                   Trajectory1d(time_profile, coarse.theta).Interpolate1d(time_sampled).GetY());

  // 强制端点精确匹配
  result.x[0]              = coarse.x.front();
  result.y[0]              = coarse.y.front();
  result.theta[0]          = coarse.theta.front();
  result.x[config_.nfe-1]     = coarse.x.back();
  result.y[config_.nfe-1]     = coarse.y.back();
  result.theta[config_.nfe-1] = coarse.theta.back();

  ROS_INFO("[ResampleCoarsePathECC] Resampled %zu -> %d points, tf=%.3f s",
           coarse.x.size(), config_.nfe, result.tf);
  return result;
}

// ECC 主优化流程：
// 1. 读取 LIOM 轨迹 → 2. DP S-V 配速得到初始解 → 3. 生成走廊 → 4. 一次性 NLP（pair1/pair2）
bool TrajectoryOptimizer::OptimizeECC(const States &liom_path, const TrajectoryPoint &start, const TrajectoryPoint &goal, States &result) {
  NLPOptimizationLogger::Initialize();

  // 清除可视化（复用 LIOM 的命名空间前缀以便对比）
  VisualizationPlot::ClearMarkers("ECC Reference", 0, 1);
  VisualizationPlot::ClearMarkers("Final Trajectory", 998, 998);
  VisualizationPlot::ClearMarkers("Vehicle Footprints", 2000, 2000 + config_.nfe);
  VisualizationPlot::ClearMarkers("Rear Corridor", 0, config_.nfe);
  VisualizationPlot::Trigger();

  // Step 1: 沿 LIOM 路径做 DP S-V 配速，得到 ECC 初始解
  DpSVConfig dp_config = dp_sv_config_;
  dp_config.nfe = config_.nfe;
  DpSVPlannerECC dp_planner(dp_config, vehicle_);
  States reference;
  if (!dp_planner.Plan(liom_path, start, reference)) {
    ROS_ERROR("[OptimizeECC] DP S-V planning failed");
    NLPOptimizationLogger::Finalize();
    return false;
  }

  // 可视化参考轨迹（DP 配速后的初始解）
  VisualizationPlot::PlotTrajectory(reference.x, reference.y, reference.v,
                                    vehicle_.max_velocity, 0.005, 0, "ECC Reference");
  VisualizationPlot::Trigger();

  Constraints constraints;
  constraints.start = start;
  constraints.goal  = goal;
  constraints.gears = common::math::GetPathGears(reference.x, reference.y, reference.theta);

  // Step 2: 一次性生成走廊（不迭代收紧）
  if (!FormulateCorridorConstraints(reference, constraints)) {
    ROS_ERROR("[OptimizeECC] Failed to generate corridor constraints");
    NLPOptimizationLogger::Finalize();
    return false;
  }

  // Step 3: 一次性求解 ECC NLP（pair1/pair2 条件分支约束）
  States nlp_result;
  bool success = nlp_.SolveECC(constraints, reference, reference, nlp_result);

  if (!success) {
    ROS_ERROR("[OptimizeECC] ECC NLP failed");
    NLPOptimizationLogger::LogFinalResult(reference.tf, reference.x, reference.y, reference.theta,
                                          reference.v, reference.phi, reference.a, reference.omega, false);
    NLPOptimizationLogger::Finalize();
    return false;
  }
  result = nlp_result;

  // 更新时间戳（从 result.tf 重新均匀分配）
  result.t = common::util::LinSpaced(0, result.tf, config_.nfe);

  NLPOptimizationLogger::LogFinalResult(result.tf, result.x, result.y, result.theta,
                                        result.v, result.phi, result.a, result.omega, success);
  NLPOptimizationLogger::Finalize();

  // 可视化最终轨迹
  VisualizationPlot::PlotTrajectory(result.x, result.y, result.v,
                                    vehicle_.max_velocity, 0.01, 998, "Final Trajectory");
  VisualizationPlot::PlotVehicleFootprints(result.x, result.y, result.theta,
                                            vehicle_.length, vehicle_.width,
                                            vehicle_.back_edge_to_center,
                                            Color::Grey, 0.005, 2000, "Vehicle Footprints");
  VisualizationPlot::Trigger();

  return true;
}

// Compute L2 norm difference between two solution vectors
// This includes all state variables: tf, x, y, theta, v, phi, a, omega
double TrajectoryOptimizer::ComputeSolutionNormDifference(const States &states1, const States &states2) const {
  double norm_diff = 0.0;

  // Add difference in tf
  double tf_diff = states1.tf - states2.tf;
  norm_diff += tf_diff * tf_diff;

  // Add differences in all trajectory points
  for(size_t i = 0; i < config_.nfe; i++) {
    double x_diff = states1.x[i] - states2.x[i];
    double y_diff = states1.y[i] - states2.y[i];
    double theta_diff = states1.theta[i] - states2.theta[i];
    double v_diff = states1.v[i] - states2.v[i];
    double phi_diff = states1.phi[i] - states2.phi[i];
    double a_diff = states1.a[i] - states2.a[i];
    double omega_diff = states1.omega[i] - states2.omega[i];

    norm_diff += x_diff * x_diff + y_diff * y_diff + theta_diff * theta_diff
               + v_diff * v_diff + phi_diff * phi_diff + a_diff * a_diff
               + omega_diff * omega_diff;
  }

  return std::sqrt(norm_diff);
}

}
