#pragma once

#include <array>
#include <string>
#include <vector>

#include "common/math/vec2d.h"

namespace optimizer {

struct TrajectoryNode {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double v = 0.0;
  double a = 0.0;
  double phy = 0.0;
  double w = 0.0;
};

struct FootprintInterval {
  double dt = 0.0;
  double s = 0.0;
  bool gear = true;
  double k = 0.0;
  double splus = 0.0;
  double sminus = 0.0;
  double up = 0.0;
  double down = 0.0;
  double left = 0.0;
  double right = 0.0;
  std::array<common::math::Vec2d, 4> corners;
};

struct Trajectory {
  std::vector<TrajectoryNode> nodes;
  std::vector<FootprintInterval> intervals;
};

struct MaximumViolation {
  double value = 0.0;
  std::string location;
};

struct NlpConstraintDiagnostics {
  MaximumViolation dynamics_x_m;
  MaximumViolation dynamics_y_m;
  MaximumViolation dynamics_theta_rad;
  MaximumViolation dynamics_v_mps;
  MaximumViolation dynamics_phi_rad;
  MaximumViolation velocity_bound_mps;
  MaximumViolation acceleration_bound_mps2;
  MaximumViolation steering_bound_rad;
  MaximumViolation steering_rate_bound_radps;
  MaximumViolation time_step_bound_s;
  MaximumViolation endpoint_x_m;
  MaximumViolation endpoint_y_m;
  MaximumViolation endpoint_theta_rad;
  MaximumViolation endpoint_v_mps;
  MaximumViolation endpoint_acceleration_mps2;
  MaximumViolation endpoint_phi_rad;
  MaximumViolation endpoint_steering_rate_radps;
  MaximumViolation total_time_equality_s;
  MaximumViolation total_time_bound_s;
  MaximumViolation ef_curvature_equality_1pm;
  MaximumViolation ef_distance_equality_m;
  MaximumViolation ef_corner_equality_m;
  MaximumViolation ef_arc;
  MaximumViolation obca_q_definition;
  MaximumViolation obca_separation_m;
  MaximumViolation obca_dual_equality;
  MaximumViolation obca_norm;
  MaximumViolation obca_nonnegative;
  MaximumViolation map_boundary_m;
};

struct NlpIterateSnapshot {
  bool available = false;
  std::string extraction_error;
  Trajectory trajectory;
  double total_time = 0.0;
  std::vector<double> k;
  std::vector<double> s;
  std::vector<double> splus;
  std::vector<double> sminus;
  std::vector<double> up;
  std::vector<double> down;
  std::vector<double> left;
  std::vector<double> right;
  std::vector<std::array<double, 8>> corners;
  std::vector<std::array<double, 4>> lambda;
  std::vector<std::array<double, 4>> mu;
  std::vector<double> obca_qx;
  std::vector<double> obca_qy;
};

struct OptimizationResult {
  Trajectory trajectory;
  bool initial_guess_succeeded = false;
  bool solver_succeeded = false;
  bool success = false;
  std::string status;
  bool solver_stats_available = false;
  bool solve_time_from_stats = false;
  bool last_iterate_available = false;
  std::string diagnostic_error;
  int iterations = -1;
  double solve_time = 0.0;
  double objective = 0.0;
  double max_dynamics_residual = 0.0;
  double max_variable_bound_violation = 0.0;
  double max_footprint_validity_violation = 0.0;
  double max_obca_violation = 0.0;
  double max_map_boundary_violation = 0.0;
  std::size_t dynamics_check_count = 0;
  std::size_t variable_bound_check_count = 0;
  std::size_t footprint_validity_check_count = 0;
  std::size_t obca_check_count = 0;
  std::size_t map_boundary_check_count = 0;
  std::string max_dynamics_location;
  std::string max_variable_bound_location;
  std::string max_footprint_validity_location;
  std::string max_obca_location;
  std::string max_map_boundary_location;
  int configured_max_iterations = 0;
  double configured_max_cpu_time = 0.0;
  double configured_tolerance = 0.0;
  std::string configured_linear_solver;
  std::string configured_hessian_approximation;
  std::string configured_mu_strategy;
  std::size_t model_variable_count = 0;
  std::size_t model_constraint_count = 0;
  std::size_t model_obstacle_count = 0;
  std::size_t model_obca_variable_count = 0;
  double model_build_time = 0.0;
  NlpConstraintDiagnostics diagnostics;
  NlpIterateSnapshot last_iterate;
};

}  // namespace optimizer
