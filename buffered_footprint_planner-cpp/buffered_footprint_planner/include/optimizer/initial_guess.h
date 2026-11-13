#pragma once

#include <array>
#include <limits>
#include <string>
#include <vector>

#include "common/math/polygon2d.h"
#include "planning/hybrid_astar/hybrid_a_star.h"
#include "planning/hybrid_astar/vehicle_parameter.h"
#include "optimizer/trajectory.h"

namespace optimizer {

struct InitialGuessConfig {
  double min_dt = 0.0001;
  double max_dt = 0.0500;
  double eta = 0.9;
  double forward_speed = 0.20;
  double reverse_speed = 0.16;
  double max_acceleration = 0.10;
  double max_steering_rate = 0.30;
  double dmin = 0.005;
  double total_time_limit = 35.0;
};

struct RejectedInterval {
  std::size_t interval_index = 0;
  std::size_t dense_start = 0;
  std::size_t dense_end = 0;
  double dt = 0.0;
  double curvature = 0.0;
  double signed_distance = 0.0;
  std::string violated_conditions;
  double max_violation = 0.0;
  std::array<double, 5> violations{};
};

struct EnvironmentFallbackInterval {
  std::size_t interval_index = 0;
  std::size_t dense_start = 0;
  std::size_t dense_end = 0;
  double dt = 0.0;
  std::string violated_condition;
  double violation = 0.0;
  double obstacle_clearance = std::numeric_limits<double>::infinity();
  double map_boundary_violation = 0.0;
};

struct InitialGuessDiagnostics {
  bool success = false;
  std::string failure_reason;
  std::size_t hybrid_point_count = 0;
  std::size_t deduplicated_point_count = 0;
  double hybrid_path_length = 0.0;
  std::size_t raw_direction_segments = 0;
  std::size_t effective_direction_segments = 0;
  std::size_t raw_gear_switches = 0;
  std::size_t effective_gear_switches = 0;
  std::vector<std::string> merged_short_segments;
  std::size_t dense_node_count = 0;
  std::size_t final_node_count = 0;
  double min_dt = 0.0;
  double max_dt = 0.0;
  double total_time = 0.0;
  double max_abs_velocity = 0.0;
  double max_abs_acceleration = 0.0;
  double max_abs_steering = 0.0;
  double max_abs_raw_steering_rate = 0.0;
  double max_abs_clipped_steering_rate = 0.0;
  double max_euler_residual = 0.0;
  double max_footprint_violation = 0.0;
  std::size_t structural_min_intervals = 0;
  std::size_t actual_interval_count = 0;
  std::size_t obstacle_limited_interval_count = 0;
  std::size_t environment_fallback_interval_count = 0;
  double max_environment_fallback_violation = 0.0;
  double reference_step_cap = 0.0;
  double maximum_available_trajectory_time = 0.0;
  double minimum_bme_obstacle_clearance =
      std::numeric_limits<double>::infinity();
  double max_bme_map_boundary_violation = 0.0;
  double endpoint_pose_error = 0.0;
  double max_velocity_bound_violation = 0.0;
  double max_acceleration_bound_violation = 0.0;
  double max_steering_bound_violation = 0.0;
  double max_steering_rate_bound_violation = 0.0;
  std::size_t nonfinite_value_count = 0;
  std::size_t direction_sign_mismatch_count = 0;
  std::size_t acceleration_clip_count = 0;
  std::size_t steering_rate_clip_count = 0;
  std::vector<RejectedInterval> rejected_intervals;
  std::vector<EnvironmentFallbackInterval> environment_fallback_intervals;
};

struct DenseTrajectorySample {
  std::size_t dense_index = 0;
  double time = 0.0;
  TrajectoryNode node;
  double phi_raw = 0.0;
  double phi_clipped = 0.0;
  double w_raw = 0.0;
  double w_clipped = 0.0;
};

struct InitialGuessTrace {
  std::vector<DenseTrajectorySample> dense_samples;
  std::vector<std::size_t> selected_dense_indices;
};

bool BuildInitialGuess(const planning::HybridAStartResult& hybrid_path,
                       const VehicleParameter& vehicle,
                       const InitialGuessConfig& config,
                       const std::vector<common::math::Polygon2d>& obstacles,
                       const std::array<double, 4>& bounds,
                       Trajectory* trajectory, InitialGuessDiagnostics* diagnostics,
                       std::string* error, InitialGuessTrace* trace = nullptr);

bool SaveInitialGuessDiagnostics(const std::string& output_file,
                                 const InitialGuessDiagnostics& diagnostics,
                                 std::string* error);

}  // namespace optimizer
