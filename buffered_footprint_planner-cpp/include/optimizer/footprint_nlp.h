#pragma once

#include <array>
#include <string>
#include <vector>

#include "common/math/polygon2d.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "optimizer/trajectory.h"

namespace optimizer {

struct FootprintNlpConfig {
  double min_dt = 0.0001;
  double max_dt = 0.0500;
  double total_time_limit = 35.0;
  double dmin = 0.005;
  double tolerance = 1e-3;
  int max_iterations = 399984;
  double max_cpu_time = 20.0;
  std::string linear_solver = "ma27";
  bool save_debug_data = false;
};

OptimizationResult SolveFootprintNlp(
    const Trajectory& initial, const VehicleParameter& vehicle,
    const std::vector<common::math::Polygon2d>& obstacles,
    const std::array<double, 4>& bounds,
    const FootprintNlpConfig& config,
    const std::string& ipopt_log_file = std::string());

bool SaveSuccessfulOptimizationResults(const std::string& result_directory,
                                       const OptimizationResult& result,
                                       std::string* error);

bool SaveNlpFailureDiagnostics(const std::string& debug_directory,
                               const OptimizationResult& result,
                               std::string* error);

bool WriteCollisionValidation(
    const std::string& output_file, const Trajectory& trajectory,
    const VehicleParameter& vehicle,
    const std::vector<common::math::Polygon2d>& obstacles,
    const std::array<double, 4>& bounds, double dmin, std::string* error);

}  // namespace optimizer
