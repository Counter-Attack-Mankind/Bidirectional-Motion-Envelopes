#include "planning/planning_pipeline.h"

#include <chrono>
#include <filesystem>

#include "experiment/experiment_logger.h"
#include "optimizer/embodied_footprint.h"
#include "optimizer/footprint_nlp.h"

namespace planning_platform {
namespace {

double SecondsSince(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

void ConfigurePendingResult(const optimizer::FootprintNlpConfig& config,
                            optimizer::OptimizationResult* result) {
  result->initial_guess_succeeded = true;
  result->status = "NLP solve started; no terminal status recorded yet";
  result->configured_max_iterations = config.max_iterations;
  result->configured_max_cpu_time = config.max_cpu_time;
  result->configured_tolerance = config.tolerance;
  result->configured_linear_solver = config.linear_solver;
  result->configured_hessian_approximation = "exact";
  result->configured_mu_strategy = "adaptive";
}

}  // namespace

PlanningPipeline::PlanningPipeline(PlanningConfiguration configuration)
    : configuration_(std::move(configuration)) {}

PlanningRunResult PlanningPipeline::Run(const scene::SceneSnapshot& snapshot,
                                        const std::string& mode) const {
  PlanningRunResult result;
  experiment::ExperimentLogger logger;
  std::string error;
  if (!logger.CreateRunDirectory(configuration_.results_root, mode,
                                 snapshot.task_id, &result.result_directory,
                                 &error)) {
    result.status = error;
    return result;
  }
  logger.WriteSceneCopy(result.result_directory, snapshot, &error);
  logger.WriteEffectiveParameters(result.result_directory, configuration_, &error);
  logger.WriteProgramVersion(result.result_directory, mode, &error);
  optimizer::WriteGeometryValidation(
      configuration_.vehicle,
      (std::filesystem::path(result.result_directory) /
       "geometry_validation.csv").string(), &error);

  if (!snapshot.Validate(configuration_.vehicle, &error)) {
    result.status = "Scene validation failed: " + error;
    logger.WriteRunStatus(result.result_directory, "scene_validation", false,
                          result.status, 0.0, &error);
    return result;
  }
  result.scene_valid = true;
  logger.WriteRunStatus(result.result_directory, "scene_validation", true,
                        "valid", 0.0, &error);

  ParkingPlannerConfig planner_config = configuration_.planner;
  planner_config.xy_bounds = snapshot.bounds;
  planning::HybridAStar planner(planner_config, configuration_.vehicle,
                                snapshot.obstacle_polygons());
  const auto hybrid_begin = std::chrono::steady_clock::now();
  result.hybrid_succeeded = planner.Plan(
      snapshot.start.x, snapshot.start.y, snapshot.start.yaw,
      snapshot.goal.x, snapshot.goal.y, snapshot.goal.yaw,
      &result.hybrid_path);
  result.hybrid_elapsed = SecondsSince(hybrid_begin);
  logger.WriteHybridPath(result.result_directory, result.hybrid_path, &error);
  logger.WriteRunStatus(result.result_directory, "hybrid_astar",
                        result.hybrid_succeeded,
                        result.hybrid_succeeded ? "succeeded" : "failed",
                        result.hybrid_elapsed, &error);
  if (!result.hybrid_succeeded) {
    result.status = "Hybrid A* failed";
    return result;
  }

  result.initial_guess_succeeded = optimizer::BuildInitialGuess(
      result.hybrid_path, configuration_.vehicle, configuration_.initial_guess,
      &result.initial_guess, &result.initial_diagnostics, &error,
      &result.initial_trace);
  logger.WriteDenseTrajectory(result.result_directory, result.initial_trace, &error);
  optimizer::SaveInitialGuessDiagnostics(
      (std::filesystem::path(result.result_directory) /
       "initial_guess_diagnostics.txt").string(),
      result.initial_diagnostics, &error);
  logger.WriteRunStatus(result.result_directory, "initial_guess",
                        result.initial_guess_succeeded,
                        result.initial_guess_succeeded ? "succeeded" : error,
                        0.0, &error);
  if (!result.initial_guess_succeeded) {
    result.status = "Initial guess failed: " + error;
    result.optimization.status = result.status;
    optimizer::SaveOptimizationResults(result.result_directory,
                                       result.initial_guess,
                                       result.optimization, &error);
    return result;
  }

  optimizer::OptimizationResult pending;
  ConfigurePendingResult(configuration_.nlp, &pending);
  optimizer::SaveOptimizationResults(result.result_directory,
                                     result.initial_guess, pending, &error);
  logger.WriteInitialGuess(result.result_directory, result.initial_guess,
                           result.initial_trace, &error);

  const std::string ipopt_log =
      (std::filesystem::path(result.result_directory) / "ipopt.log").string();
  result.optimization = optimizer::SolveFootprintNlp(
      result.initial_guess, configuration_.vehicle,
      snapshot.obstacle_polygons(), snapshot.bounds, configuration_.nlp,
      ipopt_log);
  optimizer::SaveOptimizationResults(result.result_directory,
                                     result.initial_guess,
                                     result.optimization, &error);
  logger.WriteInitialGuess(result.result_directory, result.initial_guess,
                           result.initial_trace, &error);
  optimizer::WriteCollisionValidation(
      (std::filesystem::path(result.result_directory) /
       "collision_validation.txt").string(),
      result.optimization.trajectory, configuration_.vehicle,
      snapshot.obstacle_polygons(), snapshot.bounds, &error);
  logger.WriteRunStatus(result.result_directory, "nlp",
                        result.optimization.success,
                        result.optimization.status,
                        result.optimization.solve_time, &error);
  result.success = result.optimization.success;
  result.status = result.optimization.status;
  return result;
}

}  // namespace planning_platform
