#include "planning/planning_pipeline.h"

#include <chrono>
#include <filesystem>
#include <utility>

#include "planning/experiment_logger.h"
#include "optimizer/footprint_nlp.h"

namespace planning_platform {
namespace {

double SecondsSince(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

}  // namespace

PlanningPipeline::PlanningPipeline(PlanningConfiguration configuration)
    : configuration_(std::move(configuration)) {}

PlanningRunResult PlanningPipeline::Run(const scene::SceneSnapshot& snapshot,
                                        const std::string& mode,
                                        HybridCompleteCallback hybrid_complete,
                                        HybridProgressCallback hybrid_progress,
                                        CancellationCallback cancellation) const {
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
  logger.WriteSummary(result.result_directory, mode, snapshot.task_id, result,
                      &error);

  if (!snapshot.Validate(configuration_.vehicle, &error)) {
    result.status = "Scene validation failed: " + error;
    logger.WriteSummary(result.result_directory, mode, snapshot.task_id, result,
                        &error);
    return result;
  }
  result.scene_valid = true;
  logger.WriteSummary(result.result_directory, mode, snapshot.task_id, result,
                      &error);

  ParkingPlannerConfig planner_config = configuration_.planner;
  planner_config.xy_bounds = snapshot.bounds;
  planning::HybridAStar planner(planner_config, configuration_.vehicle,
                                snapshot.obstacle_polygons(),
                                configuration_.nlp.save_debug_data);
  const auto hybrid_begin = std::chrono::steady_clock::now();
  result.hybrid_succeeded = planner.Plan(
      snapshot.start.x, snapshot.start.y, snapshot.start.yaw,
      snapshot.goal.x, snapshot.goal.y, snapshot.goal.yaw,
      &result.hybrid_path, std::move(hybrid_progress),
      std::move(cancellation));
  result.hybrid_elapsed = SecondsSince(hybrid_begin);
  result.hybrid_timed_out = planner.timed_out();
  if (!result.hybrid_succeeded) {
    result.status = result.hybrid_timed_out
        ? "Hybrid A* timeout after " + std::to_string(result.hybrid_elapsed) +
              " s (" + std::to_string(planner.timeout_seconds()) +
              " s limit)"
        : "Hybrid A* failed";
    logger.WriteSummary(result.result_directory, mode, snapshot.task_id, result,
                        &error);
    return result;
  }
  logger.WriteHybridPath(result.result_directory, result.hybrid_path, &error);
  result.status = "Hybrid A* succeeded";
  logger.WriteSummary(result.result_directory, mode, snapshot.task_id, result,
                      &error);
  if (hybrid_complete) {
    hybrid_complete(result.hybrid_path, result.hybrid_elapsed);
  }

  const auto initial_begin = std::chrono::steady_clock::now();
  result.initial_guess_succeeded = optimizer::BuildInitialGuess(
      result.hybrid_path, configuration_.vehicle, configuration_.initial_guess,
      &result.initial_guess, &result.initial_diagnostics, &error,
      &result.initial_trace);
  result.initial_guess_elapsed = SecondsSince(initial_begin);
  if (!result.initial_guess_succeeded) {
    result.status = "Initial guess failed: " + error;
    result.optimization.status = result.status;
    logger.WriteSummary(result.result_directory, mode, snapshot.task_id, result,
                        &error);
    return result;
  }
  logger.WriteInitialGuess(result.result_directory, result.initial_guess,
                           result.initial_trace, &error);
  result.status = "Initial guess succeeded";
  logger.WriteSummary(result.result_directory, mode, snapshot.task_id, result,
                      &error);

  const std::filesystem::path debug_directory =
      std::filesystem::path(result.result_directory) / "debug";
  std::error_code filesystem_error;
  std::filesystem::create_directories(debug_directory, filesystem_error);
  if (configuration_.nlp.save_debug_data) {
    logger.WriteDenseTrajectory(debug_directory.string(), result.initial_trace,
                                &error);
  }
  const std::string ipopt_log =
      (debug_directory / "ipopt.log").string();
  result.optimization = optimizer::SolveFootprintNlp(
      result.initial_guess, configuration_.vehicle,
      snapshot.obstacle_polygons(), snapshot.bounds, configuration_.nlp,
      ipopt_log);
  result.success = result.optimization.success;
  result.status = !result.success && result.optimization.solver_succeeded
      ? "NLP post-solve validation failed after " + result.optimization.status
      : result.optimization.status;
  if (result.success) {
    optimizer::SaveSuccessfulOptimizationResults(
        result.result_directory, result.optimization, &error);
    optimizer::WriteCollisionValidation(
        (std::filesystem::path(result.result_directory) /
         "collision_validation.txt").string(),
        result.optimization.trajectory, configuration_.vehicle,
        snapshot.obstacle_polygons(), snapshot.bounds, &error);
    if (!configuration_.nlp.save_debug_data) {
      std::filesystem::remove_all(debug_directory, filesystem_error);
    }
  } else {
    optimizer::SaveNlpFailureDiagnostics(debug_directory.string(),
                                         result.optimization, &error);
  }
  logger.WriteSummary(result.result_directory, mode, snapshot.task_id, result,
                      &error);
  return result;
}

}  // namespace planning_platform
