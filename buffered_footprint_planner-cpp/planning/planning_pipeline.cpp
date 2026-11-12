#include "planning/planning_pipeline.h"

#include <algorithm>
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
                                        CancellationCallback cancellation,
                                        StageCallback stage_callback) const {
  PlanningRunResult result;
  const auto pipeline_begin = std::chrono::steady_clock::now();
  auto notify_stage = [&](PlanningStage stage, PlanningStageState state,
                          double stage_elapsed, const std::string& message) {
    if (!stage_callback) return;
    stage_callback({stage, state, stage_elapsed, SecondsSince(pipeline_begin),
                    message});
  };
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
  bool hybrid_stage_started = false;
  double hybrid_stage_begin_elapsed = 0.0;
  planning::HybridSearchStage last_search_stage =
      planning::HybridSearchStage::kBuildingDpMap;
  notify_stage(PlanningStage::kDpMap, PlanningStageState::kRunning, 0.0,
               "DP Map");
  auto progress_forwarder =
      [&, hybrid_progress = std::move(hybrid_progress)](
          planning::HybridSearchProgress progress) mutable {
        last_search_stage = progress.stage;
        if (progress.stage == planning::HybridSearchStage::kHybridSearch &&
            !hybrid_stage_started) {
          hybrid_stage_started = true;
          hybrid_stage_begin_elapsed = progress.elapsed_seconds;
          notify_stage(PlanningStage::kHybridAStar,
                       PlanningStageState::kRunning, 0.0, "Hybrid A*");
        }
        if (hybrid_progress) hybrid_progress(std::move(progress));
      };
  result.hybrid_succeeded = planner.Plan(
      snapshot.start.x, snapshot.start.y, snapshot.start.yaw,
      snapshot.goal.x, snapshot.goal.y, snapshot.goal.yaw,
      &result.hybrid_path, std::move(progress_forwarder),
      std::move(cancellation));
  result.hybrid_elapsed = SecondsSince(hybrid_begin);
  result.hybrid_timed_out = planner.timed_out();
  if (!result.hybrid_succeeded) {
    result.status = result.hybrid_timed_out
        ? "Hybrid A* timeout after " + std::to_string(result.hybrid_elapsed) +
              " s (" + std::to_string(planner.timeout_seconds()) +
              " s limit)"
        : "Hybrid A* failed";
    const PlanningStage failed_stage =
        last_search_stage == planning::HybridSearchStage::kBuildingDpMap
            ? PlanningStage::kDpMap
            : PlanningStage::kHybridAStar;
    notify_stage(failed_stage, PlanningStageState::kFailed,
                 failed_stage == PlanningStage::kHybridAStar
                     ? std::max(0.0, result.hybrid_elapsed -
                                        hybrid_stage_begin_elapsed)
                     : result.hybrid_elapsed,
                 result.status);
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
  notify_stage(PlanningStage::kInitialGuess, PlanningStageState::kRunning,
               0.0, "Initial guess");
  result.initial_guess_succeeded = optimizer::BuildInitialGuess(
      result.hybrid_path, configuration_.vehicle, configuration_.initial_guess,
      snapshot.obstacle_polygons(), snapshot.bounds,
      &result.initial_guess, &result.initial_diagnostics, &error,
      &result.initial_trace);
  result.initial_guess_elapsed = SecondsSince(initial_begin);
  if (!result.initial_guess_succeeded) {
    result.status = "Initial guess failed: " + error;
    result.optimization.status = result.status;
    notify_stage(PlanningStage::kInitialGuess, PlanningStageState::kFailed,
                 result.initial_guess_elapsed, result.status);
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
  const auto nlp_begin = std::chrono::steady_clock::now();
  notify_stage(PlanningStage::kNlpOptimization,
               PlanningStageState::kRunning, 0.0, "NLP optimization");
  result.optimization = optimizer::SolveFootprintNlp(
      result.initial_guess, configuration_.vehicle,
      snapshot.obstacle_polygons(), snapshot.bounds, configuration_.nlp,
      ipopt_log);
  const double nlp_elapsed = SecondsSince(nlp_begin);
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
        snapshot.obstacle_polygons(), snapshot.bounds,
        configuration_.nlp.dmin, &error);
    if (!configuration_.nlp.save_debug_data) {
      std::filesystem::remove_all(debug_directory, filesystem_error);
    }
  } else {
    optimizer::SaveNlpFailureDiagnostics(debug_directory.string(),
                                         result.optimization, &error);
  }
  if (!result.success) {
    notify_stage(PlanningStage::kNlpOptimization,
                 PlanningStageState::kFailed, nlp_elapsed, result.status);
  }
  if (result.success) {
    notify_stage(PlanningStage::kComplete, PlanningStageState::kSucceeded,
                 nlp_elapsed, result.status);
  }
  logger.WriteSummary(result.result_directory, mode, snapshot.task_id, result,
                      &error);
  return result;
}

}  // namespace planning_platform
