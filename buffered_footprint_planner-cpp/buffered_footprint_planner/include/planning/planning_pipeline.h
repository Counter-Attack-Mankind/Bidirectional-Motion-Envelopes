#pragma once

#include <functional>
#include <string>

#include "planning/hybrid_astar/hybrid_a_star.h"
#include "optimizer/initial_guess.h"
#include "optimizer/trajectory.h"
#include "planning/planning_config_loader.h"
#include "scene/scene_snapshot.h"

namespace planning_platform {

enum class PlanningStage {
  kHybridAStar,
  kInitialGuess,
  kNlpOptimization,
  kComplete,
};

enum class PlanningStageState { kRunning, kSucceeded, kFailed };

struct PlanningStageUpdate {
  PlanningStage stage = PlanningStage::kHybridAStar;
  PlanningStageState state = PlanningStageState::kRunning;
  double stage_elapsed = 0.0;
  double total_elapsed = 0.0;
  std::string message;
};

struct PlanningRunResult {
  bool scene_valid = false;
  bool hybrid_succeeded = false;
  bool hybrid_timed_out = false;
  bool initial_guess_succeeded = false;
  bool success = false;
  std::string status;
  std::string result_directory;
  double hybrid_elapsed = 0.0;
  planning::HybridAStarStatistics hybrid_statistics;
  double initial_guess_elapsed = 0.0;
  planning::HybridAStartResult hybrid_path;
  optimizer::Trajectory initial_guess;
  optimizer::InitialGuessDiagnostics initial_diagnostics;
  optimizer::InitialGuessTrace initial_trace;
  optimizer::OptimizationResult optimization;
};

class PlanningPipeline {
 public:
  using HybridCompleteCallback =
      std::function<void(const planning::HybridAStartResult&, double)>;
  using HybridProgressCallback = planning::HybridAStar::ProgressCallback;
  using CancellationCallback = planning::HybridAStar::CancellationCallback;
  using StageCallback = std::function<void(PlanningStageUpdate)>;

  explicit PlanningPipeline(PlanningConfiguration configuration);

  PlanningRunResult Run(const scene::SceneSnapshot& snapshot,
                        const std::string& mode,
                        HybridCompleteCallback hybrid_complete = {},
                        HybridProgressCallback hybrid_progress = {},
                        CancellationCallback cancellation = {},
                        StageCallback stage_callback = {}) const;

 private:
  PlanningConfiguration configuration_;
};

}  // namespace planning_platform
