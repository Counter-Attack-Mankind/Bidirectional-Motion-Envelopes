#pragma once

#include <functional>
#include <string>

#include "hybrid_astar/hybrid_a_star.h"
#include "optimizer/initial_guess.h"
#include "optimizer/trajectory.h"
#include "planning/planning_config_loader.h"
#include "scene/scene_snapshot.h"

namespace planning_platform {

struct PlanningRunResult {
  bool scene_valid = false;
  bool hybrid_succeeded = false;
  bool hybrid_timed_out = false;
  bool initial_guess_succeeded = false;
  bool success = false;
  std::string status;
  std::string result_directory;
  double hybrid_elapsed = 0.0;
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

  explicit PlanningPipeline(PlanningConfiguration configuration);

  PlanningRunResult Run(const scene::SceneSnapshot& snapshot,
                        const std::string& mode,
                        HybridCompleteCallback hybrid_complete = {}) const;

 private:
  PlanningConfiguration configuration_;
};

}  // namespace planning_platform
