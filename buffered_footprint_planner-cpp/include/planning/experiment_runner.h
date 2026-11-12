#pragma once

#include <string>

#include "planning/planning_config_loader.h"
#include "planning/planning_pipeline.h"

namespace experiment {

class ExperimentRunner {
 public:
  explicit ExperimentRunner(
      planning_platform::PlanningConfiguration configuration);

  bool RunTask(const std::string& tasks_root, int task_id,
               planning_platform::PlanningRunResult* result,
               std::string* error,
               planning_platform::PlanningPipeline::HybridCompleteCallback
                   hybrid_complete = {},
               planning_platform::PlanningPipeline::HybridProgressCallback
                   hybrid_progress = {},
               planning_platform::PlanningPipeline::StageCallback
                   stage_callback = {}) const;

 private:
  planning_platform::PlanningConfiguration configuration_;
};

}  // namespace experiment
