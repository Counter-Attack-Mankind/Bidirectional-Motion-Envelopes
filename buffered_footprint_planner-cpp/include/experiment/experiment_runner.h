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
               std::string* error) const;

 private:
  planning_platform::PlanningConfiguration configuration_;
};

}  // namespace experiment
