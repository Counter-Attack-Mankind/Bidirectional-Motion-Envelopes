#pragma once

#include <string>

#include "planning/hybrid_astar/hybrid_a_star.h"
#include "optimizer/initial_guess.h"
#include "planning/planning_config_loader.h"
#include "scene/scene_snapshot.h"

namespace planning_platform {
struct PlanningRunResult;
}

namespace experiment {

class ExperimentLogger {
 public:
  bool CreateRunDirectory(const std::string& root, const std::string& mode,
                          int task_id, std::string* directory,
                          std::string* error) const;
  bool WriteSceneCopy(const std::string& directory,
                      const scene::SceneSnapshot& snapshot,
                      std::string* error) const;
  bool WriteEffectiveParameters(
      const std::string& directory,
      const planning_platform::PlanningConfiguration& configuration,
      std::string* error) const;
  bool WriteSummary(const std::string& directory, const std::string& mode,
                    int task_id,
                    const planning_platform::PlanningRunResult& result,
                    std::string* error) const;
  bool WriteHybridPath(const std::string& directory,
                       const planning::HybridAStartResult& path,
                       std::string* error) const;
  bool WriteDenseTrajectory(const std::string& directory,
                            const optimizer::InitialGuessTrace& trace,
                            std::string* error) const;
  bool WriteInitialGuess(const std::string& directory,
                         const optimizer::Trajectory& trajectory,
                         const optimizer::InitialGuessTrace& trace,
                         std::string* error) const;
};

}  // namespace experiment
