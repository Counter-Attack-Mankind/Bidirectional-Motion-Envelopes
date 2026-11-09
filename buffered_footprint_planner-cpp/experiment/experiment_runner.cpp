#include "experiment/experiment_runner.h"

#include <filesystem>

#include "scene/scene_storage.h"

namespace experiment {

ExperimentRunner::ExperimentRunner(
    planning_platform::PlanningConfiguration configuration)
    : configuration_(std::move(configuration)) {}

bool ExperimentRunner::RunTask(
    const std::string& tasks_root, int task_id,
    planning_platform::PlanningRunResult* result, std::string* error) const {
  if (!result || task_id <= 0) {
    if (error) *error = "Task ID must be a positive integer";
    return false;
  }
  const std::filesystem::path scene_file =
      std::filesystem::path(tasks_root) /
      ("task" + std::to_string(task_id)) / "scene.yaml";
  scene::SceneSnapshot snapshot;
  if (!scene::SceneStorage::Load(scene_file.string(), &snapshot, error)) {
    return false;
  }
  if (snapshot.task_id != task_id) {
    if (error) *error = "Scene task_id does not match requested task";
    return false;
  }
  if (!snapshot.Validate(configuration_.vehicle, error)) return false;
  auto configuration = configuration_;
  configuration.planner.xy_bounds = snapshot.bounds;
  planning_platform::PlanningPipeline pipeline(std::move(configuration));
  *result = pipeline.Run(snapshot, "offline");
  return result->success;
}

}  // namespace experiment
