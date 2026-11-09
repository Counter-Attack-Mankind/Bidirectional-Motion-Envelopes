#include <cstdlib>
#include <filesystem>
#include <string>

#include <ros/ros.h>

#include "experiment/experiment_runner.h"
#include "planning/planning_config_loader.h"
#include "scene/scene_storage.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_experiment");
  ros::NodeHandle private_node("~");
  int task_id = 0;
  std::string tasks_root;
  if (!private_node.getParam("task", task_id) || task_id <= 0 ||
      !private_node.getParam("tasks_root", tasks_root) || tasks_root.empty()) {
    ROS_ERROR("Offline experiment requires positive '~task' and '~tasks_root'");
    return EXIT_FAILURE;
  }
  const std::filesystem::path scene_file =
      std::filesystem::path(tasks_root) /
      ("task" + std::to_string(task_id)) / "scene.yaml";
  scene::SceneSnapshot snapshot;
  std::string error;
  if (!scene::SceneStorage::Load(scene_file.string(), &snapshot, &error)) {
    ROS_ERROR("Offline scene load failed (no env.yaml fallback): %s",
              error.c_str());
    return EXIT_FAILURE;
  }
  planning_platform::PlanningConfiguration configuration;
  if (!planning_platform::LoadPlanningConfiguration(
          private_node, snapshot.bounds, &configuration, &error)) {
    ROS_ERROR("Cannot load experiment parameters: %s", error.c_str());
    return EXIT_FAILURE;
  }
  experiment::ExperimentRunner runner(configuration);
  planning_platform::PlanningRunResult result;
  const bool success = runner.RunTask(tasks_root, task_id, &result, &error);
  if (!success) {
    const std::string message = result.status.empty() ? error : result.status;
    ROS_ERROR("Offline experiment failed: %s; results: %s",
              message.c_str(), result.result_directory.c_str());
    return EXIT_FAILURE;
  }
  ROS_INFO("Offline experiment succeeded; results: %s",
           result.result_directory.c_str());
  return EXIT_SUCCESS;
}
