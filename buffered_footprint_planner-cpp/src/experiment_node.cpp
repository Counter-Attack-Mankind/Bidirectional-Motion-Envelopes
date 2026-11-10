#include <cstdlib>
#include <filesystem>
#include <string>

#include <ros/ros.h>

#include "experiment/experiment_runner.h"
#include "planning/planning_config_loader.h"
#include "scene/scene_storage.h"
#include "visualize/scene_visualizer.h"
#include "visualize/trajectory_playback_visualizer.h"

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
  visualization::SceneVisualizer visualizer;
  visualizer.Initialize(private_node, snapshot.frame_id,
                        configuration.vehicle);
  visualizer.QueueScene(snapshot);
  visualizer.Trigger();

  // Service visualization timers and deliver the Hybrid stage marker while
  // the main thread continues through initial-guess construction and IPOPT.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  experiment::ExperimentRunner runner(configuration);
  planning_platform::PlanningRunResult result;
  const bool success = runner.RunTask(
      tasks_root, task_id, &result, &error,
      [&](const planning::HybridAStartResult& path, double elapsed) {
        visualizer.QueueHybridPath(path);
        visualizer.Trigger();
        ROS_INFO("Hybrid A* path published after %.3f s; continuing with "
                 "initial guess and NLP", elapsed);
      });
  if (!success) {
    const std::string message = result.status.empty() ? error : result.status;
    ROS_ERROR("Offline experiment failed: %s; results: %s",
              message.c_str(), result.result_directory.c_str());
  } else {
    ROS_INFO("Offline experiment succeeded; results: %s",
             result.result_directory.c_str());
  }

  double playback_speed = 1.0;
  double swept_area_resolution = 0.005;
  private_node.param("visualization/playback_speed", playback_speed, 1.0);
  private_node.param("visualization/swept_area_resolution",
                     swept_area_resolution, 0.005);
  visualization::TrajectoryPlaybackVisualizer playback;
  bool playback_started = false;
  if (!playback.Initialize(private_node, &visualizer, configuration.vehicle,
                           playback_speed, swept_area_resolution, &error)) {
    ROS_ERROR("Cannot initialize trajectory visualization: %s", error.c_str());
  } else if (result.success) {
    playback_started = playback.Start(result.optimization.trajectory, &error);
    if (!playback_started) {
      ROS_ERROR("Optimized trajectory visualization disabled: %s",
                error.c_str());
    }
  }
  visualizer.Trigger();
  ROS_INFO("Experiment visualization remains active; press Ctrl+C to exit");

  ros::Timer republish_timer = private_node.createTimer(
      ros::Duration(1.0),
      [&](const ros::TimerEvent&) {
        visualizer.QueueScene(snapshot);
        if (result.hybrid_succeeded) {
          visualizer.QueueHybridPath(result.hybrid_path);
        }
        if (playback_started) {
          visualizer.QueueOptimizedTrajectory(
              result.optimization.trajectory, false);
          visualizer.QueueOptimizedConfigurationPoints(
              result.optimization.trajectory);
        }
        visualizer.Trigger();
      });
  ros::waitForShutdown();
  return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
