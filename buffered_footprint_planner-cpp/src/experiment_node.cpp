#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <utility>

#include <ros/ros.h>

#include "planning/experiment_runner.h"
#include "planning/planning_config_loader.h"
#include "scene/scene_storage.h"
#include "visualize/scene_visualizer.h"
#include "visualize/trajectory_playback_visualizer.h"

namespace {

const char* StageName(planning_platform::PlanningStage stage) {
  switch (stage) {
    case planning_platform::PlanningStage::kHybridAStar: return "Hybrid A*";
    case planning_platform::PlanningStage::kInitialGuess: return "Initial guess";
    case planning_platform::PlanningStage::kNlpOptimization:
      return "NLP optimization";
    case planning_platform::PlanningStage::kComplete: return "Planning";
  }
  return "Planning";
}

std::string StageText(const planning_platform::PlanningStageUpdate& update,
                      const ros::WallTime& begin) {
  double elapsed = update.stage_elapsed;
  if (update.state == planning_platform::PlanningStageState::kRunning) {
    elapsed = (ros::WallTime::now() - begin).toSec();
  }
  std::ostringstream text;
  text << std::fixed << std::setprecision(2);
  if (update.stage == planning_platform::PlanningStage::kComplete &&
      update.state == planning_platform::PlanningStageState::kSucceeded) {
    text << "Planning succeeded | total: " << update.total_elapsed << " s";
  } else if (update.state == planning_platform::PlanningStageState::kFailed) {
    text << StageName(update.stage) << " failed: " << elapsed << " s";
  } else {
    text << StageName(update.stage) << ": " << elapsed << " s";
  }
  return text.str();
}

}  // namespace

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

  std::mutex search_progress_mutex;
  std::mutex visualization_mutex;
  std::shared_ptr<planning::HybridSearchProgress> pending_search_progress;
  std::shared_ptr<planning::HybridSearchProgress> latest_search_progress;
  std::shared_ptr<planning::HybridAStartResult> completed_hybrid_path;
  std::shared_ptr<planning_platform::PlanningStageUpdate> active_stage;
  ros::WallTime active_stage_begin;
  std::deque<std::pair<planning_platform::PlanningStageUpdate, ros::WallTime>>
      pending_stage_updates;
  bool search_finished = false;
  {
    std::lock_guard<std::mutex> visualization_lock(visualization_mutex);
    visualizer.QueueScene(snapshot);
    visualizer.Trigger();
  }

  // Service visualization timers and deliver the Hybrid stage marker while
  // the main thread continues through initial-guess construction and IPOPT.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool show_hybrid_search = true;
  private_node.param("visualization/show_hybrid_search",
                     show_hybrid_search, true);
  // Keep the latched MarkerArray useful for RViz instances that connect after
  // the first publication. Once Hybrid A* succeeds, include its final path in
  // the same batch while initial-guess construction and IPOPT continue.
  ros::Timer scene_republish_timer = private_node.createTimer(
      ros::Duration(1.0),
      [&](const ros::TimerEvent&) {
        std::shared_ptr<planning::HybridAStartResult> hybrid_path;
        std::shared_ptr<planning::HybridSearchProgress> search_progress;
        {
          std::lock_guard<std::mutex> lock(search_progress_mutex);
          hybrid_path = completed_hybrid_path;
          search_progress = latest_search_progress;
        }
        std::lock_guard<std::mutex> visualization_lock(visualization_mutex);
        visualizer.QueueScene(snapshot);
        if (hybrid_path) {
          visualizer.QueueHybridPath(*hybrid_path);
        } else if (search_progress) {
          visualizer.QueueHybridSearch(*search_progress);
        }
        visualizer.Trigger();
      });
  ros::Timer search_progress_timer;
  if (show_hybrid_search) {
    search_progress_timer = private_node.createTimer(
        ros::Duration(0.05),
        [&](const ros::TimerEvent&) {
          std::shared_ptr<planning::HybridSearchProgress> progress;
          {
            std::lock_guard<std::mutex> lock(search_progress_mutex);
            if (search_finished) return;
            progress = std::move(pending_search_progress);
          }
          if (!progress) return;
          std::lock_guard<std::mutex> visualization_lock(visualization_mutex);
          {
            std::lock_guard<std::mutex> lock(search_progress_mutex);
            if (search_finished) return;
          }
          // Publish static and dynamic markers atomically as one MarkerArray,
          // so a late RViz subscriber never receives search data without the
          // map, obstacles, start, and goal.
          visualizer.QueueScene(snapshot);
          visualizer.QueueHybridSearch(*progress);
          std::shared_ptr<planning_platform::PlanningStageUpdate> stage;
          ros::WallTime stage_begin;
          {
            std::lock_guard<std::mutex> lock(search_progress_mutex);
            stage = active_stage;
            stage_begin = active_stage_begin;
          }
          if (stage) {
            visualizer.QueuePlanningStage(snapshot.bounds,
                                          StageText(*stage, stage_begin));
          }
          visualizer.Trigger();
        });
  }
  ros::Timer stage_timer = private_node.createTimer(
      ros::Duration(0.1),
      [&](const ros::TimerEvent&) {
        std::shared_ptr<planning_platform::PlanningStageUpdate> stage;
        std::shared_ptr<planning::HybridSearchProgress> search_progress;
        std::shared_ptr<planning::HybridAStartResult> hybrid_path;
        ros::WallTime stage_begin;
        {
          std::lock_guard<std::mutex> lock(search_progress_mutex);
          if (!pending_stage_updates.empty()) {
            active_stage =
                std::make_shared<planning_platform::PlanningStageUpdate>(
                    std::move(pending_stage_updates.front().first));
            active_stage_begin = pending_stage_updates.front().second;
            pending_stage_updates.pop_front();
          }
          stage = active_stage;
          stage_begin = active_stage_begin;
          search_progress = latest_search_progress;
          hybrid_path = completed_hybrid_path;
        }
        if (!stage) return;
        std::lock_guard<std::mutex> visualization_lock(visualization_mutex);
        visualizer.QueueScene(snapshot);
        if (hybrid_path) {
          visualizer.QueueHybridPath(*hybrid_path);
        } else if (search_progress) {
          visualizer.QueueHybridSearch(*search_progress);
        }
        visualizer.QueuePlanningStage(snapshot.bounds,
                                      StageText(*stage, stage_begin));
        visualizer.Trigger();
      });

  experiment::ExperimentRunner runner(configuration);
  planning_platform::PlanningRunResult result;
  const bool success = runner.RunTask(
      tasks_root, task_id, &result, &error,
      [&](const planning::HybridAStartResult& path, double elapsed) {
        {
          std::lock_guard<std::mutex> lock(search_progress_mutex);
          search_finished = true;
          pending_search_progress.reset();
          completed_hybrid_path =
              std::make_shared<planning::HybridAStartResult>(path);
        }
        std::lock_guard<std::mutex> visualization_lock(visualization_mutex);
        visualizer.QueueScene(snapshot);
        visualizer.QueueHybridPath(path);
        visualizer.Trigger();
        ROS_INFO("Hybrid A* path published after %.3f s; continuing with "
                 "initial guess and NLP", elapsed);
      },
      show_hybrid_search
          ? planning_platform::PlanningPipeline::HybridProgressCallback(
                [&](planning::HybridSearchProgress progress) {
                  std::lock_guard<std::mutex> lock(search_progress_mutex);
                  if (search_finished) return;
                  auto progress_snapshot =
                      std::make_shared<planning::HybridSearchProgress>(
                          std::move(progress));
                  pending_search_progress = progress_snapshot;
                  latest_search_progress = std::move(progress_snapshot);
                })
          : planning_platform::PlanningPipeline::HybridProgressCallback{},
      [&](planning_platform::PlanningStageUpdate update) {
        std::lock_guard<std::mutex> lock(search_progress_mutex);
        pending_stage_updates.emplace_back(std::move(update),
                                           ros::WallTime::now());
      });
  {
    std::lock_guard<std::mutex> lock(search_progress_mutex);
    if (!pending_stage_updates.empty()) {
      active_stage =
          std::make_shared<planning_platform::PlanningStageUpdate>(
              std::move(pending_stage_updates.back().first));
      active_stage_begin = pending_stage_updates.back().second;
      pending_stage_updates.clear();
    }
  }
  scene_republish_timer.stop();
  stage_timer.stop();
  ROS_INFO("Initial guess %s; NLP nodes: %zu; elapsed: %.3f s",
           result.initial_guess_succeeded ? "succeeded" : "failed",
           result.initial_guess.nodes.size(), result.initial_guess_elapsed);
  if (result.initial_guess_succeeded) {
    ROS_INFO("IPOPT status: %s; iterations: %d; elapsed: %.3f s",
             result.optimization.status.c_str(), result.optimization.iterations,
             result.optimization.solve_time);
  }
  if (!success) {
    const std::string message = result.status.empty() ? error : result.status;
    ROS_ERROR("Offline experiment failed: %s; max dynamics residual: %.3g; "
              "max OBCA violation: %.3g; results: %s",
              message.c_str(), result.optimization.max_dynamics_residual,
              result.optimization.max_obca_violation,
              result.result_directory.c_str());
  } else {
    ROS_INFO("Constraint validation passed; max dynamics residual: %.3g; "
             "max OBCA violation: %.3g",
             result.optimization.max_dynamics_residual,
             result.optimization.max_obca_violation);
    ROS_INFO("Offline experiment succeeded; results: %s",
             result.result_directory.c_str());
  }

  double playback_speed = 1.0;
  double playback_frequency_hz = 100.0;
  double swept_area_resolution = 0.005;
  private_node.param("visualization/playback_speed", playback_speed, 1.0);
  private_node.param("visualization/playback_frequency_hz",
                     playback_frequency_hz, 100.0);
  private_node.param("visualization/swept_area_resolution",
                     swept_area_resolution, 0.005);
  visualization::TrajectoryPlaybackVisualizer playback;
  bool playback_started = false;
  if (!playback.Initialize(private_node, &visualizer, configuration.vehicle,
                           playback_speed, playback_frequency_hz,
                           swept_area_resolution, &error)) {
    ROS_ERROR("Cannot initialize trajectory visualization: %s", error.c_str());
  } else if (result.success) {
    playback_started = playback.Start(result.optimization.trajectory, &error);
    if (!playback_started) {
      ROS_ERROR("Optimized trajectory visualization disabled: %s",
                error.c_str());
    }
  }
  {
    std::lock_guard<std::mutex> visualization_lock(visualization_mutex);
    visualizer.Trigger();
  }
  ROS_INFO("Experiment visualization remains active; press Ctrl+C to exit");

  ros::Timer republish_timer = private_node.createTimer(
      ros::Duration(1.0),
      [&](const ros::TimerEvent&) {
        std::shared_ptr<planning::HybridSearchProgress> search_progress;
        std::shared_ptr<planning_platform::PlanningStageUpdate> stage;
        ros::WallTime stage_begin;
        {
          std::lock_guard<std::mutex> lock(search_progress_mutex);
          search_progress = latest_search_progress;
          stage = active_stage;
          stage_begin = active_stage_begin;
        }
        std::lock_guard<std::mutex> visualization_lock(visualization_mutex);
        visualizer.QueueScene(snapshot);
        if (result.hybrid_succeeded) {
          visualizer.QueueHybridPath(result.hybrid_path);
        } else if (search_progress) {
          visualizer.QueueHybridSearch(*search_progress);
        }
        if (stage) {
          visualizer.QueuePlanningStage(snapshot.bounds,
                                        StageText(*stage, stage_begin));
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
