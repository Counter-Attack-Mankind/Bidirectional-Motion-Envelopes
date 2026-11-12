#include <cstdlib>
#include <atomic>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "buffered_footprint_planner/SetObjectPose.h"
#include "scene/scenario_manager.h"
#include "planning/planning_config_loader.h"
#include "planning/planning_pipeline.h"
#include "visualize/scene_visualizer.h"
#include "visualize/trajectory_playback_visualizer.h"

namespace {

const char* StageName(planning_platform::PlanningStage stage) {
  switch (stage) {
    case planning_platform::PlanningStage::kDpMap: return "DP Map";
    case planning_platform::PlanningStage::kHybridAStar: return "Hybrid A*";
    case planning_platform::PlanningStage::kInitialGuess: return "Initial guess";
    case planning_platform::PlanningStage::kNlpOptimization:
      return "NLP optimization";
    case planning_platform::PlanningStage::kComplete: return "Planning";
  }
  return "Planning";
}

class PlanningNode {
 public:
  PlanningNode() : private_node_("~") {}
  ~PlanningNode() {
    cancel_planning_ = true;
    if (planning_thread_.joinable()) planning_thread_.join();
  }

  bool Initialize() {
    if (!scenario_.Load(private_node_)) return false;
    std::string error;
    if (!planning_platform::LoadPlanningConfiguration(
            private_node_, scenario_.bounds(), &configuration_, &error)) {
      ROS_ERROR("Cannot load planning configuration: %s", error.c_str());
      return false;
    }
    private_node_.param("vehicle/id", vehicle_id_, std::string("vehicle_1"));
    visualizer_.Initialize(private_node_, scenario_.frame_id(),
                           configuration_.vehicle);
    double playback_speed = 1.0;
    double swept_area_resolution = 0.005;
    private_node_.param("visualization/playback_speed", playback_speed, 1.0);
    private_node_.param("visualization/swept_area_resolution",
                        swept_area_resolution, 0.005);
    private_node_.param("visualization/show_hybrid_search",
                        show_hybrid_search_, true);
    if (!playback_.Initialize(private_node_, &visualizer_,
                              configuration_.vehicle, playback_speed,
                              swept_area_resolution, &error)) {
      ROS_ERROR("Cannot initialize live trajectory visualization: %s",
                error.c_str());
      return false;
    }
    command_client_ = private_node_.serviceClient<
        buffered_footprint_planner::SetObjectPose>(
        "/simulation/set_object_pose");
    if (!command_client_.waitForExistence(ros::Duration(10.0))) {
      ROS_ERROR("Simulation scene service is unavailable");
      return false;
    }
    lock_subscriber_ = private_node_.subscribe(
        "/simulation/start_locked", 1, &PlanningNode::LockCallback, this);
    result_timer_ = private_node_.createTimer(
        ros::Duration(0.05), &PlanningNode::ProcessPlanningEvents, this);
    ROS_INFO("Live planner ready; fixed goal=(%.3f, %.3f, %.3f rad)",
             scenario_.goal().x, scenario_.goal().y, scenario_.goal().yaw);
    return true;
  }

 private:
  struct StageEvent {
    planning_platform::PlanningStageUpdate update;
    ros::WallTime received;
    std::uint64_t generation = 0;
  };

  bool Command(std::uint8_t action, const std::string& object_id,
               buffered_footprint_planner::SetObjectPose::Response* response) {
    buffered_footprint_planner::SetObjectPose service;
    service.request.action = action;
    service.request.object_id = object_id;
    if (!command_client_.call(service)) return false;
    if (response) *response = service.response;
    return service.response.success;
  }

  bool AcquireFrozenSnapshot(scene::SceneSnapshot* snapshot,
                             std::string* error) {
    buffered_footprint_planner::SetObjectPose::Response response;
    if (!Command(buffered_footprint_planner::SetObjectPoseRequest::BEGIN_PLANNING,
                 std::string(), &response)) {
      if (error) *error = response.message.empty() ?
          "Cannot freeze simulation scene" : response.message;
      return false;
    }
    scene_locked_ = true;
    if (!Command(buffered_footprint_planner::SetObjectPoseRequest::QUERY_POSE,
                 vehicle_id_, &response)) {
      if (error) *error = "Cannot query current vehicle pose";
      return false;
    }
    const scene::Pose2d start{response.x, response.y, response.yaw};
    for (const auto& obstacle : scenario_.obstacles()) {
      if (!Command(buffered_footprint_planner::SetObjectPoseRequest::QUERY_POSE,
                   obstacle.id, &response) ||
          !scenario_.SetObstaclePose(obstacle.id, response.x, response.y,
                                     response.yaw)) {
        if (error) *error = "Cannot query current obstacle '" + obstacle.id + "'";
        return false;
      }
    }
    *snapshot = scenario_.Snapshot(start, 0);
    return true;
  }

  void ReleaseScene() {
    if (!scene_locked_) return;
    buffered_footprint_planner::SetObjectPose::Response response;
    if (!Command(buffered_footprint_planner::SetObjectPoseRequest::END_PLANNING,
                 std::string(), &response)) {
      ROS_ERROR("Cannot release simulation scene: %s", response.message.c_str());
    }
    scene_locked_ = false;
  }

  void LockCallback(const std_msgs::Bool::ConstPtr& message) {
    if (!message->data) {
      last_locked_ = false;
      cancel_planning_ = true;
      ++scene_generation_;
      playback_.StopAndClear();
      stage_active_ = false;
      {
        std::lock_guard<std::mutex> lock(event_mutex_);
        pending_stage_updates_.clear();
      }
      return;
    }
    const bool rising_edge = message->data && !last_locked_;
    last_locked_ = message->data;
    if (!rising_edge || planning_.exchange(true)) return;
    scene::SceneSnapshot snapshot;
    std::string error;
    if (!AcquireFrozenSnapshot(&snapshot, &error)) {
      ROS_ERROR("Cannot acquire live scene snapshot: %s", error.c_str());
      ReleaseScene();
      planning_ = false;
      return;
    }
    visualizer_.ClearPlanningResults();
    visualizer_.QueueGoal(snapshot.goal);
    visualizer_.Trigger();
    if (planning_thread_.joinable()) planning_thread_.join();
    const std::uint64_t generation = scene_generation_.load();
    cancel_planning_ = false;
    planning_thread_ = std::thread(
        [this, snapshot = std::move(snapshot), generation]() mutable {
          planning_platform::PlanningPipeline pipeline(configuration_);
          auto result = std::make_shared<planning_platform::PlanningRunResult>(
              pipeline.Run(
                  snapshot, "live",
                  [this, generation](const planning::HybridAStartResult& path,
                                     double elapsed) {
                    std::lock_guard<std::mutex> lock(event_mutex_);
                    pending_hybrid_ =
                        std::make_shared<planning::HybridAStartResult>(path);
                    pending_hybrid_generation_ = generation;
                    ROS_INFO("Hybrid A* completed after %.3f s; queued for "
                             "immediate RViz publication", elapsed);
                  },
                  show_hybrid_search_
                      ? planning_platform::PlanningPipeline::HybridProgressCallback(
                            [this, generation](
                                planning::HybridSearchProgress progress) {
                              std::lock_guard<std::mutex> lock(event_mutex_);
                              pending_hybrid_progress_ =
                                  std::make_shared<planning::HybridSearchProgress>(
                                      std::move(progress));
                              pending_progress_generation_ = generation;
                            })
                      : planning_platform::PlanningPipeline::HybridProgressCallback{},
                  [this, generation]() {
                    return cancel_planning_.load() || !ros::ok() ||
                           generation != scene_generation_.load();
                  },
                  [this, generation](
                      planning_platform::PlanningStageUpdate update) {
                    std::lock_guard<std::mutex> lock(event_mutex_);
                    pending_stage_updates_.push_back(
                        {std::move(update), ros::WallTime::now(), generation});
                  }));
          ReleaseScene();
          {
            std::lock_guard<std::mutex> lock(event_mutex_);
            completed_result_ = std::move(result);
            completed_generation_ = generation;
          }
          planning_ = false;
        });
  }

  void ProcessPlanningEvents(const ros::TimerEvent&) {
    std::shared_ptr<planning::HybridSearchProgress> progress;
    std::shared_ptr<planning::HybridAStartResult> hybrid;
    std::shared_ptr<planning_platform::PlanningRunResult> result;
    std::uint64_t hybrid_generation = 0;
    std::uint64_t progress_generation = 0;
    std::uint64_t result_generation = 0;
    StageEvent stage_event;
    bool has_stage_event = false;
    {
      std::lock_guard<std::mutex> lock(event_mutex_);
      progress = std::move(pending_hybrid_progress_);
      progress_generation = pending_progress_generation_;
      hybrid = std::move(pending_hybrid_);
      hybrid_generation = pending_hybrid_generation_;
      result = std::move(completed_result_);
      result_generation = completed_generation_;
      const ros::WallTime now = ros::WallTime::now();
      if (!pending_stage_updates_.empty() &&
          (now - last_stage_publish_).toSec() >= 0.1) {
        stage_event = std::move(pending_stage_updates_.front());
        pending_stage_updates_.pop_front();
        has_stage_event = true;
      }
    }
    const std::uint64_t current_generation = scene_generation_.load();
    if (has_stage_event && stage_event.generation == current_generation) {
      active_stage_ = std::move(stage_event.update);
      stage_active_ = true;
      if (active_stage_.state ==
          planning_platform::PlanningStageState::kRunning) {
        active_stage_begin_ = stage_event.received;
      }
    }
    const ros::WallTime now = ros::WallTime::now();
    if (stage_active_ &&
        (now - last_stage_publish_).toSec() >= 0.1) {
      double elapsed = active_stage_.stage_elapsed;
      if (active_stage_.state ==
          planning_platform::PlanningStageState::kRunning) {
        elapsed = (now - active_stage_begin_).toSec();
      }
      std::ostringstream text;
      text << std::fixed << std::setprecision(2);
      if (active_stage_.stage == planning_platform::PlanningStage::kComplete &&
          active_stage_.state ==
              planning_platform::PlanningStageState::kSucceeded) {
        text << "Planning succeeded | total: "
             << active_stage_.total_elapsed << " s";
      } else if (active_stage_.state ==
                 planning_platform::PlanningStageState::kFailed) {
        text << StageName(active_stage_.stage) << " failed: " << elapsed
             << " s";
      } else {
        text << StageName(active_stage_.stage) << ": " << elapsed << " s";
      }
      visualizer_.QueuePlanningStage(scenario_.bounds(), text.str());
      visualizer_.Trigger();
      last_stage_publish_ = now;
    }
    if (progress && progress_generation == current_generation) {
      visualizer_.QueueHybridSearch(*progress);
      visualizer_.Trigger();
    }
    if (hybrid && hybrid_generation == current_generation) {
      visualizer_.QueueHybridPath(*hybrid);
      visualizer_.Trigger();
    }
    if (!result || result_generation != current_generation) return;
    if (!result->hybrid_succeeded) {
      playback_.StopAndClear();
      ROS_ERROR("Hybrid A* %s after %.3f s; results: %s",
                result->hybrid_timed_out ? "timed out" : "failed",
                result->hybrid_elapsed, result->result_directory.c_str());
      return;
    }
    ROS_INFO("Initial guess %s; NLP nodes: %zu; elapsed: %.3f s",
             result->initial_guess_succeeded ? "succeeded" : "failed",
             result->initial_guess.nodes.size(), result->initial_guess_elapsed);
    if (result->initial_guess_succeeded) {
      ROS_INFO("IPOPT status: %s; iterations: %d; elapsed: %.3f s",
               result->optimization.status.c_str(),
               result->optimization.iterations,
               result->optimization.solve_time);
    }
    if (result->success) {
      ROS_INFO("Constraint validation passed; max dynamics residual: %.3g; "
               "max OBCA violation: %.3g",
               result->optimization.max_dynamics_residual,
               result->optimization.max_obca_violation);
      std::string error;
      if (!playback_.Start(result->optimization.trajectory, &error)) {
        ROS_ERROR("Live trajectory visualization disabled: %s", error.c_str());
      }
      ROS_INFO("Live planning succeeded; results: %s",
               result->result_directory.c_str());
    } else {
      playback_.StopAndClear();
      ROS_ERROR("Live planning failed: %s; max dynamics residual: %.3g; "
                "max OBCA violation: %.3g; results: %s",
                result->status.c_str(),
                result->optimization.max_dynamics_residual,
                result->optimization.max_obca_violation,
                result->result_directory.c_str());
    }
  }

  ros::NodeHandle private_node_;
  ScenarioManager scenario_;
  planning_platform::PlanningConfiguration configuration_;
  std::string vehicle_id_ = "vehicle_1";
  std::atomic<bool> planning_{false};
  std::atomic<bool> cancel_planning_{false};
  bool show_hybrid_search_ = true;
  bool scene_locked_ = false;
  bool last_locked_ = false;
  std::atomic<std::uint64_t> scene_generation_{0};
  visualization::SceneVisualizer visualizer_;
  visualization::TrajectoryPlaybackVisualizer playback_;
  ros::ServiceClient command_client_;
  ros::Subscriber lock_subscriber_;
  ros::Timer result_timer_;
  std::thread planning_thread_;
  std::mutex event_mutex_;
  std::shared_ptr<planning::HybridSearchProgress> pending_hybrid_progress_;
  std::shared_ptr<planning::HybridAStartResult> pending_hybrid_;
  std::shared_ptr<planning_platform::PlanningRunResult> completed_result_;
  std::deque<StageEvent> pending_stage_updates_;
  planning_platform::PlanningStageUpdate active_stage_;
  ros::WallTime active_stage_begin_;
  ros::WallTime last_stage_publish_;
  bool stage_active_ = false;
  std::uint64_t pending_hybrid_generation_ = 0;
  std::uint64_t pending_progress_generation_ = 0;
  std::uint64_t completed_generation_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_planning");
  PlanningNode node;
  if (!node.Initialize()) return EXIT_FAILURE;
  ros::spin();
  return EXIT_SUCCESS;
}
