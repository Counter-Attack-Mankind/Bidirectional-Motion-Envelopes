#include <atomic>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <sandbox_msgs/AprilObject.h>
#include <sandbox_msgs/ChassisCommand.h>
#include <sandbox_msgs/Trajectory.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include "execution/experiment_manager.h"
#include "execution/real_vehicle_logger.h"
#include "execution/task_generation.h"
#include "execution/trajectory_adapter.h"
#include "planning/planning_config_loader.h"
#include "planning/planning_pipeline.h"
#include "scene/mocap_scene_provider.h"
#include "scene/scenario_manager.h"
#include "visualize/real_vehicle_visualizer.h"
#include "visualize/scene_visualizer.h"
#include "visualize/trajectory_playback_visualizer.h"

namespace {

class RealVehicleNode {
 public:
  RealVehicleNode() : private_node_("~"), manager_(LoadManagerConfig()) {}
  ~RealVehicleNode() {
    shutting_down_ = true;
    if (planning_thread_.joinable()) planning_thread_.join();
    PublishEstop(true);
  }

  bool Initialize() {
    if (!scenario_.Load(private_node_)) return false;
    std::string error;
    if (!planning_platform::LoadPlanningConfiguration(
            private_node_, scenario_.bounds(), &configuration_, &error)) {
      ROS_ERROR("Cannot load planning configuration: %s", error.c_str());
      return false;
    }
    private_node_.param("enable_execution", execution_enabled_, false);
    private_node_.param("visualization/show_hybrid_search",
                        show_hybrid_search_, true);
    const scene::Pose2d unused_start{};
    if (!mocap_.Initialize(private_node_, scenario_.Snapshot(unused_start)))
      return false;
    visualizer_.Initialize(private_node_, scenario_.frame_id(),
                           configuration_.vehicle);
    double playback_speed = 1.0;
    double playback_frequency = 100.0;
    double swept_resolution = 0.005;
    private_node_.param("visualization/playback_speed", playback_speed,
                        playback_speed);
    private_node_.param("visualization/playback_frequency", playback_frequency,
                        playback_frequency);
    private_node_.param("visualization/swept_area_resolution", swept_resolution,
                        swept_resolution);
    if (!playback_.Initialize(private_node_, &visualizer_, configuration_.vehicle,
                              playback_speed, playback_frequency,
                              swept_resolution, &error)) {
      ROS_ERROR("Cannot initialize trajectory playback: %s", error.c_str());
      return false;
    }
    if (!real_visualizer_.Initialize(configuration_.vehicle, swept_resolution,
                                     &error)) {
      ROS_ERROR("Cannot initialize measured-track visualization: %s",
                error.c_str());
      return false;
    }
    trajectory_publisher_ = private_node_.advertise<sandbox_msgs::Trajectory>(
        "/traj", 1, true);
    object_publisher_ = private_node_.advertise<sandbox_msgs::AprilObject>(
        "/object", 5, false);
    estop_publisher_ = private_node_.advertise<std_msgs::Bool>(
        "/estop", 1, true);
    reached_subscriber_ = private_node_.subscribe(
        "/reached", 1, &RealVehicleNode::ReachedCallback, this);
    enable_subscriber_ = private_node_.subscribe(
        "/execution_enable", 1, &RealVehicleNode::EnableCallback, this);
    command_subscriber_ = private_node_.subscribe(
        "/chassis", 20, &RealVehicleNode::CommandCallback, this);
    timer_ = private_node_.createTimer(
        ros::Duration(0.05), &RealVehicleNode::Tick, this);
    PublishEstop(true);
    ROS_WARN("Real vehicle node started with execution %s. Hardware motion "
             "requires enable_execution:=true and a valid fresh plan.",
             execution_enabled_ ? "ENABLED" : "disabled");
    return true;
  }

 private:
  execution::ExperimentManager::Configuration LoadManagerConfig() {
    execution::ExperimentManager::Configuration config;
    private_node_.param("experiment/stationary_duration",
                        config.stationary_duration,
                        config.stationary_duration);
    private_node_.param("experiment/vehicle_stationary_position_threshold",
                        config.vehicle_stationary_position_threshold,
                        config.vehicle_stationary_position_threshold);
    private_node_.param("experiment/vehicle_stationary_yaw_threshold",
                        config.vehicle_stationary_yaw_threshold,
                        config.vehicle_stationary_yaw_threshold);
    private_node_.param("experiment/obstacle_stationary_boundary_threshold",
                        config.obstacle_stationary_boundary_threshold,
                        config.obstacle_stationary_boundary_threshold);
    private_node_.param("experiment/frozen_vehicle_position_threshold",
                        config.frozen_vehicle_position_threshold,
                        config.frozen_vehicle_position_threshold);
    private_node_.param("experiment/frozen_vehicle_yaw_threshold",
                        config.frozen_vehicle_yaw_threshold,
                        config.frozen_vehicle_yaw_threshold);
    private_node_.param("experiment/frozen_obstacle_boundary_threshold",
                        config.frozen_obstacle_boundary_threshold,
                        config.frozen_obstacle_boundary_threshold);
    private_node_.param("experiment/rearm_distance", config.rearm_distance,
                        config.rearm_distance);
    return config;
  }

  void PublishEstop(bool value) {
    if (!estop_publisher_) return;
    if (estop_published_ && estop_active_ == value) return;
    std_msgs::Bool message;
    message.data = value;
    estop_publisher_.publish(message);
    estop_active_ = value;
    estop_published_ = true;
  }

  void EnableCallback(const std_msgs::Bool::ConstPtr& message) {
    execution_enabled_ = message && message->data;
    if (!execution_enabled_) {
      PublishEstop(true);
      manager_.AbortTask();
      logger_.Stop();
      ROS_WARN("Real vehicle execution disabled at runtime");
    } else {
      ROS_WARN("Real vehicle execution authorized; the current state and next "
               "fresh planning result still gate motion");
    }
  }

  void ReachedCallback(const std_msgs::Int32::ConstPtr& message) {
    if (!message || message->data != active_vehicle_id_ ||
        manager_.state() != execution::ExperimentManager::State::kTracking)
      return;
    PublishEstop(true);
    manager_.Reached(latest_scene_.start);
    logger_.Stop();
    ROS_INFO("Vehicle %d reported goal reached; task %llu completed",
             active_vehicle_id_,
             static_cast<unsigned long long>(manager_.task_id()));
  }

  void PublishVehicle(const scene::MocapSceneProvider::Status& status,
                      const scene::SceneSnapshot& snapshot) {
    if (!status.vehicle_fresh || status.vehicle_id < 0) return;
    sandbox_msgs::AprilObject object;
    object.header.stamp = ros::Time::now();
    object.header.frame_id = snapshot.frame_id;
    object.type = sandbox_msgs::AprilObject::VEHICLE;
    object.id = status.vehicle_id;
    object.x = snapshot.start.x;
    object.y = snapshot.start.y;
    object.yaw = snapshot.start.yaw;
    object_publisher_.publish(object);
  }

  void CommandCallback(
      const sandbox_msgs::ChassisCommand::ConstPtr& message) {
    if (!message || message->target != planned_vehicle_id_ ||
        manager_.state() != execution::ExperimentManager::State::kTracking)
      return;
    logger_.LogCommand(ros::Time::now().toSec(), *message);
  }

  void StartPlanning() {
    scene::SceneSnapshot frozen;
    std::uint64_t generation = 0;
    if (!manager_.BeginPlanning(&frozen, &generation)) return;
    if (planning_thread_.joinable()) planning_thread_.join();
    planning_active_ = true;
    active_generation_ = generation;
    planned_vehicle_id_ = active_vehicle_id_;
    playback_.StopAndClear();
    real_visualizer_.Clear();
    logger_.Stop();
    visualizer_.ClearPlanningResults();
    PublishEstop(true);
    const int vehicle_id = planned_vehicle_id_;
    planning_thread_ = std::thread([this, frozen, generation, vehicle_id]() {
      planning_platform::PlanningPipeline pipeline(configuration_);
      auto result = std::make_shared<planning_platform::PlanningRunResult>(
          pipeline.Run(
              frozen, "real_vehicle",
              [this, generation](const planning::HybridAStartResult& path, double) {
                std::lock_guard<std::mutex> lock(result_mutex_);
                pending_hybrid_ =
                    std::make_shared<planning::HybridAStartResult>(path);
                pending_hybrid_generation_ = generation;
              },
              show_hybrid_search_
                  ? planning_platform::PlanningPipeline::HybridProgressCallback(
                        [this, generation](planning::HybridSearchProgress progress) {
                          std::lock_guard<std::mutex> lock(result_mutex_);
                          pending_progress_ =
                              std::make_shared<planning::HybridSearchProgress>(
                                  std::move(progress));
                          pending_progress_generation_ = generation;
                        })
                  : planning_platform::PlanningPipeline::HybridProgressCallback{},
              [this, generation]() {
                return shutting_down_.load() || !ros::ok() ||
                       generation != active_generation_.load();
              },
              [this, generation](planning_platform::PlanningStageUpdate update) {
                std::lock_guard<std::mutex> lock(result_mutex_);
                pending_stage_ = std::make_shared<
                    planning_platform::PlanningStageUpdate>(std::move(update));
                pending_stage_generation_ = generation;
              }));
      std::lock_guard<std::mutex> lock(result_mutex_);
      completed_result_ = std::move(result);
      completed_generation_ = generation;
      completed_vehicle_id_ = vehicle_id;
      planning_active_ = false;
    });
    ROS_INFO("Started real-scene planning task %d (generation %llu)",
             frozen.task_id, static_cast<unsigned long long>(generation));
  }

  void ProcessPlanningEvents() {
    std::shared_ptr<planning::HybridSearchProgress> progress;
    std::shared_ptr<planning::HybridAStartResult> hybrid;
    std::shared_ptr<planning_platform::PlanningRunResult> result;
    std::shared_ptr<planning_platform::PlanningStageUpdate> stage;
    std::uint64_t progress_generation = 0;
    std::uint64_t hybrid_generation = 0;
    std::uint64_t result_generation = 0;
    std::uint64_t stage_generation = 0;
    int result_vehicle = -1;
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      progress = std::move(pending_progress_);
      hybrid = std::move(pending_hybrid_);
      result = std::move(completed_result_);
      stage = std::move(pending_stage_);
      progress_generation = pending_progress_generation_;
      hybrid_generation = pending_hybrid_generation_;
      result_generation = completed_generation_;
      result_vehicle = completed_vehicle_id_;
      stage_generation = pending_stage_generation_;
    }
    const std::uint64_t active = active_generation_.load();
    if (progress && progress_generation == active)
      visualizer_.QueueHybridSearch(*progress);
    if (hybrid && hybrid_generation == active)
      visualizer_.QueueHybridPath(*hybrid);
    if (stage && stage_generation == active) {
      visualizer_.QueuePlanningStage(
          latest_scene_.bounds, stage->message + " (" +
          std::to_string(stage->total_elapsed) + " s)");
    }
    if ((progress && progress_generation == active) ||
        (hybrid && hybrid_generation == active) ||
        (stage && stage_generation == active)) visualizer_.Trigger();
    if (!result) return;

    if (!execution::IsCurrentTask(
            result_generation, result_vehicle, active, planned_vehicle_id_,
            active_vehicle_id_,
            manager_.state() ==
                execution::ExperimentManager::State::kPlanning)) {
      ROS_WARN("Discarded stale planning result generation=%llu vehicle=%d",
               static_cast<unsigned long long>(result_generation),
               result_vehicle);
      return;
    }

    const bool usable = result->success && manager_.frozen_scene_valid();
    if (usable) {
      visualizer_.QueueOptimizedTrajectory(result->optimization.trajectory);
      visualizer_.QueueOptimizedConfigurationPoints(
          result->optimization.trajectory);
      visualizer_.Trigger();
    }
    const bool execute = usable && execution_enabled_;
    manager_.PlanningFinished(usable, execute);
    if (!execute) {
      PublishEstop(true);
      if (usable) {
        std::string playback_error;
        if (!playback_.Start(result->optimization.trajectory, &playback_error))
          ROS_ERROR("Cannot start pure-planning playback: %s",
                    playback_error.c_str());
      }
      ROS_INFO("Planning task finished (%s); result=%s, execution=%s, dir=%s",
               usable ? "usable" : "rejected", result->status.c_str(),
               execution_enabled_ ? "authorized" : "disabled",
               result->result_directory.c_str());
      return;
    }
    sandbox_msgs::Trajectory trajectory;
    std::string error;
    if (!execution::TrajectoryAdapter::Convert(
            result->optimization.trajectory, active_vehicle_id_,
            static_cast<std::uint32_t>(manager_.task_id()), &trajectory,
            &error)) {
      PublishEstop(true);
      manager_.ForceSafeWait();
      ROS_ERROR("Refusing invalid optimized trajectory: %s", error.c_str());
      return;
    }
    // The real-vehicle PP selects the auto-discovered target from this message.
    // Publish this task exactly once; repeated publication resets PP timing.
    trajectory_publisher_.publish(trajectory);
    real_visualizer_.StartTask(result->optimization.trajectory);
    if (!logger_.Start(result->result_directory, manager_.task_id(),
                       active_vehicle_id_, &error))
      ROS_ERROR("Real-vehicle logging disabled for this task: %s",
                error.c_str());
    PublishEstop(false);
    ROS_WARN("Published task %u once to /traj for vehicle %d; vehicle motion is enabled",
             trajectory.header.seq, active_vehicle_id_);
  }

  void Tick(const ros::TimerEvent&) {
    scene::MocapSceneProvider::Status status;
    scene::SceneSnapshot snapshot;
    const bool ready = mocap_.Snapshot(&snapshot, &status);
    latest_scene_ = snapshot;
    active_vehicle_id_ = status.vehicle_id;
    PublishVehicle(status, snapshot);
    manager_.Observe(snapshot, ready, status.revision, ros::WallTime::now().toSec());

    visualizer_.QueueMap(snapshot);
    for (std::size_t i = 0; i < snapshot.obstacles.size(); ++i)
      visualizer_.QueueObstacle(snapshot.obstacles[i], i);
    if (status.vehicle_id >= 0)
      visualizer_.QueueVehicle(
          snapshot.start, "Vehicle" + std::to_string(status.vehicle_id));
    visualizer_.QueueGoal(snapshot.goal);
    visualizer_.QueuePlanningStage(
        snapshot.bounds, ready ? manager_.StateName() : status.message);
    visualizer_.Trigger();

    if (!ready || !manager_.frozen_scene_valid()) {
      PublishEstop(true);
      if (planning_active_ &&
          manager_.state() != execution::ExperimentManager::State::kPlanning)
        active_generation_.fetch_add(1);
    }
    if (ready && manager_.state() ==
                     execution::ExperimentManager::State::kTracking) {
      real_visualizer_.Update(snapshot.start);
      logger_.LogPose(ros::Time::now().toSec(), snapshot.start);
    }
    if (manager_.state() ==
            execution::ExperimentManager::State::kReadyToPlan &&
        !planning_active_) {
      StartPlanning();
    }
    ProcessPlanningEvents();
  }

  ros::NodeHandle private_node_;
  ScenarioManager scenario_;
  planning_platform::PlanningConfiguration configuration_;
  scene::MocapSceneProvider mocap_;
  execution::ExperimentManager manager_;
  execution::RealVehicleLogger logger_;
  visualization::SceneVisualizer visualizer_;
  visualization::TrajectoryPlaybackVisualizer playback_;
  visualization::RealVehicleVisualizer real_visualizer_;
  scene::SceneSnapshot latest_scene_;
  ros::Publisher trajectory_publisher_;
  ros::Publisher object_publisher_;
  ros::Publisher estop_publisher_;
  ros::Subscriber reached_subscriber_;
  ros::Subscriber enable_subscriber_;
  ros::Subscriber command_subscriber_;
  ros::Timer timer_;
  std::thread planning_thread_;
  std::mutex result_mutex_;
  std::shared_ptr<planning::HybridSearchProgress> pending_progress_;
  std::shared_ptr<planning::HybridAStartResult> pending_hybrid_;
  std::shared_ptr<planning_platform::PlanningRunResult> completed_result_;
  std::shared_ptr<planning_platform::PlanningStageUpdate> pending_stage_;
  std::uint64_t pending_progress_generation_ = 0;
  std::uint64_t pending_hybrid_generation_ = 0;
  std::uint64_t pending_stage_generation_ = 0;
  std::uint64_t completed_generation_ = 0;
  int completed_vehicle_id_ = -1;
  int planned_vehicle_id_ = -1;
  std::atomic<std::uint64_t> active_generation_{0};
  std::atomic<bool> planning_active_{false};
  std::atomic<bool> shutting_down_{false};
  bool execution_enabled_ = false;
  bool show_hybrid_search_ = true;
  bool estop_active_ = true;
  bool estop_published_ = false;
  int active_vehicle_id_ = -1;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_real_vehicle");
  RealVehicleNode node;
  if (!node.Initialize()) return EXIT_FAILURE;
  ros::spin();
  return EXIT_SUCCESS;
}
