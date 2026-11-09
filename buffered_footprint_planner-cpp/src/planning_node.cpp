#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "buffered_footprint_planner/SetObjectPose.h"
#include "environment/scenario_manager.h"
#include "planning/planning_config_loader.h"
#include "planning/planning_pipeline.h"
#include "visualize/visualization_plot.h"

namespace {

class PlanningNode {
 public:
  PlanningNode() : private_node_("~") {}

  bool Initialize() {
    if (!scenario_.Load(private_node_)) return false;
    std::string error;
    if (!planning_platform::LoadPlanningConfiguration(
            private_node_, scenario_.bounds(), &configuration_, &error)) {
      ROS_ERROR("Cannot load planning configuration: %s", error.c_str());
      return false;
    }
    private_node_.param("vehicle/id", vehicle_id_, std::string("vehicle_1"));
    VisualizationPlot::Init(private_node_, scenario_.frame_id(),
                            "/buffered_footprint_planner/markers");
    command_client_ = private_node_.serviceClient<
        buffered_footprint_planner::SetObjectPose>(
        "/simulation/set_object_pose");
    if (!command_client_.waitForExistence(ros::Duration(10.0))) {
      ROS_ERROR("Simulation scene service is unavailable");
      return false;
    }
    lock_subscriber_ = private_node_.subscribe(
        "/simulation/start_locked", 1, &PlanningNode::LockCallback, this);
    ROS_INFO("Live planner ready; fixed goal=(%.3f, %.3f, %.3f rad)",
             scenario_.goal().x, scenario_.goal().y, scenario_.goal().yaw);
    return true;
  }

 private:
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
    const bool rising_edge = message->data && !last_locked_;
    last_locked_ = message->data;
    if (!rising_edge || planning_) return;
    planning_ = true;
    scene::SceneSnapshot snapshot;
    std::string error;
    if (!AcquireFrozenSnapshot(&snapshot, &error)) {
      ROS_ERROR("Cannot acquire live scene snapshot: %s", error.c_str());
      ReleaseScene();
      planning_ = false;
      return;
    }
    QueueGoal(snapshot.goal);
    planning_platform::PlanningPipeline pipeline(configuration_);
    const auto result = pipeline.Run(snapshot, "live");
    if (result.hybrid_succeeded) QueueHybridPath(result.hybrid_path);
    if (result.success) QueueOptimized(result.optimization.trajectory);
    VisualizationPlot::Trigger();
    if (result.success) {
      ROS_INFO("Live planning succeeded; results: %s",
               result.result_directory.c_str());
    } else {
      ROS_ERROR("Live planning failed: %s; results: %s",
                result.status.c_str(), result.result_directory.c_str());
    }
    ReleaseScene();
    planning_ = false;
  }

  void QueueGoal(const scene::Pose2d& goal) const {
    const auto box = configuration_.vehicle.GenerateBox(
        common::math::Pose(goal.x, goal.y, goal.yaw));
    VisualizationPlot::PlotPolygon(common::math::Polygon2d(box), 0.015,
                                   Color::Magenta, 0, "planning_goal_body");
    const double arrow_length = configuration_.vehicle.wheel_base +
                                configuration_.vehicle.front_hang + 0.04;
    VisualizationPlot::PlotArrow(
        goal.x, goal.y, goal.x + arrow_length * std::cos(goal.yaw),
        goal.y + arrow_length * std::sin(goal.yaw), Color::Magenta, 0,
        "planning_goal_heading");
    VisualizationPlot::PlotPoints({goal.x}, {goal.y}, Color::Magenta, 0.03,
                                  0, "planning_goal_point");
  }

  void QueueHybridPath(const planning::HybridAStartResult& path) const {
    std::vector<Color> colors(path.x.size(), Color::Green);
    for (std::size_t i = 0; i < colors.size() && i < path.gear.size(); ++i) {
      if (!path.gear[i]) colors[i] = Color::Cyan;
    }
    VisualizationPlot::Plot(path.x, path.y, 0.025, colors, 0,
                            "hybrid_astar_path");
  }

  void QueueOptimized(const optimizer::Trajectory& trajectory) const {
    if (trajectory.nodes.empty() || trajectory.intervals.empty()) return;
    VisualizationPlot::ClearMarkers("optimized_trajectory", 0, 1023);
    std::vector<double> x{trajectory.nodes.front().x};
    std::vector<double> y{trajectory.nodes.front().y};
    bool forward = trajectory.intervals.front().gear;
    int marker_id = 0;
    for (std::size_t i = 0; i < trajectory.intervals.size(); ++i) {
      x.push_back(trajectory.nodes[i + 1].x);
      y.push_back(trajectory.nodes[i + 1].y);
      const bool end = i + 1 == trajectory.intervals.size() ||
                       trajectory.intervals[i + 1].gear != forward;
      if (!end) continue;
      VisualizationPlot::Plot(x, y, 0.03,
                              forward ? Color::Blue : Color::Red,
                              marker_id++, "optimized_trajectory");
      if (i + 1 < trajectory.intervals.size()) {
        x = {trajectory.nodes[i + 1].x};
        y = {trajectory.nodes[i + 1].y};
        forward = trajectory.intervals[i + 1].gear;
      }
    }
  }

  ros::NodeHandle private_node_;
  ScenarioManager scenario_;
  planning_platform::PlanningConfiguration configuration_;
  std::string vehicle_id_ = "vehicle_1";
  bool planning_ = false;
  bool scene_locked_ = false;
  bool last_locked_ = false;
  ros::ServiceClient command_client_;
  ros::Subscriber lock_subscriber_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_planning");
  PlanningNode node;
  if (!node.Initialize()) return EXIT_FAILURE;
  ros::spin();
  return EXIT_SUCCESS;
}
