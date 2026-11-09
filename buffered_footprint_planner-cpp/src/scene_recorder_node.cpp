#include <cstdlib>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "buffered_footprint_planner/SetObjectPose.h"
#include "environment/scenario_manager.h"
#include "planning/planning_config_loader.h"
#include "scene/scene_recorder.h"

namespace {

class SceneRecorderNode {
 public:
  SceneRecorderNode() : private_node_("~") {}

  bool Initialize() {
    if (!scenario_.Load(private_node_)) return false;
    std::string error;
    if (!planning_platform::LoadVehicleParameter(private_node_, &vehicle_,
                                                  &error)) {
      ROS_ERROR("Cannot load vehicle parameters: %s", error.c_str());
      return false;
    }
    private_node_.param("vehicle/id", vehicle_id_, std::string("vehicle_1"));
    if (!private_node_.getParam("tasks_root", tasks_root_) ||
        tasks_root_.empty()) {
      ROS_ERROR("Parameter 'tasks_root' is required");
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
        "/simulation/start_locked", 1,
        &SceneRecorderNode::LockCallback, this);
    ROS_INFO("Scene recorder ready; press Enter after editing to save one task");
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

  bool AcquireSnapshot(scene::SceneSnapshot* snapshot, std::string* error) {
    buffered_footprint_planner::SetObjectPose::Response response;
    if (!Command(buffered_footprint_planner::SetObjectPoseRequest::BEGIN_PLANNING,
                 std::string(), &response)) {
      if (error) *error = response.message.empty() ?
          "Cannot freeze current scene" : response.message;
      return false;
    }
    scene_locked_ = true;
    if (!Command(buffered_footprint_planner::SetObjectPoseRequest::QUERY_POSE,
                 vehicle_id_, &response)) {
      if (error) *error = "Cannot query final vehicle pose";
      return false;
    }
    const scene::Pose2d start{response.x, response.y, response.yaw};
    for (const auto& obstacle : scenario_.obstacles()) {
      if (!Command(buffered_footprint_planner::SetObjectPoseRequest::QUERY_POSE,
                   obstacle.id, &response) ||
          !scenario_.SetObstaclePose(obstacle.id, response.x, response.y,
                                     response.yaw)) {
        if (error) *error = "Cannot query final obstacle '" + obstacle.id + "'";
        return false;
      }
    }
    *snapshot = scenario_.Snapshot(start);
    return true;
  }

  void ReleaseScene() {
    if (!scene_locked_) return;
    buffered_footprint_planner::SetObjectPose::Response response;
    Command(buffered_footprint_planner::SetObjectPoseRequest::END_PLANNING,
            std::string(), &response);
    scene_locked_ = false;
  }

  void LockCallback(const std_msgs::Bool::ConstPtr& message) {
    const bool rising_edge = message->data && !last_locked_;
    last_locked_ = message->data;
    if (!rising_edge || completed_) return;
    completed_ = true;
    scene::SceneSnapshot snapshot;
    std::string error;
    if (!AcquireSnapshot(&snapshot, &error)) {
      ROS_ERROR("Cannot acquire scene for recording: %s", error.c_str());
      ReleaseScene();
      ros::shutdown();
      return;
    }
    scene::SceneRecorder recorder(tasks_root_);
    std::string saved_file;
    if (!recorder.SaveNext(snapshot, vehicle_, &saved_file, &error)) {
      ROS_ERROR("Scene validation/save failed: %s", error.c_str());
    } else {
      ROS_INFO("Scene saved without running planning: %s", saved_file.c_str());
    }
    ReleaseScene();
    ros::shutdown();
  }

  ros::NodeHandle private_node_;
  ScenarioManager scenario_;
  VehicleParameter vehicle_;
  std::string vehicle_id_ = "vehicle_1";
  std::string tasks_root_;
  bool last_locked_ = false;
  bool completed_ = false;
  bool scene_locked_ = false;
  ros::ServiceClient command_client_;
  ros::Subscriber lock_subscriber_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_scene_recorder");
  SceneRecorderNode node;
  if (!node.Initialize()) return EXIT_FAILURE;
  ros::spin();
  return EXIT_SUCCESS;
}
