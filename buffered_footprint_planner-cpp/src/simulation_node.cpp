#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>

#include "buffered_footprint_planner/SetObjectPose.h"
#include "common/math/math_utils.h"
#include "scene/scenario_manager.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "visualize/scene_visualizer.h"

namespace {

geometry_msgs::Quaternion QuaternionFromYaw(double yaw) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

class SimulationNode {
 public:
  SimulationNode() : private_node_("~") {}

  bool Initialize() {
    if (!scenario_.Load(private_node_) || !LoadVehicle()) return false;
    private_node_.param("vehicle/id", vehicle_id_, std::string("vehicle_1"));
    if (!private_node_.getParam("vehicle/x", vehicle_pose_.x) ||
        !private_node_.getParam("vehicle/y", vehicle_pose_.y) ||
        !private_node_.getParam("vehicle/yaw", vehicle_pose_.yaw) ||
        !ValidateVehiclePose(vehicle_pose_)) {
      ROS_ERROR("Invalid initial vehicle pose");
      return false;
    }
    private_node_.param("start_lock/stationary_duration", stationary_duration_, 3.0);
    private_node_.param("start_lock/position_threshold", position_threshold_, 0.002);
    private_node_.param("start_lock/yaw_threshold", yaw_threshold_, 0.0087266463);

    visualizer_.Initialize(private_node_, scenario_.frame_id(), vehicle_,
                           marker_topic_);
    interactive_server_ =
        std::make_unique<interactive_markers::InteractiveMarkerServer>(
            "/simulation/scene_editor", "", false);
    BuildInteractiveMarkers();
    obstacle_poses_publisher_ = private_node_.advertise<geometry_msgs::PoseArray>(
        "/simulation/scene_obstacle_poses", 1, true);
    vehicle_pose_publisher_ = private_node_.advertise<geometry_msgs::PoseStamped>(
        "/simulation/vehicle_pose", 1, true);
    locked_start_publisher_ = private_node_.advertise<geometry_msgs::PoseStamped>(
        "/simulation/locked_start", 1, true);
    start_locked_publisher_ = private_node_.advertise<std_msgs::Bool>(
        "/simulation/start_locked", 1, true);
    revision_publisher_ = private_node_.advertise<std_msgs::UInt64>(
        "/simulation/scene_revision", 1, true);
    command_service_ = private_node_.advertiseService(
        "/simulation/set_object_pose", &SimulationNode::HandleCommand, this);
    stationary_timer_ = private_node_.createTimer(
        ros::Duration(0.1), &SimulationNode::CheckStationary, this);

    QueueStaticScene();
    visualizer_.QueuePlanningStage(scenario_.bounds(), "Scene editing");
    visualizer_.Trigger();
    PublishSceneState();
    ROS_INFO("Scene editor ready; run 'rosrun buffered_footprint_planner "
             "keyboard_editor.py' in a separate terminal");
    return true;
  }

 private:
  using VehiclePose = scene::Pose2d;

  bool LoadVehicle() {
    if (!private_node_.getParam("wheel_base", vehicle_.wheel_base) ||
        !private_node_.getParam("front_hang", vehicle_.front_hang) ||
        !private_node_.getParam("rear_hang", vehicle_.rear_hang) ||
        !private_node_.getParam("width", vehicle_.width)) {
      ROS_ERROR("Missing vehicle geometry in planner.yaml");
      return false;
    }
    vehicle_.GenerateDisc();
    return vehicle_.wheel_base > 0.0 && vehicle_.front_hang >= 0.0 &&
           vehicle_.rear_hang >= 0.0 && vehicle_.width > 0.0;
  }

  common::math::Box2d VehicleBox(const VehiclePose& pose) const {
    return vehicle_.GenerateBox(common::math::Pose(pose.x, pose.y, pose.yaw));
  }

  bool ValidateVehiclePose(const VehiclePose& pose) const {
    const auto box = VehicleBox(pose);
    const auto& bounds = scenario_.bounds();
    for (const auto& corner : box.corners()) {
      if (corner.x() < bounds[0] || corner.x() > bounds[1] ||
          corner.y() < bounds[2] || corner.y() > bounds[3]) return false;
    }
    const common::math::Polygon2d footprint(box);
    for (const auto& obstacle : scenario_.obstacles()) {
      if (footprint.HasOverlap(obstacle.polygon())) return false;
    }
    return true;
  }

  void QueueStaticScene() const {
    visualizer_.QueueMap(scenario_.Snapshot(vehicle_pose_));
    visualizer_.QueueGoal(scenario_.goal());
  }

  void ClearTaskMarkers() const {
    visualizer_.ClearPlanningResults();
  }

  void BeginEditing() {
    if (editing_active_) return;
    editing_active_ = true;
    start_locked_ = false;
    ClearTaskMarkers();
    visualizer_.QueuePlanningStage(scenario_.bounds(), "Scene editing");
    visualizer_.Trigger();
    PublishSceneState();
    ROS_INFO("Scene editing resumed; previous planning task invalidated");
  }

  bool HandleCommand(
      buffered_footprint_planner::SetObjectPose::Request& request,
      buffered_footprint_planner::SetObjectPose::Response& response) {
    if (request.action == request.BEGIN_PLANNING) {
      if (planning_active_ || editing_active_ || !start_locked_) {
        return FillResponse(false, "Start is not locked or planner is busy",
                            request.object_id, response);
      }
      planning_active_ = true;
      SetInteractionEnabled(false);
      return FillResponse(true, "Planning scene locked", request.object_id,
                          response);
    }
    if (request.action == request.END_PLANNING) {
      planning_active_ = false;
      SetInteractionEnabled(true);
      return FillResponse(true, "Planning scene released", request.object_id,
                          response);
    }
    if (request.action == request.FINISH_EDITING) {
      if (planning_active_) {
        return FillResponse(false, "Planning is active", request.object_id,
                            response);
      }
      editing_active_ = false;
      start_locked_ = false;
      stable_pose_ = vehicle_pose_;
      stable_since_ = ros::Time::now();
      last_countdown_seconds_ = -1;
      ClearTaskMarkers();
      QueueCountdown();
      PublishSceneState();
      ROS_INFO("Editing finished; checking Vehicle 1 stability for %.1f s",
               stationary_duration_);
      return FillResponse(true, "Stationary detection started", vehicle_id_,
                          response);
    }
    if (request.action == request.QUERY_POSE) {
      return FillResponse(true, "Current pose", request.object_id, response);
    }
    if (request.action != request.SET_POSE) {
      return FillResponse(false, "Unknown command", request.object_id,
                          response);
    }
    if (planning_active_) {
      return FillResponse(false, "Editing is disabled during planning",
                          request.object_id, response);
    }

    std::string message;
    if (!ApplyPose(request.object_id, request.x, request.y, request.yaw,
                   &message)) {
      return FillResponse(false, message, request.object_id, response);
    }
    return FillResponse(true, "Pose updated", request.object_id, response);
  }

  bool ApplyPose(const std::string& object_id, double x, double y, double yaw,
                 std::string* message) {
    if (planning_active_) {
      if (message) *message = "Editing is disabled during planning";
      return false;
    }
    yaw = common::math::NormalizeAngle(yaw);
    if (object_id == vehicle_id_) {
      const VehiclePose candidate{x, y, yaw};
      if (!ValidateVehiclePose(candidate)) {
        if (message) *message = "Vehicle leaves map or collides";
        return false;
      }
      vehicle_pose_ = candidate;
    } else {
      const auto found = std::find_if(
          scenario_.obstacles().begin(), scenario_.obstacles().end(),
          [&](const scene::SceneObstacle& obstacle) {
            return obstacle.id == object_id;
          });
      if (found == scenario_.obstacles().end()) {
        if (message) *message = "Unknown object";
        return false;
      }
      if (!scenario_.SetObstaclePose(object_id, x, y, yaw)) {
        if (message) *message = "Obstacle would leave map bounds";
        return false;
      }
    }
    // Only a valid edit invalidates a completed plan or restarts the lock
    // sequence. Rejected mouse/key poses leave the current result untouched.
    BeginEditing();
    ++scene_revision_;
    SyncInteractiveMarker(object_id);
    PublishSceneState();
    if (message) *message = "Pose updated";
    return true;
  }

  bool FillResponse(
      bool success, const std::string& message, const std::string& object_id,
      buffered_footprint_planner::SetObjectPose::Response& response) const {
    response.success = success;
    response.message = message;
    response.start_locked = start_locked_;
    response.scene_revision = scene_revision_;
    if (object_id == vehicle_id_ || object_id.empty()) {
      response.x = vehicle_pose_.x;
      response.y = vehicle_pose_.y;
      response.yaw = vehicle_pose_.yaw;
      return true;
    }
    for (const auto& obstacle : scenario_.obstacles()) {
      if (obstacle.id == object_id) {
        response.x = obstacle.x;
        response.y = obstacle.y;
        response.yaw = obstacle.yaw;
        return true;
      }
    }
    response.success = false;
    response.message = "Unknown object";
    return true;
  }

  geometry_msgs::Pose InteractivePose(const std::string& object_id) const {
    geometry_msgs::Pose pose;
    if (object_id == vehicle_id_) {
      pose.position.x = vehicle_pose_.x;
      pose.position.y = vehicle_pose_.y;
      pose.orientation = QuaternionFromYaw(vehicle_pose_.yaw);
      return pose;
    }
    for (const auto& obstacle : scenario_.obstacles()) {
      if (obstacle.id != object_id) continue;
      pose.position.x = obstacle.x;
      pose.position.y = obstacle.y;
      pose.orientation = QuaternionFromYaw(obstacle.yaw);
      return pose;
    }
    pose.orientation.w = 1.0;
    return pose;
  }

  visualization_msgs::Marker BoxMarker(double length, double width,
                                        double center_x, float red,
                                        float green, float blue,
                                        float alpha) const {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = center_x;
    marker.pose.position.z = 0.015;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = length;
    marker.scale.y = width;
    marker.scale.z = 0.03;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    return marker;
  }

  visualization_msgs::Marker OutlineMarker(double rear_x, double front_x,
                                            double width, float red,
                                            float green, float blue) const {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.014;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;
    const std::pair<double, double> corners[] = {
        {rear_x, 0.5 * width}, {front_x, 0.5 * width},
        {front_x, -0.5 * width}, {rear_x, -0.5 * width},
        {rear_x, 0.5 * width}};
    for (const auto& corner : corners) {
      geometry_msgs::Point point;
      point.x = corner.first;
      point.y = corner.second;
      point.z = 0.035;
      marker.points.push_back(point);
    }
    return marker;
  }

  visualization_msgs::InteractiveMarker MakeInteractiveMarker(
      const std::string& object_id, double length, double width,
      bool is_vehicle, bool interactive,
      std::size_t obstacle_number = 0) const {
    visualization_msgs::InteractiveMarker marker;
    marker.header.frame_id = scenario_.frame_id();
    marker.name = object_id;
    marker.description = is_vehicle ? object_id : std::string();
    marker.pose = InteractivePose(object_id);
    marker.scale = std::max(0.28, 1.5 * std::max(length, width));

    visualization_msgs::InteractiveMarkerControl visual;
    visual.name = "object";
    visual.always_visible = true;
    if (is_vehicle) {
      const double rear = -vehicle_.rear_hang;
      const double front = vehicle_.wheel_base + vehicle_.front_hang;
      visual.markers.push_back(BoxMarker(front - rear, width,
                                         0.5 * (front + rear),
                                         0.25f, 0.78f, 0.25f, 0.22f));
      visual.markers.push_back(
          OutlineMarker(rear, front, width, 0.0f, 0.65f, 0.0f));
      visualization_msgs::Marker heading;
      heading.type = visualization_msgs::Marker::ARROW;
      heading.pose.orientation.w = 1.0;
      heading.scale.x = std::max(0.12, front);
      heading.scale.y = 0.025;
      heading.scale.z = 0.04;
      heading.color.g = 0.65f;
      heading.color.a = 1.0f;
      visual.markers.push_back(heading);
    } else {
      visual.markers.push_back(
          BoxMarker(length, width, 0.0, 0.55f, 0.55f, 0.55f, 1.0f));
      visual.markers.push_back(OutlineMarker(-0.5 * length, 0.5 * length,
                                             width, 0.0f, 0.0f, 0.0f));
      visualization_msgs::Marker number;
      number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      number.pose.position.z = 0.06;
      number.pose.orientation.w = 1.0;
      number.scale.z = 0.08;
      number.color.r = 0.0f;
      number.color.g = 0.0f;
      number.color.b = 0.0f;
      number.color.a = 1.0f;
      number.text = std::to_string(obstacle_number);
      visual.markers.push_back(number);
    }
    marker.controls.push_back(visual);

    if (interactive) {
      visualization_msgs::InteractiveMarkerControl move;
      move.name = "move_xy";
      move.orientation.w = 0.7071067812;
      move.orientation.y = 0.7071067812;
      move.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
      marker.controls.push_back(move);
      visualization_msgs::InteractiveMarkerControl rotate = move;
      rotate.name = "rotate_z";
      rotate.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      marker.controls.push_back(rotate);
    }
    return marker;
  }

  void BuildInteractiveMarkers(bool interactive = true) {
    if (!interactive_server_) return;
    interaction_enabled_ = interactive;
    interactive_server_->clear();
    for (std::size_t i = 0; i < scenario_.obstacles().size(); ++i) {
      const auto& obstacle = scenario_.obstacles()[i];
      interactive_server_->insert(
          MakeInteractiveMarker(obstacle.id, obstacle.length, obstacle.width,
                                false, interactive, i + 1),
          [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr&
                     feedback) { HandleInteractiveFeedback(feedback); });
    }
    const double vehicle_length =
        vehicle_.rear_hang + vehicle_.wheel_base + vehicle_.front_hang;
    interactive_server_->insert(
        MakeInteractiveMarker(vehicle_id_, vehicle_length, vehicle_.width,
                              true, interactive),
        [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr&
                   feedback) { HandleInteractiveFeedback(feedback); });
    interactive_server_->applyChanges();
  }

  void SetInteractionEnabled(bool enabled) {
    if (enabled == interaction_enabled_) return;
    BuildInteractiveMarkers(enabled);
  }

  void SyncInteractiveMarker(const std::string& object_id) {
    if (!interactive_server_) return;
    interactive_server_->setPose(object_id, InteractivePose(object_id));
    interactive_server_->applyChanges();
  }

  void HandleInteractiveFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    if (!feedback ||
        feedback->event_type !=
            visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
      return;
    }
    if (!interaction_enabled_ || planning_active_) {
      SyncInteractiveMarker(feedback->marker_name);
      return;
    }
    std::string message;
    const double yaw = tf2::getYaw(feedback->pose.orientation);
    if (!ApplyPose(feedback->marker_name, feedback->pose.position.x,
                   feedback->pose.position.y, yaw, &message)) {
      SyncInteractiveMarker(feedback->marker_name);
      ROS_WARN("Interactive edit rejected for '%s': %s",
               feedback->marker_name.c_str(), message.c_str());
    }
  }

  void QueueCountdown() {
    const double elapsed = (ros::Time::now() - stable_since_).toSec();
    const int remaining = std::max(
        1, static_cast<int>(std::ceil(stationary_duration_ - elapsed)));
    if (remaining == last_countdown_seconds_) return;
    last_countdown_seconds_ = remaining;
    visualizer_.QueuePlanningStage(
        scenario_.bounds(),
        "Start locking: " + std::to_string(remaining) + " s");
    visualizer_.Trigger();
  }

  void CheckStationary(const ros::TimerEvent&) {
    if (editing_active_ || planning_active_ || start_locked_) return;
    const double position_change =
        std::hypot(vehicle_pose_.x - stable_pose_.x,
                   vehicle_pose_.y - stable_pose_.y);
    const double yaw_change = std::abs(common::math::NormalizeAngle(
        vehicle_pose_.yaw - stable_pose_.yaw));
    if (position_change > position_threshold_ || yaw_change > yaw_threshold_) {
      stable_pose_ = vehicle_pose_;
      stable_since_ = ros::Time::now();
      last_countdown_seconds_ = -1;
      return;
    }
    if ((ros::Time::now() - stable_since_).toSec() < stationary_duration_) {
      QueueCountdown();
      return;
    }

    locked_start_ = vehicle_pose_;
    start_locked_ = true;
    visualizer_.QueuePlanningStage(scenario_.bounds(), "Start locked");
    visualizer_.Trigger();
    PublishSceneState();
    ROS_INFO("Vehicle 1 start locked at (%.3f, %.3f, %.1f deg)",
             locked_start_.x, locked_start_.y,
             locked_start_.yaw * 180.0 / M_PI);
  }

  geometry_msgs::PoseStamped ToPoseStamped(const VehiclePose& pose) const {
    geometry_msgs::PoseStamped message;
    message.header.frame_id = scenario_.frame_id();
    message.header.stamp = ros::Time::now();
    message.pose.position.x = pose.x;
    message.pose.position.y = pose.y;
    message.pose.orientation = QuaternionFromYaw(pose.yaw);
    return message;
  }

  void PublishSceneState() const {
    geometry_msgs::PoseArray obstacles;
    obstacles.header.frame_id = scenario_.frame_id();
    obstacles.header.stamp = ros::Time::now();
    for (const auto& obstacle : scenario_.obstacles()) {
      geometry_msgs::Pose pose;
      pose.position.x = obstacle.x;
      pose.position.y = obstacle.y;
      pose.orientation = QuaternionFromYaw(obstacle.yaw);
      obstacles.poses.push_back(pose);
    }
    obstacle_poses_publisher_.publish(obstacles);
    vehicle_pose_publisher_.publish(ToPoseStamped(vehicle_pose_));
    if (start_locked_) locked_start_publisher_.publish(ToPoseStamped(locked_start_));
    std_msgs::Bool locked;
    locked.data = start_locked_;
    start_locked_publisher_.publish(locked);
    std_msgs::UInt64 revision;
    revision.data = scene_revision_;
    revision_publisher_.publish(revision);
  }

  ros::NodeHandle private_node_;
  ScenarioManager scenario_;
  VehicleParameter vehicle_;
  std::string vehicle_id_;
  VehiclePose vehicle_pose_;
  VehiclePose stable_pose_;
  VehiclePose locked_start_;
  bool editing_active_ = true;
  bool planning_active_ = false;
  bool start_locked_ = false;
  double stationary_duration_ = 3.0;
  double position_threshold_ = 0.002;
  double yaw_threshold_ = 0.0087266463;
  int last_countdown_seconds_ = -1;
  bool interaction_enabled_ = true;
  ros::Time stable_since_;
  std::uint64_t scene_revision_ = 0;
  const std::string marker_topic_ = "/buffered_footprint_planner/markers";
  visualization::SceneVisualizer visualizer_;
  ros::Publisher obstacle_poses_publisher_;
  ros::Publisher vehicle_pose_publisher_;
  ros::Publisher locked_start_publisher_;
  ros::Publisher start_locked_publisher_;
  ros::Publisher revision_publisher_;
  ros::ServiceServer command_service_;
  ros::Timer stationary_timer_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer>
      interactive_server_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_simulation");
  SimulationNode node;
  if (!node.Initialize()) return EXIT_FAILURE;
  ros::spin();
  return EXIT_SUCCESS;
}
