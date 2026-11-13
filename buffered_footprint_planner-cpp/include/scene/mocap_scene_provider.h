#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "scene/scene_snapshot.h"

namespace scene {

// Discovers the VRPN topics created by vrpn_client_ros and turns their latest
// measurements into a metric, map-frame planning scene.  Discovery is repeated
// throughout the process lifetime because VRPN tracker topics appear lazily.
class MocapSceneProvider {
 public:
  struct Status {
    bool ready = false;
    bool vehicle_fresh = false;
    bool obstacles_fresh = false;
    int vehicle_id = -1;
    std::size_t obstacle_count = 0;
    std::uint64_t revision = 0;
    std::string message;
  };

  bool Initialize(ros::NodeHandle& node, const SceneSnapshot& static_scene);
  bool Snapshot(SceneSnapshot* snapshot, Status* status = nullptr) const;
  Status GetStatus() const;

 private:
  struct Sample {
    geometry_msgs::PoseStamped pose;
    ros::Time received;
    bool seen = false;
  };
  struct ObstacleTrack {
    std::map<int, Sample> markers;
    SceneObstacle last_geometry;
    ros::Time last_geometry_time;
    bool has_geometry = false;
  };

  void DiscoverTopics(const ros::TimerEvent&);
  void VehicleCallback(int id,
                       const geometry_msgs::PoseStamped::ConstPtr& message);
  void MarkerCallback(int obstacle_id, int marker_id,
                      const geometry_msgs::PoseStamped::ConstPtr& message);
  void RefreshGeometry(const ros::Time& now);
  Status StatusLocked(const ros::Time& now) const;

  Pose2d TransformVehicle(const geometry_msgs::PoseStamped& pose) const;
  common::math::Vec2d TransformPoint(double x, double y) const;

  mutable std::mutex mutex_;
  SceneSnapshot static_scene_;
  std::map<int, Sample> vehicles_;
  std::map<int, ObstacleTrack> obstacles_;
  std::map<std::string, ros::Subscriber> subscribers_;
  ros::Timer discovery_timer_;
  ros::Timer geometry_timer_;
  std::string vrpn_namespace_ = "/vrpn_client_node";
  double position_scale_ = 0.001;
  double rotation_ = 0.0;
  double translation_x_ = 0.0;
  double translation_y_ = 0.0;
  double yaw_offset_ = 0.0;
  double body_to_rear_x_ = 0.0;
  double body_to_rear_y_ = 0.0;
  double data_timeout_ = 0.25;
  double geometry_retention_ = 2.0;
  double obstacle_padding_ = 0.0;
  int minimum_obstacle_markers_ = 3;
  std::uint64_t revision_ = 0;
};

}  // namespace scene
