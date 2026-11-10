#include "visualize/scene_visualizer.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

#include "common/math/polygon2d.h"
#include "visualize/visualization_plot.h"

namespace visualization {

void SceneVisualizer::Initialize(ros::NodeHandle& node,
                                 const std::string& frame_id,
                                 const VehicleParameter& vehicle,
                                 const std::string& marker_topic) {
  vehicle_ = vehicle;
  VisualizationPlot::Init(node, frame_id, marker_topic);
}

void SceneVisualizer::QueueScene(const scene::SceneSnapshot& snapshot) const {
  QueueMap(snapshot);
  for (std::size_t i = 0; i < snapshot.obstacles.size(); ++i) {
    QueueObstacle(snapshot.obstacles[i], i);
  }
  QueueStart(snapshot.start);
  QueueGoal(snapshot.goal);
}

void SceneVisualizer::QueueMap(const scene::SceneSnapshot& snapshot) const {
  const common::math::Polygon2d ground(snapshot.boundary());
  VisualizationPlot::PlotFilledPolygon(ground, Color::White, 0,
                                       "scene_ground", 0.0);
  VisualizationPlot::PlotPolygon(ground, 0.015, Color::Black, 0,
                                 "scene_boundary");
}

void SceneVisualizer::QueueObstacle(const scene::SceneObstacle& obstacle,
                                    std::size_t index) const {
  const auto polygon = obstacle.polygon();
  const int marker_id = static_cast<int>(index);
  VisualizationPlot::PlotFilledPolygon(polygon, Color::Grey, marker_id,
                                       "scene_obstacle_fill", 0.01);
  VisualizationPlot::PlotPolygon(polygon, 0.012, Color::Black, marker_id,
                                 "scene_obstacle_outline");
  VisualizationPlot::PlotText(obstacle.x, obstacle.y, obstacle.id, 0.12,
                              Color::Black, marker_id,
                              "scene_obstacle_label");
}

void SceneVisualizer::QueuePose(const scene::Pose2d& pose,
                                const std::string& role,
                                const std::string& label,
                                const Color& color) const {
  const auto box = vehicle_.GenerateBox(
      common::math::Pose(pose.x, pose.y, pose.yaw));
  const std::string prefix = "scene_" + role;
  VisualizationPlot::PlotPolygon(common::math::Polygon2d(box), 0.015, color,
                                 0, prefix + "_body");
  VisualizationPlot::PlotPoints({pose.x}, {pose.y}, color, 0.04, 0,
                                prefix + "_rear_axle");
  const double arrow_length =
      vehicle_.wheel_base + vehicle_.front_hang + 0.04;
  VisualizationPlot::PlotArrow(
      pose.x, pose.y, pose.x + arrow_length * std::cos(pose.yaw),
      pose.y + arrow_length * std::sin(pose.yaw), color, 0,
      prefix + "_heading");
  const double label_x = (box.min_x() + box.max_x()) / 2.0;
  const double label_y = box.min_y() - 0.10;
  VisualizationPlot::PlotText(label_x, label_y, label, 0.08, color, 0,
                              prefix + "_label");
}

void SceneVisualizer::QueueStart(const scene::Pose2d& start,
                                 bool locked) const {
  QueuePose(start, "start", locked ? "START LOCKED" : "START",
            Color::Green);
}

void SceneVisualizer::QueueGoal(const scene::Pose2d& goal) const {
  QueuePose(goal, "goal", "GOAL", Color::Magenta);
}

void SceneVisualizer::QueueHybridPath(
    const planning::HybridAStartResult& path) const {
  if (path.x.empty() || path.x.size() != path.y.size()) return;
  std::vector<Color> colors(path.x.size(), Color::Green);
  for (std::size_t i = 0; i < colors.size() && i < path.gear.size(); ++i) {
    if (!path.gear[i]) colors[i] = Color::Cyan;
  }
  VisualizationPlot::Plot(path.x, path.y, 0.025, colors, 0,
                          "hybrid_astar_path");
}

void SceneVisualizer::QueueOptimizedTrajectory(
    const optimizer::Trajectory& trajectory, bool clear_existing) const {
  if (clear_existing) {
    VisualizationPlot::ClearMarkers("optimized_trajectory", 0, 1023);
  }
  if (trajectory.nodes.empty() || trajectory.intervals.empty()) return;
  std::vector<double> x{trajectory.nodes.front().x};
  std::vector<double> y{trajectory.nodes.front().y};
  bool forward = trajectory.intervals.front().gear;
  int marker_id = 0;
  for (std::size_t i = 0; i < trajectory.intervals.size(); ++i) {
    if (i + 1 >= trajectory.nodes.size()) break;
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

void SceneVisualizer::QueueOptimizedConfigurationPoints(
    const optimizer::Trajectory& trajectory) const {
  std::vector<double> x;
  std::vector<double> y;
  x.reserve(trajectory.nodes.size());
  y.reserve(trajectory.nodes.size());
  for (const auto& node : trajectory.nodes) {
    x.push_back(node.x);
    y.push_back(node.y);
  }
  VisualizationPlot::PlotSpheres(
      x, y, Color::Red, 0.012, 0,
      "trajectory_optimized_configurations", 0.05);
}

void SceneVisualizer::QueuePlaybackVehicle(const PlaybackState& state) const {
  const auto& pose = state.pose;
  const auto box = vehicle_.GenerateBox(
      common::math::Pose(pose.x, pose.y, pose.theta));
  const Color color = state.gear ? Color::Blue : Color::Red;
  VisualizationPlot::PlotPolygon(
      common::math::Polygon2d(box), 0.018, color, 0,
      "trajectory_vehicle_body");
  VisualizationPlot::PlotPoints(
      {pose.x}, {pose.y}, color, 0.045, 0,
      "trajectory_vehicle_rear_axle", 0.055);
  const double arrow_length =
      vehicle_.wheel_base + vehicle_.front_hang + 0.04;
  VisualizationPlot::PlotArrow(
      pose.x, pose.y,
      pose.x + arrow_length * std::cos(pose.theta),
      pose.y + arrow_length * std::sin(pose.theta), color, 0,
      "trajectory_vehicle_heading", 0.012, 0.032, 0.04, 0.06);

  std::ostringstream status;
  status << std::fixed << std::setprecision(3)
         << "t=" << state.trajectory_time << "/" << state.total_time
         << " s  interval=" << (state.interval_index + 1)
         << "/" << state.interval_count
         << "\nv=" << pose.v << " m/s  phi=" << pose.phy
         << " rad  w=" << pose.w << " rad/s  "
         << (state.gear ? "FORWARD" : "REVERSE");
  if (state.paused_at_end) status << "  END PAUSE";
  VisualizationPlot::PlotText(
      pose.x, box.max_y() + 0.18, status.str(), 0.065, color, 0,
      "trajectory_playback_status");
}

void SceneVisualizer::ClearPlanningResults() const {
  VisualizationPlot::ClearMarkers("hybrid_astar_path", 0, 0);
  ClearTrajectoryPlayback();
}

void SceneVisualizer::ClearTrajectoryPlayback() const {
  VisualizationPlot::ClearMarkers("optimized_trajectory", 0, 1023);
  VisualizationPlot::ClearMarkers("trajectory_optimized_configurations", 0, 0);
  VisualizationPlot::ClearMarkers("trajectory_vehicle_body", 0, 0);
  VisualizationPlot::ClearMarkers("trajectory_vehicle_rear_axle", 0, 0);
  VisualizationPlot::ClearMarkers("trajectory_vehicle_heading", 0, 0);
  VisualizationPlot::ClearMarkers("trajectory_playback_status", 0, 0);
}

void SceneVisualizer::Trigger() const { VisualizationPlot::Trigger(); }

}  // namespace visualization
