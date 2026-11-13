#include "visualize/scene_visualizer.h"

#include <algorithm>
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
  VisualizationPlot::PlotText(obstacle.x, obstacle.y,
                              std::to_string(index + 1), 0.08, Color::Black,
                              marker_id, "scene_obstacle_label");
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

void SceneVisualizer::QueuePlanningStage(
    const std::array<double, 4>& bounds, const std::string& text) const {
  if (text.empty()) {
    ClearPlanningStage();
    return;
  }
  const double map_width = std::max(0.1, bounds[1] - bounds[0]);
  const double map_height = std::max(0.1, bounds[3] - bounds[2]);
  const double margin = std::min(0.10, 0.04 * map_height);
  const double available_width = std::max(0.1, map_width - 2.0 * margin);
  const double nominal_scale = std::min(0.075, 0.045 * map_height);
  const double estimated_units = std::max<std::size_t>(1, text.size()) * 0.55;
  const double text_scale = std::max(
      0.04, std::min(nominal_scale, available_width / estimated_units));
  const double estimated_width =
      std::min(available_width, estimated_units * text_scale);
  const double x = bounds[0] + margin + 0.5 * estimated_width;
  const double y = bounds[3] - margin - 0.5 * text_scale;
  VisualizationPlot::PlotText(x, y, text, text_scale, Color::Black, 0,
                              "planning_stage_status");
}

void SceneVisualizer::ClearPlanningStage() const {
  VisualizationPlot::ClearMarkers("planning_stage_status", 0, 0);
}

void SceneVisualizer::QueueHybridPath(
    const planning::HybridAStartResult& path) const {
  if (path.x.empty() || path.x.size() != path.y.size()) return;
  ClearHybridSearch();
  std::vector<Color> colors(path.x.size(), Color::Green);
  for (std::size_t i = 0; i < colors.size() && i < path.gear.size(); ++i) {
    if (!path.gear[i]) colors[i] = Color::Cyan;
  }
  VisualizationPlot::Plot(path.x, path.y, 0.025, colors, 0,
                          "hybrid_astar_path");
}

void SceneVisualizer::QueueHybridSearch(
    const planning::HybridSearchProgress& progress) const {
  if (!progress.expanded_x.empty()) {
    VisualizationPlot::PlotPoints(
        progress.expanded_x, progress.expanded_y,
        Color::fromRGB(0.72f, 0.72f, 0.72f), 0.018, 0,
        "hybrid_search_expanded", 0.025);
  }
  if (progress.backtrack_x.size() >= 2) {
    VisualizationPlot::Plot(progress.backtrack_x, progress.backtrack_y, 0.012,
                            Color::fromRGB(1.0f, 0.55f, 0.0f), 0,
                            "hybrid_search_backtrack");
  }
  if (progress.has_current) {
    constexpr double kArrowLength = 0.16;
    VisualizationPlot::PlotArrow(
        progress.current_x, progress.current_y,
        progress.current_x + kArrowLength * std::cos(progress.current_phi),
        progress.current_y + kArrowLength * std::sin(progress.current_phi),
        Color::Yellow, 0, "hybrid_search_current", 0.010, 0.028,
        0.035, 0.05);
  }

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
  ClearHybridSearch();
  ClearTrajectoryPlayback();
}

void SceneVisualizer::ClearHybridSearch() const {
  VisualizationPlot::ClearMarkers("hybrid_search_expanded", 0, 0);
  VisualizationPlot::ClearMarkers("hybrid_search_backtrack", 0, 0);
  VisualizationPlot::ClearMarkers("hybrid_search_current", 0, 0);
  VisualizationPlot::ClearMarkers("hybrid_search_status", 0, 0);
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
