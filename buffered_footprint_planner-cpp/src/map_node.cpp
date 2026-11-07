#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "common/math/polygon2d.h"
#include "environment/my_env.h"
#include "hybrid_astar/hybrid_a_star.h"
#include "hybrid_astar/parking_planner_config.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "visualize/visualization_plot.h"

namespace {

struct VehicleLimits {
  double rear_hang = 0.0;
  double front_hang = 0.0;
  double wheel_base = 0.0;
  double length = 0.0;
  double width = 0.0;
  double max_velocity = 0.0;
  double max_reverse_velocity = 0.0;
  double max_acceleration = 0.0;
  double max_deceleration = 0.0;
  double max_phi = 0.0;
  double min_phi = 0.0;
  double max_omega = 0.0;
};

class MapNode {
 public:
  MapNode() : private_node_("~") {}

  bool Initialize() {
    std::string env_file;
    if (!private_node_.getParam("env_file", env_file) || env_file.empty()) {
      ROS_FATAL("Required private parameter '~env_file' is missing or empty");
      return false;
    }
    if (!ReadVehicleParameters() || !ReadPlannerParameters()) return false;
    if (!environment_.Read(env_file)) return false;
    if (!environment_.has_start || !environment_.has_goal) {
      ROS_FATAL("Environment must contain both 'start' and 'goal'; zero coordinates are valid");
      return false;
    }

    ComputeBounds();
    vehicle_.wheel_base = limits_.wheel_base;
    vehicle_.front_hang = limits_.front_hang;
    vehicle_.rear_hang = limits_.rear_hang;
    vehicle_.width = limits_.width;
    vehicle_.v_max = limits_.max_velocity;
    vehicle_.a_max = limits_.max_acceleration;
    vehicle_.delta_max = limits_.max_phi;
    vehicle_.omega_max = limits_.max_omega;
    vehicle_.steer_ratio = 1.0;
    vehicle_.GenerateDisc();

    VisualizationPlot::Init(private_node_, "map", "/buffered_footprint_planner/markers");
    QueueEnvironmentMarkers();
    PlanAndQueuePath();
    VisualizationPlot::Trigger();

    ROS_INFO("Stage 1 node is ready (fixed frame: map, marker topic: /buffered_footprint_planner/markers)");
    return true;
  }

 private:
  bool RequiredDouble(const std::string& name, double* value) {
    if (!private_node_.getParam(name, *value) || !std::isfinite(*value)) {
      ROS_FATAL("Required finite parameter '~%s' is missing or invalid", name.c_str());
      return false;
    }
    return true;
  }

  bool ReadVehicleParameters() {
    bool ok = true;
    ok &= RequiredDouble("rear_hang", &limits_.rear_hang);
    ok &= RequiredDouble("front_hang", &limits_.front_hang);
    ok &= RequiredDouble("wheel_base", &limits_.wheel_base);
    ok &= RequiredDouble("length", &limits_.length);
    ok &= RequiredDouble("width", &limits_.width);
    ok &= RequiredDouble("max_velocity", &limits_.max_velocity);
    ok &= RequiredDouble("max_reverse_velocity", &limits_.max_reverse_velocity);
    ok &= RequiredDouble("max_acceleration", &limits_.max_acceleration);
    ok &= RequiredDouble("max_deceleration", &limits_.max_deceleration);
    ok &= RequiredDouble("max_phi", &limits_.max_phi);
    ok &= RequiredDouble("min_phi", &limits_.min_phi);
    ok &= RequiredDouble("max_omega", &limits_.max_omega);
    if (!ok) return false;

    if (limits_.rear_hang < 0.0 || limits_.front_hang < 0.0 ||
        limits_.wheel_base <= 0.0 || limits_.length <= 0.0 || limits_.width <= 0.0 ||
        limits_.max_velocity <= 0.0 || limits_.max_reverse_velocity < 0.0 ||
        limits_.max_acceleration <= 0.0 || limits_.max_deceleration >= 0.0 ||
        limits_.max_phi <= 0.0 || limits_.min_phi >= 0.0 || limits_.max_omega <= 0.0) {
      ROS_FATAL("Vehicle parameters contain invalid signs or non-positive dimensions");
      return false;
    }

    const double geometric_length = limits_.rear_hang + limits_.wheel_base + limits_.front_hang;
    if (std::abs(geometric_length - limits_.length) > 1e-6) {
      ROS_FATAL("Vehicle length mismatch: YAML length=%.9f, rear+wheelbase+front=%.9f",
                limits_.length, geometric_length);
      return false;
    }
    if (std::abs(limits_.max_phi + limits_.min_phi) > 1e-6) {
      ROS_FATAL("Stage 1 Hybrid A* expects symmetric steering bounds, got [%.6f, %.6f]",
                limits_.min_phi, limits_.max_phi);
      return false;
    }

    ROS_INFO("Vehicle: rear=%.3f front=%.3f wheelbase=%.3f length=%.3f width=%.3f",
             limits_.rear_hang, limits_.front_hang, limits_.wheel_base,
             limits_.length, limits_.width);
    ROS_INFO("Limits: v=[-%.3f, %.3f], a=[%.3f, %.3f], phi=[%.4f, %.4f], omega=%.3f",
             limits_.max_reverse_velocity, limits_.max_velocity,
             limits_.max_deceleration, limits_.max_acceleration,
             limits_.min_phi, limits_.max_phi, limits_.max_omega);
    return true;
  }

  bool ReadPlannerParameters() {
    private_node_.param("xy_grid_resolution", planner_config_.xy_grid_resolution, 0.02);
    private_node_.param("phi_grid_resolution", planner_config_.phi_grid_resolution, 0.05);
    private_node_.param("step_size", planner_config_.step_size, 0.03);
    private_node_.param("traj_forward_penalty", planner_config_.traj_forward_penalty, 1.0);
    private_node_.param("traj_back_penalty", planner_config_.traj_back_penalty, 1e6);
    private_node_.param("traj_gear_switch_penalty", planner_config_.traj_gear_switch_penalty, 1e6);
    private_node_.param("traj_steer_penalty", planner_config_.traj_steer_penalty, 1.0);
    private_node_.param("traj_steer_change_penalty", planner_config_.traj_steer_change_penalty, 5.0);
    private_node_.param("grid_a_star_xy_resolution", planner_config_.grid_a_star_xy_resolution, 0.03);
    private_node_.param("delta_t", planner_config_.delta_t, 0.5);
    private_node_.param("analytic_expansion_cost", planner_config_.analytic_expansion_cost, 20.0);
    private_node_.param("dubins_phi_gamma", planner_config_.dubins_phi_gamma, 1.1);

    int next_node_num = 20;
    int explore_node_num = 2000000;
    private_node_.param("next_node_num", next_node_num, next_node_num);
    private_node_.param("explore_node_num", explore_node_num, explore_node_num);
    if (next_node_num < 4 || next_node_num % 2 != 0 || explore_node_num <= 0 ||
        planner_config_.xy_grid_resolution <= 0.0 || planner_config_.phi_grid_resolution <= 0.0 ||
        planner_config_.step_size <= 0.0 || planner_config_.grid_a_star_xy_resolution <= 0.0 ||
        planner_config_.delta_t <= 0.0) {
      ROS_FATAL("Hybrid A* parameters are invalid");
      return false;
    }
    planner_config_.next_node_num = static_cast<std::size_t>(next_node_num);
    planner_config_.explore_node_num = static_cast<std::size_t>(explore_node_num);
    return true;
  }

  void ComputeBounds() {
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& point : environment_.border) {
      min_x = std::min(min_x, point.x());
      max_x = std::max(max_x, point.x());
      min_y = std::min(min_y, point.y());
      max_y = std::max(max_y, point.y());
    }
    planner_config_.xy_bounds = {min_x, max_x, min_y, max_y};
    ROS_INFO("Planning bounds: x=[%.3f, %.3f], y=[%.3f, %.3f]", min_x, max_x, min_y, max_y);
  }

  void QueueEnvironmentMarkers() const {
    std::vector<double> border_x;
    std::vector<double> border_y;
    border_x.reserve(environment_.border.size() + 1);
    border_y.reserve(environment_.border.size() + 1);
    for (const auto& point : environment_.border) {
      border_x.push_back(point.x());
      border_y.push_back(point.y());
    }
    if (!(environment_.border.front() == environment_.border.back())) {
      border_x.push_back(environment_.border.front().x());
      border_y.push_back(environment_.border.front().y());
    }
    VisualizationPlot::Plot(border_x, border_y, 0.01, Color::White, 0, "Border");

    for (std::size_t i = 0; i < environment_.obstacles.size(); ++i) {
      VisualizationPlot::PlotFilledPolygon(environment_.obstacles[i], Color::Red,
                                            static_cast<int>(i), "Obstacles");
    }

    VisualizationPlot::PlotPolygon(common::math::Polygon2d(vehicle_.GenerateBox(environment_.start)),
                                    0.012, Color::Blue, 0, "Start");
    VisualizationPlot::PlotPolygon(common::math::Polygon2d(vehicle_.GenerateBox(environment_.goal)),
                                    0.012, Color::Cyan, 0, "Goal");
  }

  bool ValidateForwardCollisionFreePath(const planning::HybridAStartResult& result) const {
    if (result.x.size() < 2 || result.x.size() != result.y.size() ||
        result.x.size() != result.phi.size()) {
      ROS_ERROR("Hybrid A* returned inconsistent or empty x/y/theta arrays");
      return false;
    }

    const auto& bounds = planner_config_.xy_bounds;
    for (std::size_t i = 0; i < result.x.size(); ++i) {
      if (result.x[i] < bounds[0] || result.x[i] > bounds[1] ||
          result.y[i] < bounds[2] || result.y[i] > bounds[3]) {
        ROS_ERROR("Path validation failed: reference point %zu leaves map bounds", i);
        return false;
      }
      const common::math::Pose pose(result.x[i], result.y[i], result.phi[i]);
      const auto box = vehicle_.GenerateBox(pose);
      const common::math::Polygon2d footprint(box);
      for (std::size_t obstacle = 0; obstacle < environment_.obstacles.size(); ++obstacle) {
        if (footprint.HasOverlap(environment_.obstacles[obstacle])) {
          ROS_ERROR("Path validation failed: point %zu collides with obstacle %zu", i, obstacle);
          return false;
        }
      }
    }

    for (std::size_t i = 0; i + 1 < result.x.size(); ++i) {
      const double dx = result.x[i + 1] - result.x[i];
      const double dy = result.y[i + 1] - result.y[i];
      const double signed_motion = dx * std::cos(result.phi[i]) + dy * std::sin(result.phi[i]);
      if (signed_motion < -1e-6) {
        ROS_ERROR("Forward-only validation failed at path segment %zu (projection %.9f)",
                  i, signed_motion);
        return false;
      }
    }
    return true;
  }

  void PlanAndQueuePath() {
    planning::HybridAStar planner(planner_config_, vehicle_, environment_.obstacles);
    planning::HybridAStartResult result;
    ROS_INFO("Planning forward-only Hybrid A*: start=(%.3f, %.3f, %.3f), goal=(%.3f, %.3f, %.3f)",
             environment_.start.x(), environment_.start.y(), environment_.start.theta(),
             environment_.goal.x(), environment_.goal.y(), environment_.goal.theta());

    if (!planner.Plan(environment_.start.x(), environment_.start.y(), environment_.start.theta(),
                      environment_.goal.x(), environment_.goal.y(), environment_.goal.theta(), &result)) {
      ROS_ERROR("Forward-only Hybrid A* failed; environment markers will remain visible");
      return;
    }
    if (!ValidateForwardCollisionFreePath(result)) {
      ROS_ERROR("Hybrid A* result was rejected by Stage 1 post-validation and will not be displayed");
      return;
    }

    VisualizationPlot::Plot(result.x, result.y, 0.025, Color::Green, 0, "HybridAStarPath");
    ROS_INFO("Forward-only Hybrid A* succeeded: %zu collision-free path points", result.x.size());
  }

  ros::NodeHandle private_node_;
  MyEnvironment environment_;
  VehicleLimits limits_;
  ParkingPlannerConfig planner_config_;
  VehicleParameter vehicle_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_map");
  MapNode node;
  if (!node.Initialize()) return EXIT_FAILURE;
  ros::spin();
  return EXIT_SUCCESS;
}
