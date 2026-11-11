#include "planning/planning_config_loader.h"

#include "optimizer/embodied_footprint.h"

namespace planning_platform {

bool LoadVehicleParameter(const ros::NodeHandle& node,
                          VehicleParameter* vehicle, std::string* error) {
  if (!vehicle || !node.getParam("wheel_base", vehicle->wheel_base) ||
      !node.getParam("front_hang", vehicle->front_hang) ||
      !node.getParam("rear_hang", vehicle->rear_hang) ||
      !node.getParam("width", vehicle->width) ||
      !node.getParam("max_velocity", vehicle->v_max) ||
      !node.getParam("max_acceleration", vehicle->a_max) ||
      !node.getParam("max_phi", vehicle->delta_max) ||
      !node.getParam("max_omega", vehicle->omega_max)) {
    if (error) *error = "Missing vehicle parameters in planner.yaml";
    return false;
  }
  vehicle->steer_ratio = 1.0;
  vehicle->GenerateDisc();
  return true;
}

bool LoadPlanningConfiguration(const ros::NodeHandle& node,
                               const std::array<double, 4>& bounds,
                               PlanningConfiguration* configuration,
                               std::string* error) {
  if (!configuration) {
    if (error) *error = "Planning configuration output is null";
    return false;
  }
  auto& vehicle = configuration->vehicle;
  if (!LoadVehicleParameter(node, &vehicle, error)) return false;

  auto& planner = configuration->planner;
  node.param("xy_grid_resolution", planner.xy_grid_resolution, 0.02);
  node.param("phi_grid_resolution", planner.phi_grid_resolution, 0.05);
  node.param("step_size", planner.step_size, 0.03);
  node.param("traj_forward_penalty", planner.traj_forward_penalty, 1.0);
  node.param("traj_back_penalty", planner.traj_back_penalty, 1.5);
  node.param("traj_gear_switch_penalty", planner.traj_gear_switch_penalty, 10.0);
  node.param("traj_steer_penalty", planner.traj_steer_penalty, 1.0);
  node.param("traj_steer_change_penalty", planner.traj_steer_change_penalty, 5.0);
  node.param("grid_a_star_xy_resolution", planner.grid_a_star_xy_resolution, 0.03);
  node.param("delta_t", planner.delta_t, 0.5);
  node.param("analytic_expansion_cost", planner.analytic_expansion_cost, 20.0);
  node.param("dubins_phi_gamma", planner.dubins_phi_gamma, 1.1);
  int next_node_num = 20;
  int explore_node_num = 2000000;
  node.param("next_node_num", next_node_num, next_node_num);
  node.param("explore_node_num", explore_node_num, explore_node_num);
  planner.next_node_num = static_cast<std::size_t>(next_node_num);
  planner.explore_node_num = static_cast<std::size_t>(explore_node_num);
  planner.xy_bounds = bounds;

  auto& initial = configuration->initial_guess;
  node.param("min_dt", initial.min_dt, 0.0001);
  node.param("max_dt", initial.max_dt, 0.0500);
  node.param("temporal_tightening_eta", initial.eta, 0.90);
  node.param("max_velocity", initial.forward_speed, 0.20);
  node.param("max_reverse_velocity", initial.reverse_speed, 0.16);
  initial.max_acceleration = vehicle.a_max;
  initial.max_steering_rate = vehicle.omega_max;

  auto& nlp = configuration->nlp;
  nlp.min_dt = initial.min_dt;
  nlp.max_dt = initial.max_dt;
  node.param("dmin", nlp.dmin, 0.005);
  node.param("total_time_limit", nlp.total_time_limit, 35.0);
  node.param("ipopt_max_iterations", nlp.max_iterations, 399984);
  node.param("ipopt_max_cpu_time", nlp.max_cpu_time, 20.0);
  node.param("ipopt_tolerance", nlp.tolerance, 1e-3);
  node.param("ipopt_linear_solver", nlp.linear_solver, std::string("ma27"));
  node.param("save_debug_data", nlp.save_debug_data, false);
  node.param("results_directory", configuration->results_root,
             std::string("results"));

  std::string geometry_error;
  if (next_node_num < 4 || next_node_num % 2 != 0 || explore_node_num <= 0 ||
      planner.xy_grid_resolution <= 0.0 || planner.phi_grid_resolution <= 0.0 ||
      planner.step_size <= 0.0 || planner.grid_a_star_xy_resolution <= 0.0 ||
      planner.delta_t <= 0.0 || initial.forward_speed <= 0.0 ||
      initial.reverse_speed <= 0.0 || nlp.dmin <= 0.0 ||
      nlp.max_cpu_time <= 0.0 ||
      !optimizer::CheckVehicleGeometryCondition(vehicle, &geometry_error)) {
    if (error) *error = geometry_error.empty() ?
        "Invalid planning configuration" : geometry_error;
    return false;
  }
  return true;
}

}  // namespace planning_platform
