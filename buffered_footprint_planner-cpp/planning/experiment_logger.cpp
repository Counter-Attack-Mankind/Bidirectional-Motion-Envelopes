#include "planning/experiment_logger.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "planning/planning_pipeline.h"
#include "scene/scene_storage.h"

namespace experiment {
namespace {

template <typename T>
T ValueAt(const std::vector<T>& values, std::size_t index, T fallback = T{}) {
  return index < values.size() ? values[index] : fallback;
}

}  // namespace

bool ExperimentLogger::CreateRunDirectory(const std::string& root,
                                          const std::string& mode,
                                          int task_id,
                                          std::string* directory,
                                          std::string* error) const {
  namespace fs = std::filesystem;
  std::error_code code;
  fs::create_directories(root, code);
  if (code) {
    if (error) *error = "Cannot create results root: " + code.message();
    return false;
  }
  const auto now = std::chrono::system_clock::now();
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()).count();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
#ifdef _WIN32
  localtime_s(&local, &time);
#else
  localtime_r(&time, &local);
#endif
  for (int sequence = 0; sequence < 10000; ++sequence) {
    std::ostringstream name;
    name << std::put_time(&local, "%Y%m%d_%H%M%S") << '_'
         << (milliseconds % 1000) << '_' << mode << "_task" << task_id
         << '_' << sequence;
    const fs::path candidate = fs::path(root) / name.str();
    if (fs::create_directory(candidate, code)) {
      if (directory) *directory = candidate.string();
      return true;
    }
    if (code) {
      if (error) *error = "Cannot create result directory: " + code.message();
      return false;
    }
  }
  if (error) *error = "Cannot allocate a unique result directory";
  return false;
}

bool ExperimentLogger::WriteSceneCopy(const std::string& directory,
                                      const scene::SceneSnapshot& snapshot,
                                      std::string* error) const {
  return scene::SceneStorage::Save(
      (std::filesystem::path(directory) / "scene.yaml").string(), snapshot,
      error);
}

bool ExperimentLogger::WriteEffectiveParameters(
    const std::string& directory,
    const planning_platform::PlanningConfiguration& c,
    std::string* error) const {
  std::ofstream out(std::filesystem::path(directory) / "effective_parameters.yaml");
  if (!out) {
    if (error) *error = "Cannot write effective parameters";
    return false;
  }
  out << std::setprecision(17)
      << "vehicle:\n"
      << "  wheel_base: " << c.vehicle.wheel_base << '\n'
      << "  front_hang: " << c.vehicle.front_hang << '\n'
      << "  rear_hang: " << c.vehicle.rear_hang << '\n'
      << "  width: " << c.vehicle.width << '\n'
      << "  v_max: " << c.vehicle.v_max << '\n'
      << "  a_max: " << c.vehicle.a_max << '\n'
      << "  max_phi: " << c.vehicle.delta_max << '\n'
      << "  max_omega: " << c.vehicle.omega_max << '\n'
      << "planner:\n"
      << "  xy_grid_resolution: " << c.planner.xy_grid_resolution << '\n'
      << "  phi_grid_resolution: " << c.planner.phi_grid_resolution << '\n'
      << "  next_node_num: " << c.planner.next_node_num << '\n'
      << "  step_size: " << c.planner.step_size << '\n'
      << "  traj_forward_penalty: " << c.planner.traj_forward_penalty << '\n'
      << "  traj_back_penalty: " << c.planner.traj_back_penalty << '\n'
      << "  traj_gear_switch_penalty: " << c.planner.traj_gear_switch_penalty << '\n'
      << "  traj_steer_penalty: " << c.planner.traj_steer_penalty << '\n'
      << "  traj_steer_change_penalty: " << c.planner.traj_steer_change_penalty << '\n'
      << "  grid_a_star_xy_resolution: " << c.planner.grid_a_star_xy_resolution << '\n'
      << "  delta_t: " << c.planner.delta_t << '\n'
      << "  analytic_expansion_cost: " << c.planner.analytic_expansion_cost << '\n'
      << "  dubins_phi_gamma: " << c.planner.dubins_phi_gamma << '\n'
      << "  explore_node_num: " << c.planner.explore_node_num << '\n'
      << "  hybrid_astar_timeout: " << c.planner.hybrid_astar_timeout << '\n'
      << "initial_guess:\n"
      << "  min_dt: " << c.initial_guess.min_dt << '\n'
      << "  max_dt: " << c.initial_guess.max_dt << '\n'
      << "  temporal_tightening_eta: " << c.initial_guess.eta << '\n'
      << "  forward_speed: " << c.initial_guess.forward_speed << '\n'
      << "  reverse_speed: " << c.initial_guess.reverse_speed << '\n'
      << "  max_acceleration: " << c.initial_guess.max_acceleration << '\n'
      << "  max_steering_rate: " << c.initial_guess.max_steering_rate << '\n'
      << "optimizer:\n"
      << "  min_dt: " << c.nlp.min_dt << '\n'
      << "  max_dt: " << c.nlp.max_dt << '\n'
      << "  dmin: " << c.nlp.dmin << '\n'
      << "  total_time_limit: " << c.nlp.total_time_limit << '\n'
      << "  tol: " << c.nlp.tolerance << '\n'
      << "  max_iter: " << c.nlp.max_iterations << '\n'
      << "  max_cpu_time: " << c.nlp.max_cpu_time << '\n'
      << "  linear_solver: " << c.nlp.linear_solver << '\n'
      << "  hessian_approximation: exact\n"
      << "  mu_strategy: adaptive\n"
      << "  console_print_level: 0\n"
      << "  file_print_level: 5\n"
      << "  save_debug_data: " << std::boolalpha << c.nlp.save_debug_data
      << '\n';
  return static_cast<bool>(out);
}

bool ExperimentLogger::WriteSummary(
    const std::string& directory, const std::string& mode, int task_id,
    const planning_platform::PlanningRunResult& result,
    std::string* error) const {
  std::ofstream out(std::filesystem::path(directory) / "summary.txt");
  if (!out) {
    if (error) *error = "Cannot write experiment summary";
    return false;
  }
  const double path_length = result.hybrid_path.accumulated_s.empty()
      ? 0.0 : result.hybrid_path.accumulated_s.back();
  const auto& initial = result.initial_diagnostics;
  const auto& nlp = result.optimization;
  out << std::setprecision(17) << std::boolalpha
      << "package=buffered_footprint_planner\n"
      << "package_version=0.1.0\n"
      << "mode=" << mode << '\n'
      << "task_id=" << task_id << '\n'
      << "cxx_standard=17\n"
      << "build_date=" << __DATE__ << '\n'
      << "build_time=" << __TIME__ << '\n'
      << "git_commit=not_embedded\n"
      << "scene_valid=" << result.scene_valid << '\n'
      << "hybrid_astar_succeeded=" << result.hybrid_succeeded << '\n'
      << "hybrid_astar_timed_out=" << result.hybrid_timed_out << '\n'
      << "hybrid_astar_time_s=" << result.hybrid_elapsed << '\n'
      << "hybrid_astar_points=" << result.hybrid_path.x.size() << '\n'
      << "hybrid_astar_path_length_m=" << path_length << '\n'
      << "initial_guess_succeeded=" << result.initial_guess_succeeded << '\n'
      << "initial_guess_time_s=" << result.initial_guess_elapsed << '\n'
      << "initial_guess_nodes=" << result.initial_guess.nodes.size() << '\n'
      << "initial_guess_dense_nodes=" << initial.dense_node_count << '\n'
      << "initial_guess_min_dt_s=" << initial.min_dt << '\n'
      << "initial_guess_max_dt_s=" << initial.max_dt << '\n'
      << "initial_guess_total_time_s=" << initial.total_time << '\n'
      << "initial_guess_max_euler_residual=" << initial.max_euler_residual << '\n'
      << "initial_guess_max_footprint_violation="
      << initial.max_footprint_violation << '\n'
      << "nlp_solver_succeeded=" << nlp.solver_succeeded << '\n'
      << "nlp_validated_success=" << nlp.success << '\n'
      << "nlp_status=" << nlp.status << '\n'
      << "nlp_iterations=" << nlp.iterations << '\n'
      << "nlp_solve_time_s=" << nlp.solve_time << '\n'
      << "nlp_objective=" << nlp.objective << '\n'
      << "nlp_nodes=" << result.initial_guess.nodes.size() << '\n'
      << "nlp_solver_stats_available=" << nlp.solver_stats_available << '\n'
      << "nlp_solve_time_source="
      << (nlp.solve_time_from_stats ? "opti.stats.t_wall_total"
                                    : "measured_wall_clock") << '\n'
      << "nlp_max_dynamics_residual=" << nlp.max_dynamics_residual << '\n'
      << "nlp_max_dynamics_x_m=" << nlp.diagnostics.dynamics_x_m.value << '\n'
      << "nlp_max_dynamics_y_m=" << nlp.diagnostics.dynamics_y_m.value << '\n'
      << "nlp_max_dynamics_theta_rad="
      << nlp.diagnostics.dynamics_theta_rad.value << '\n'
      << "nlp_max_dynamics_velocity_mps="
      << nlp.diagnostics.dynamics_v_mps.value << '\n'
      << "nlp_max_dynamics_steering_rad="
      << nlp.diagnostics.dynamics_phi_rad.value << '\n'
      << "nlp_max_variable_bound_violation="
      << nlp.max_variable_bound_violation << '\n'
      << "nlp_max_time_step_bound_s="
      << nlp.diagnostics.time_step_bound_s.value << '\n'
      << "nlp_max_footprint_validity_violation="
      << nlp.max_footprint_validity_violation << '\n'
      << "nlp_max_ef_distance_equality_m="
      << nlp.diagnostics.ef_distance_equality_m.value << '\n'
      << "nlp_max_ef_corner_equality_m="
      << nlp.diagnostics.ef_corner_equality_m.value << '\n'
      << "nlp_max_obca_violation=" << nlp.max_obca_violation << '\n'
      << "nlp_max_obca_separation_m="
      << nlp.diagnostics.obca_separation_m.value << '\n'
      << "nlp_max_obca_dual_equality="
      << nlp.diagnostics.obca_dual_equality.value << '\n'
      << "nlp_max_map_boundary_violation="
      << nlp.max_map_boundary_violation << '\n'
      << "nlp_last_iterate_available=" << nlp.last_iterate_available << '\n'
      << "run_success=" << result.success << '\n'
      << "run_status=" << result.status << '\n';
  return static_cast<bool>(out);
}

bool ExperimentLogger::WriteHybridPath(
    const std::string& directory, const planning::HybridAStartResult& path,
    std::string* error) const {
  std::ofstream out(std::filesystem::path(directory) / "hybrid_astar_path.csv");
  if (!out) {
    if (error) *error = "Cannot write Hybrid A* path";
    return false;
  }
  out << std::setprecision(17)
      << "index,x_m,y_m,theta_rad,gear,v_mps,a_mps2,steer_rad,accumulated_s_m\n";
  const std::size_t count = path.x.size();
  for (std::size_t i = 0; i < count; ++i) {
    out << i << ',' << path.x[i] << ',' << ValueAt(path.y, i) << ','
        << ValueAt(path.phi, i) << ',' << (ValueAt(path.gear, i, true) ? 1 : -1)
        << ',' << ValueAt(path.v, i) << ',' << ValueAt(path.a, i) << ','
        << ValueAt(path.steer, i) << ',' << ValueAt(path.accumulated_s, i) << '\n';
  }
  return static_cast<bool>(out);
}

bool ExperimentLogger::WriteDenseTrajectory(
    const std::string& directory, const optimizer::InitialGuessTrace& trace,
    std::string* error) const {
  std::ofstream out(std::filesystem::path(directory) / "dense_trajectory.csv");
  if (!out) {
    if (error) *error = "Cannot write dense trajectory";
    return false;
  }
  out << std::setprecision(17)
      << "dense_index,time_s,x_m,y_m,theta_rad,v_mps,a_mps2,phy_rad,w_radps,"
         "phi_raw_rad,phi_clipped_rad,w_raw_radps,w_clipped_radps\n";
  for (const auto& sample : trace.dense_samples) {
    const auto& node = sample.node;
    out << sample.dense_index << ',' << sample.time << ',' << node.x << ','
        << node.y << ',' << node.theta << ',' << node.v << ',' << node.a << ','
        << node.phy << ',' << node.w << ',' << sample.phi_raw << ','
        << sample.phi_clipped << ',' << sample.w_raw << ','
        << sample.w_clipped << '\n';
  }
  return static_cast<bool>(out);
}

bool ExperimentLogger::WriteInitialGuess(
    const std::string& directory, const optimizer::Trajectory& trajectory,
    const optimizer::InitialGuessTrace& trace, std::string* error) const {
  std::ofstream out(std::filesystem::path(directory) / "initial_guess.csv");
  if (!out) {
    if (error) *error = "Cannot write initial guess";
    return false;
  }
  out << std::setprecision(17)
      << "index,source_dense_index,time_s,x_m,y_m,theta_rad,v_mps,a_mps2,"
         "phy_rad,w_radps,dt_s,gear,phi_raw_rad,phi_clipped_rad,w_raw_radps,"
         "w_clipped_radps\n";
  double time = 0.0;
  for (std::size_t i = 0; i < trajectory.nodes.size(); ++i) {
    const auto& node = trajectory.nodes[i];
    const std::size_t dense_index = i < trace.selected_dense_indices.size()
        ? trace.selected_dense_indices[i] : 0;
    const auto* dense = dense_index < trace.dense_samples.size()
        ? &trace.dense_samples[dense_index] : nullptr;
    double raw_w = 0.0;
    if (i < trajectory.intervals.size() && trajectory.intervals[i].dt > 0.0) {
      raw_w = (trajectory.nodes[i + 1].phy - node.phy) /
              trajectory.intervals[i].dt;
    }
    out << i << ',' << dense_index << ',' << time << ',' << node.x << ','
        << node.y << ',' << node.theta << ',' << node.v << ',' << node.a << ','
        << node.phy << ',' << node.w;
    if (i < trajectory.intervals.size()) {
      out << ',' << trajectory.intervals[i].dt << ','
          << (trajectory.intervals[i].gear ? 1 : -1);
      time += trajectory.intervals[i].dt;
    } else {
      out << ",0,0";
    }
    out << ',' << (dense ? dense->phi_raw : node.phy) << ',' << node.phy << ','
        << raw_w << ',' << node.w << '\n';
  }
  return static_cast<bool>(out);
}

}  // namespace experiment
