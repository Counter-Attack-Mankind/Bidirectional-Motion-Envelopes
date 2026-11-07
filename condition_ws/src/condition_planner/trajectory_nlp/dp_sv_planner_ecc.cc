#include "dp_sv_planner_ecc.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/util/vector.h"
#include <ros/ros.h>

namespace trajectory_nlp {

DpSVPlannerECC::DpSVPlannerECC(const DpSVConfig &config, const VehicleParam &vehicle)
    : config_(config), vehicle_(vehicle) {}

// Speed limit from phi: if |phi[closest_path_idx]| > 0.5*phimax → vhalf
void DpSVPlannerECC::EstimateSpeedLimitFromPhi(const States &liom_path) {
  int NS = config_.num_s_nodes;
  v_ub_list_.clear();
  v_ub_list_.resize(NS + 1, vehicle_.max_velocity);

  s_list_.resize(NS + 1);
  s_resolution_ = (s_list_from_path_.back() - s_list_from_path_.front()) / NS;
  for (int i = 0; i <= NS; ++i) {
    s_list_[i] = s_list_from_path_.front() + i * s_resolution_;
  }

  const double phi_threshold = 0.5 * vehicle_.max_steer_angle;
  const double vhalf = vehicle_.max_velocity * config_.v_reduction_rate;

  for (int i = 0; i <= NS; ++i) {
    int ind = FindClosestIndex(s_list_from_path_, s_list_[i]);
    if (std::abs(liom_path.phi[ind]) > phi_threshold) {
      v_ub_list_[i] = vhalf;
    }
  }

  // Count limited nodes for logging
  int n_limited = 0;
  for (int i = 0; i <= NS; ++i) {
    if (v_ub_list_[i] < vehicle_.max_velocity - 1e-6) n_limited++;
  }
  ROS_INFO("[DpSVPlannerECC] Speed limit: %d/%d nodes limited to %.3f m/s (phi_thresh=%.4f rad)",
           n_limited, NS + 1, vhalf, phi_threshold);
}

// Cost function: reference DP cost + jerk penalty between consecutive S steps
SVTransitionCost DpSVPlannerECC::GetCost(int parent_v_ind, int s_ind, int v_ind, double v_guide) {
  double parent_v = (s_ind > 0) ? velocity_vec_.at(s_ind).at(parent_v_ind) : start_v_;
  double velocity  = velocity_vec_.at(s_ind + 1).at(v_ind);

  double current_time = std::fabs(2.0 * s_resolution_ / (velocity + parent_v + 1e-9));

  double acc = (velocity * velocity - parent_v * parent_v) / (2.0 * s_resolution_);
  double parent_acc = 0.0;
  if (s_ind > 0 && parent_v_ind >= 0) {
    parent_acc = states_.at(s_ind - 1).at(parent_v_ind).acc;
  }

  double cost_v = config_.w_nominal_velocity * std::fabs(0.5 * (velocity + parent_v) - v_guide * config_.v_ratio);

  double cost_acc = ((acc - acc_max_) > 1e-9 || (acc - acc_min_) < -1e-9)
                    ? config_.dp_cost_max
                    : config_.w_acceleration * std::fabs(acc);

  double jerk = (s_ind == 0) ? 0.0 : std::fabs(acc - parent_acc) / std::max(current_time, 1e-6);
  double cost_jerk = config_.w_jerk * jerk;

  return {cost_v + cost_acc + cost_jerk, current_time, acc};
}

int DpSVPlannerECC::FindClosestIndex(const std::vector<double> &vec, double value) {
  auto it = std::min_element(vec.begin(), vec.end(),
                             [value](double a, double b) {
                               return std::abs(a - value) < std::abs(b - value);
                             });
  return std::distance(vec.begin(), it);
}

// Fine-grained interpolation → truncate → downsample to nfe points
void DpSVPlannerECC::GenerateIG(
    const std::vector<double> &speed_s, const std::vector<double> &speed_v,
    const std::vector<double> &speed_t, const States &liom_path,
    const TrajectoryPoint &start, States &result) {

  // Step 1: Fine-grained interpolation at dt_resolution
  std::vector<double> new_s, new_v, new_t;
  for (int i = 1; i < (int)speed_t.size(); ++i) {
    double dt = speed_t[i] - speed_t[i - 1];
    int nfe_local = std::max(1, (int)std::round(std::fabs(dt) / config_.dt_resolution));

    auto tmp_t = common::util::LinSpaced(speed_t[i - 1], speed_t[i], nfe_local);
    new_t.insert(new_t.end(), tmp_t.begin(), tmp_t.end() - 1);

    auto tmp_s = common::util::LinSpaced(speed_s[i - 1], speed_s[i], nfe_local);
    new_s.insert(new_s.end(), tmp_s.begin(), tmp_s.end() - 1);

    auto tmp_v = common::util::LinSpaced(speed_v[i - 1], speed_v[i], nfe_local);
    new_v.insert(new_v.end(), tmp_v.begin(), tmp_v.end() - 1);
  }
  new_t.push_back(speed_t.back());
  new_s.push_back(speed_s.back());
  new_v.push_back(speed_v.back());

  // Step 2: Truncate at config_.tf (safety cap; normally speed_t.back() < config_.tf)
  double tf = config_.tf;
  auto it = std::find_if(new_t.rbegin(), new_t.rend(),
                         [tf](double val) { return val <= tf; });
  int tf_threshold_ind = it.base() - new_t.begin();
  if (tf_threshold_ind < (int)new_t.size()) {
    bool use_next = (new_t[tf_threshold_ind] - tf) < (tf - new_t[tf_threshold_ind - 1]);
    tf_threshold_ind = use_next ? tf_threshold_ind : tf_threshold_ind - 1;
  }
  tf_threshold_ind = std::max(0, std::min(tf_threshold_ind, (int)new_t.size() - 1));

  new_t.resize(tf_threshold_ind + 1);
  new_s.resize(tf_threshold_ind + 1);
  new_v.resize(tf_threshold_ind + 1);

  // Step 3: Downsample to config_.nfe points
  // BUG FIX vs reference: use (size-1) as end to avoid out-of-bounds index
  auto index = common::util::LinSpaced(0.0, (double)(new_t.size() - 1), config_.nfe);

  std::vector<double> s_index(config_.nfe), t_index(config_.nfe), v_index(config_.nfe);
  for (int i = 0; i < config_.nfe; ++i) {
    int ind = static_cast<int>(std::round(index[i]));
    ind = std::min(ind, (int)new_t.size() - 1);
    s_index[i] = new_s[ind];
    t_index[i] = new_t[ind];
    v_index[i] = new_v[ind];
  }

  // Step 4: Build output States
  result.tf = t_index.back();
  result.t  = t_index;
  result.x.resize(config_.nfe);
  result.y.resize(config_.nfe);
  result.theta.resize(config_.nfe);
  result.v.resize(config_.nfe);
  result.phi.resize(config_.nfe, 0.0);
  result.a.resize(config_.nfe, 0.0);
  result.omega.resize(config_.nfe, 0.0);

  // Map arc length → position from liom_path (which is already kinematically feasible)
  for (int i = 0; i < config_.nfe; ++i) {
    int ind = FindClosestIndex(s_list_from_path_, s_index[i]);
    result.x[i]     = liom_path.x[ind];
    result.y[i]     = liom_path.y[ind];
    result.theta[i] = liom_path.theta[ind];
    result.v[i]     = v_index[i];
  }

  // Enforce boundary conditions exactly
  result.x[0]              = liom_path.x.front();
  result.y[0]              = liom_path.y.front();
  result.theta[0]          = start.theta;
  result.v[0]              = start.v;
  result.x[config_.nfe - 1]     = liom_path.x.back();
  result.y[config_.nfe - 1]     = liom_path.y.back();
  result.theta[config_.nfe - 1] = liom_path.theta.back();
  result.v[config_.nfe - 1]     = 0.0;

  // Compute phi[i] from theta/v differences (kinematic: tan(phi) = dtheta/ds * L)
  result.phi[0] = start.phi;
  for (int i = 1; i < config_.nfe - 1; ++i) {
    double dt_i = t_index[i] - t_index[i - 1];
    if (dt_i < 1e-6) dt_i = 1e-6;
    if (std::abs(result.v[i]) > 1e-3) {
      double dphi_arg = (result.theta[i + 1] - result.theta[i]) * vehicle_.wheel_base
                        / (dt_i * result.v[i]);
      result.phi[i] = std::atan(dphi_arg);
      result.phi[i] = std::min(std::max(result.phi[i], -vehicle_.max_steer_angle),
                               vehicle_.max_steer_angle);
    }
  }
  result.phi[config_.nfe - 1] = 0.0;

  // Compute a[i] = (v[i+1] - v[i]) / dt
  result.a[0] = start.a;
  for (int i = 0; i < config_.nfe - 1; ++i) {
    double dt_i = t_index[i + 1] - t_index[i];
    if (dt_i < 1e-6) dt_i = 1e-6;
    double dv = result.v[i + 1] - result.v[i];
    result.a[i] = std::min(std::max(dv / dt_i, vehicle_.max_deceleration),
                           vehicle_.max_acceleration);
  }
  result.a[config_.nfe - 1] = 0.0;

  // Compute omega[i] = (phi[i+1] - phi[i]) / dt
  result.omega[0] = start.omega;
  for (int i = 0; i < config_.nfe - 1; ++i) {
    double dt_i = t_index[i + 1] - t_index[i];
    if (dt_i < 1e-6) dt_i = 1e-6;
    double dphi = result.phi[i + 1] - result.phi[i];
    result.omega[i] = std::min(std::max(dphi / dt_i, -vehicle_.max_steer_angle_rate),
                               vehicle_.max_steer_angle_rate);
  }
  result.omega[config_.nfe - 1] = 0.0;

  ROS_INFO("[DpSVPlannerECC] GenerateIG done: tf=%.3f s, %d points, s_total=%.3f m",
           result.tf, config_.nfe, s_index.back());
}

// Main DP planning function
bool DpSVPlannerECC::Plan(const States &liom_path, const TrajectoryPoint &start, States &result) {
  int n_path = (int)liom_path.x.size();
  if (n_path < 2) {
    ROS_ERROR("[DpSVPlannerECC] liom_path too short: %d points", n_path);
    return false;
  }

  acc_max_ = vehicle_.max_acceleration  * config_.a_ratio;
  acc_min_ = vehicle_.max_deceleration  * config_.a_ratio;
  start_v_ = start.v;

  int NS = config_.num_s_nodes;
  int NV = config_.num_v_nodes;

  // Build cumulative arc length array for the LIOM path
  s_list_from_path_.resize(n_path, 0.0);
  for (int i = 1; i < n_path; ++i) {
    double ds = std::hypot(liom_path.x[i] - liom_path.x[i - 1],
                           liom_path.y[i] - liom_path.y[i - 1]);
    s_list_from_path_[i] = s_list_from_path_[i - 1] + ds;
  }

  // Estimate speed limits at each DP S node based on phi from liom_path
  EstimateSpeedLimitFromPhi(liom_path);

  // Build velocity candidate sets for each S node
  velocity_vec_.resize(NS + 1);
  for (int i = 0; i <= NS; ++i) {
    int tmp_nv = static_cast<int>(std::round(
        v_ub_list_[i] / vehicle_.max_velocity * (NV - 2))) + 2;
    velocity_vec_[i] = common::util::LinSpaced(config_.v_min, v_ub_list_[i], tmp_nv - 1);
    velocity_vec_[i].push_back(v_ub_list_[i] * config_.v_ratio);
  }

  // Allocate DP table: states_[i] ↔ S node (i+1), size = velocity_vec_[i+1].size()
  states_.clear();
  states_.resize(NS);
  for (int i = 0; i < NS; ++i) {
    states_[i].resize(velocity_vec_[i + 1].size());
  }

  // DP forward pass: first S step (from S=0 to S=1)
  for (int i = 0; i < (int)states_.front().size(); ++i) {
    auto tmp = GetCost(-1, 0, i, v_ub_list_[1]);
    states_[0][i].cost         = tmp.cost;
    states_[0][i].t            = tmp.time;
    states_[0][i].s            = s_list_[1];
    states_[0][i].v            = velocity_vec_[1][i];
    states_[0][i].acc          = tmp.acc;
    states_[0][i].parent_v_ind = -1;
  }

  // DP forward pass: S steps 1 to NS-1
  for (int i = 0; i < NS - 1; ++i) {
    for (int j = 0; j < (int)states_[i].size(); ++j) {
      for (int k = 0; k < (int)states_[i + 1].size(); ++k) {
        auto tmp = GetCost(j, i + 1, k, v_ub_list_[i + 2]);
        double cur_cost = states_[i][j].cost + tmp.cost;
        if (cur_cost < states_[i + 1][k].cost) {
          states_[i + 1][k].cost         = cur_cost;
          states_[i + 1][k].parent_v_ind = j;
          states_[i + 1][k].t            = states_[i][j].t + tmp.time;
          states_[i + 1][k].v            = velocity_vec_[i + 2][k];
          states_[i + 1][k].s            = s_list_[i + 2];
          states_[i + 1][k].acc          = tmp.acc;
        }
      }
    }
  }

  // Find best final state
  int cur_best_v_ind = 0;
  for (int j = 0; j < (int)states_.back().size(); ++j) {
    if (states_[NS - 1][j].cost < states_[NS - 1][cur_best_v_ind].cost) {
      cur_best_v_ind = j;
    }
  }
  double best_cost = states_[NS - 1][cur_best_v_ind].cost;

  // Backtrack optimal path
  std::vector<double> speed_s(NS + 1, 0.0), speed_v(NS + 1, 0.0), speed_t(NS + 1, 0.0);
  speed_s[0] = s_list_[0];
  speed_v[0] = start_v_;
  for (int i = NS; i > 0; --i) {
    speed_s[i] = states_[i - 1][cur_best_v_ind].s;
    speed_t[i] = states_[i - 1][cur_best_v_ind].t;
    speed_v[i] = states_[i - 1][cur_best_v_ind].v;
    cur_best_v_ind = states_[i - 1][cur_best_v_ind].parent_v_ind;
  }

  ROS_INFO("[DpSVPlannerECC] DP done: best_cost=%.1f, total_time=%.3f s, w_jerk=%.3f",
           best_cost, speed_t.back(), config_.w_jerk);

  GenerateIG(speed_s, speed_v, speed_t, liom_path, start, result);

  return best_cost < config_.dp_cost_max;
}

}  // namespace trajectory_nlp
