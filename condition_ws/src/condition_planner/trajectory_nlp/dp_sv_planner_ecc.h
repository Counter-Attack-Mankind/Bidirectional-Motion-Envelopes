#pragma once
#include <vector>
#include "common/vehicle_param.h"
#include "common/trajectory_point.h"
#include "trajectory_nlp.h"  // for States

namespace trajectory_nlp {

struct DpSVConfig {
  double tf = 60.0;           // planning horizon (> expected DP traversal time)
  int nfe = 100;              // must match NLP nfe
  int num_s_nodes = 80;       // S nodes in DP grid
  int num_v_nodes = 25;       // V candidates per S node (at vmax)
  double w_nominal_velocity = 5.0;
  double w_acceleration = 1.0;
  double w_jerk = 2.0;
  double v_min = 0.0001;
  double dp_cost_max = 1e8;
  double v_ratio = 0.9;       // guide velocity = v_ub * v_ratio
  double a_ratio = 1.0;
  double v_reduction_rate = 0.5;  // vhalf = vmax * 0.5 = 0.125 m/s
  double dt_resolution = 0.001;
};

struct SVCell {
  int parent_v_ind = -1;
  double cost = 1e18;
  double v = 0.0;
  double t = 0.0;
  double s = 0.0;
  double acc = 0.0;
};

struct SVTransitionCost {
  double cost = 0.0;
  double time = 0.0;
  double acc = 0.0;
};

class DpSVPlannerECC {
public:
  DpSVPlannerECC(const DpSVConfig &config, const VehicleParam &vehicle);

  // Plan speed profile via DP in S-V space.
  // liom_path: LIOM-optimized trajectory (phi field used for speed limit estimation)
  // start:     initial state (v, phi, a, omega boundary conditions)
  // result:    output States with DP-derived speed profile, ready for ECC NLP initial guess
  bool Plan(const States &liom_path, const TrajectoryPoint &start, States &result);

private:
  // Speed limit estimation: |phi[closest_idx]| > 0.5*phimax → limit to vhalf
  void EstimateSpeedLimitFromPhi(const States &liom_path);

  // Cost function (adapted from reference DpSVGraph::GetCost)
  SVTransitionCost GetCost(int parent_v_ind, int s_ind, int v_ind, double v_guide);

  // Fine-grained interpolation at dt_resolution, truncate at tf, downsample to nfe points
  void GenerateIG(const std::vector<double> &speed_s, const std::vector<double> &speed_v,
                  const std::vector<double> &speed_t, const States &liom_path,
                  const TrajectoryPoint &start, States &result);

  int FindClosestIndex(const std::vector<double> &vec, double value);

  DpSVConfig config_;
  VehicleParam vehicle_;

  double acc_max_, acc_min_;
  double start_v_;

  std::vector<double> s_list_;                          // DP S grid (NS+1 nodes)
  std::vector<double> v_ub_list_;                       // speed upper bound per S node
  std::vector<std::vector<double>> velocity_vec_;        // velocity candidates per S node
  std::vector<std::vector<SVCell>> states_;              // DP table (NS cells)

  std::vector<double> s_list_from_path_;                // cumulative arc length of liom_path
  double s_resolution_;
};

}  // namespace trajectory_nlp
