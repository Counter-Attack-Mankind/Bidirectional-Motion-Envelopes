#pragma once

#include "dp_sv_planner_ecc.h"
#include "trajectory_nlp.h"
#include "trajectory_nlp_config.h"

#include "common/math/aabox2d.h"
#include "common/math/polygon2d.h"

namespace trajectory_nlp {

using common::math::Polygon2d;
using common::math::AABox2d;
using common::math::Box2d;

struct CoarseDecision {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> theta;
  std::vector<int> gears;  // 1 = forward, -1 = reverse (from Hybrid A* velocity signs)
};


class TrajectoryOptimizer {
public:
  TrajectoryOptimizer(const TrajectoryNLPConfig &config, const DpSVConfig &dp_sv_config, Env env);

  bool Optimize(const CoarseDecision &decision, const TrajectoryPoint &start, const TrajectoryPoint &goal, States &result);

  // ECC 版本：基于曲率限速的 DP 配速 + 一次性 NLP（pair1/pair2 条件分支约束）
  bool OptimizeECC(const States &liom_path, const TrajectoryPoint &start, const TrajectoryPoint &goal, States &result);

private:
  TrajectoryNLPConfig config_;
  DpSVConfig dp_sv_config_;
  Env env_;
  VehicleParam vehicle_;
  TrajectoryNLP nlp_;

  // 1.
  States ResampleCoarsePath(CoarseDecision &coarse);

  // ECC 专用：基于曲率速度限制的重采样（前向-反向速度规划）
  States ResampleCoarsePathECC(const CoarseDecision &coarse) const;

  // for 1.
  std::vector<double> GenerateOptimalTimeProfile(const CoarseDecision &coarse);

  // ECC 专用：计算满足转角-车速约束的时间序列（前向-反向通行）
  std::vector<double> GenerateTimeProfileWithCurvatureLimit(const CoarseDecision &coarse) const;

  std::vector<double> GenerateOptimalTimeProfileSegment(int gear, const std::vector<double> &stations, double start_time) const;

  // 2.
  void CalculateInitialGuess(States &states) const;

  // 3.
  bool FormulateCorridorConstraints(const States &states, Constraints &constraints, bool log_to_csv = false);

  bool GenerateAABox(double &x, double &y, double radius, AABox2d &box) const;

  // Helper function to compute L2 norm difference between two solution vectors
  double ComputeSolutionNormDifference(const States &states1, const States &states2) const;

};

}
