#pragma once

struct TrajectoryNLPConfig {
  int nfe = 101;
  double tf_max = 30.0;

  /**
   * penalty factor to |omega|_2 cost
   */
  double opti_omega = 5.0;

  /**
   * penalty factor to |a|_2 cost
   */
  double opti_a = 1.0;

  /**
   * penalty factor to |phi|_2 cost (steering angle)
   */
  double opti_phi = 1.0;

  /**
   * penalty factor for reference trajectory tracking
   * This prevents the optimizer from deviating too much from Hybrid A* result
   */
  double opti_ref_x = 0.5;
  double opti_ref_y = 0.5;
  double opti_ref_theta = 0.1;
  double opti_ref_v = 0.5;

  /**
   * Maximum iteration number in LIOM
   */
  int opti_iter_max = 100;

  /**
   * Initial value of weighting parameter w_penalty
   */
  double opti_w_penalty0 = 1e3;

  /**
   * Multiplier to enlarge w_penalty during the iterations
   * Reduced from 10 to 3 for smoother convergence
   */
  double opti_alpha = 3.0;

  /**
   * Maximum penalty weight to prevent ill-conditioning
   * Default: 1e7
   */
  double opti_w_penalty_max = 1e7;

  /**
   * Violation tolerance w.r.t. the softened nonlinear constraints
   */
  double opti_varepsilon_tol = 1e-6;

  /**
   * Solution vector norm difference tolerance for convergence
   * Checks if consecutive iterations produce similar solutions
   */
  double opti_solution_norm_tol = 1e-1;

  int iterative_direct_iterations = 10000;
  int iterative_first_iterations = 4000;
  int iterative_rest_iterations = 10000;

  bool use_iterative = true;

  int corridor_max_iter = 1000;
  double corridor_incremental_limit = 20.0;

  // ========== ECC (条件分支约束) 参数 ==========
  // 来自 NLP4.mod 的 pair1/pair2 约束参数
  // 条件: 当 |phi| > phimax/2 时，v 必须 <= ecc_vhalf
  double ecc_vhalf = 0.125;       // vmax/2 = 0.25/2，转角大时的速度上限
  double ecc_NATAN = 10000.0;     // smooth step 斜率（越大越近似阶跃函数）
  double ecc_NF = 1.0;            // 额外缩放因子
  double ecc_epsl = 0.10;         // 可行性缓冲量（pair1 的 +epsl 项）
  double ecc_epsl_sq = 0.01;      // pair1/pair2 中的阈值偏移（防止 atan 在中点饱和）
  double ecc_NG = 10.0;           // pair1 的增益（速度约束的约束力度）

  // ECC objective smoothing weights
  double opti_smooth_a = 5.0;
  double opti_smooth_omega = 5.0;
};
