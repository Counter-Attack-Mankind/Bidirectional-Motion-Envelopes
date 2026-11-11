#include "optimizer/footprint_nlp.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <casadi/casadi.hpp>

#include "common/math/math_utils.h"
#include "optimizer/embodied_footprint.h"

namespace optimizer {
namespace {

struct HalfspaceObstacle {
  std::array<std::array<double, 2>, 4> a{};
  std::array<double, 4> b{};
  common::math::Vec2d center;
};

common::math::Vec2d ClosestPointOnSegment(
    const common::math::Vec2d& point, const common::math::Vec2d& first,
    const common::math::Vec2d& second) {
  const double dx = second.x() - first.x();
  const double dy = second.y() - first.y();
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1e-20) return first;
  const double ratio = std::clamp(
      ((point.x() - first.x()) * dx + (point.y() - first.y()) * dy) /
          length_squared,
      0.0, 1.0);
  return {first.x() + ratio * dx, first.y() + ratio * dy};
}

double ClosestPolygonPoints(
    const std::array<common::math::Vec2d, 4>& body,
    const common::math::Polygon2d& obstacle,
    common::math::Vec2d* body_point, common::math::Vec2d* obstacle_point) {
  double best = std::numeric_limits<double>::infinity();
  const auto& points = obstacle.points();
  auto consider = [&](const common::math::Vec2d& first,
                      const common::math::Vec2d& second) {
    const double distance = first.DistanceTo(second);
    if (distance < best) {
      best = distance;
      *body_point = first;
      *obstacle_point = second;
    }
  };
  for (const auto& corner : body) {
    for (std::size_t edge = 0; edge < points.size(); ++edge) {
      consider(corner, ClosestPointOnSegment(
                           corner, points[edge], points[(edge + 1) % points.size()]));
    }
  }
  for (const auto& point : points) {
    for (std::size_t edge = 0; edge < body.size(); ++edge) {
      const auto closest = ClosestPointOnSegment(
          point, body[edge], body[(edge + 1) % body.size()]);
      consider(closest, point);
    }
  }
  return best;
}

std::array<double, 4> RepresentDirection(
    const HalfspaceObstacle& obstacle, const common::math::Vec2d& boundary_point,
    double qx, double qy) {
  std::array<double, 4> best{};
  double best_score = std::numeric_limits<double>::infinity();
  auto score = [&](const std::array<double, 4>& candidate) {
    double represented_x = 0.0, represented_y = 0.0, support = 0.0;
    for (std::size_t edge = 0; edge < 4; ++edge) {
      represented_x += obstacle.a[edge][0] * candidate[edge];
      represented_y += obstacle.a[edge][1] * candidate[edge];
      support += obstacle.b[edge] * candidate[edge];
    }
    const double residual = std::hypot(represented_x - qx,
                                       represented_y - qy);
    const double support_error = std::abs(
        support - qx * boundary_point.x() - qy * boundary_point.y());
    const double current_score = support_error + 1000.0 * residual;
    if (current_score < best_score) {
      best_score = current_score;
      best = candidate;
    }
  };
  for (std::size_t edge = 0; edge < 4; ++edge) {
    std::array<double, 4> candidate{};
    candidate[edge] = std::max(
        0.0, obstacle.a[edge][0] * qx + obstacle.a[edge][1] * qy);
    score(candidate);
  }
  for (std::size_t first = 0; first < 4; ++first) {
    for (std::size_t second = first + 1; second < 4; ++second) {
      const double determinant =
          obstacle.a[first][0] * obstacle.a[second][1] -
          obstacle.a[second][0] * obstacle.a[first][1];
      if (std::abs(determinant) <= 1e-12) continue;
      const double first_coefficient =
          (qx * obstacle.a[second][1] - qy * obstacle.a[second][0]) /
          determinant;
      const double second_coefficient =
          (obstacle.a[first][0] * qy - obstacle.a[first][1] * qx) /
          determinant;
      if (first_coefficient < -1e-10 || second_coefficient < -1e-10) continue;
      std::array<double, 4> candidate{};
      candidate[first] = std::max(0.0, first_coefficient);
      candidate[second] = std::max(0.0, second_coefficient);
      score(candidate);
    }
  }
  return best;
}

std::vector<HalfspaceObstacle> BuildHalfspaces(
    const std::vector<common::math::Polygon2d>& obstacles) {
  std::vector<HalfspaceObstacle> result;
  result.reserve(obstacles.size());
  for (std::size_t obstacle_index = 0;
       obstacle_index < obstacles.size(); ++obstacle_index) {
    const auto& obstacle = obstacles[obstacle_index];
    HalfspaceObstacle halfspace;
    halfspace.center = obstacle.center();
    const auto& points = obstacle.points();
    if (points.size() != 4) {
      throw std::invalid_argument(
          "Obstacle " + std::to_string(obstacle_index) +
          " must be a convex quadrilateral");
    }
    for (std::size_t edge = 0; edge < 4; ++edge) {
      const auto& first = points[edge];
      const auto& second = points[(edge + 1) % points.size()];
      double nx = second.y() - first.y();
      double ny = -(second.x() - first.x());
      const double length = std::hypot(nx, ny);
      if (length <= 1e-12) {
        throw std::invalid_argument(
            "Obstacle " + std::to_string(obstacle_index) +
            " contains a zero-length edge");
      }
      nx /= length;
      ny /= length;
      double boundary = nx * first.x() + ny * first.y();
      if (nx * obstacle.center().x() + ny * obstacle.center().y() > boundary) {
        nx = -nx;
        ny = -ny;
        boundary = -boundary;
      }
      halfspace.a[edge] = {nx, ny};
      halfspace.b[edge] = boundary;
    }
    result.push_back(halfspace);
  }
  return result;
}

std::string GenericToString(const casadi::GenericType& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

double PositiveViolation(double value) { return std::max(0.0, value); }

std::string CompactMessage(const std::string& message) {
  const std::size_t line_end = message.find_first_of("\r\n");
  std::string compact = message.substr(0, line_end);
  if (compact.size() > 300) compact.resize(300);
  return compact;
}

void RecordMaximum(double value, const std::string& location,
                   double* maximum, std::string* maximum_location) {
  if (value > *maximum) {
    *maximum = value;
    *maximum_location = location;
  }
}

void RecordMaximum(double value, const std::string& location,
                   MaximumViolation* maximum) {
  if (value > maximum->value) {
    maximum->value = value;
    maximum->location = location;
  }
}

void CopyMaximum(const MaximumViolation& source, double* value,
                 std::string* location) {
  if (source.value > *value) {
    *value = source.value;
    *location = source.location;
  }
}

}  // namespace

OptimizationResult SolveFootprintNlp(
    const Trajectory& initial, const VehicleParameter& vehicle,
    const std::vector<common::math::Polygon2d>& obstacles,
    const std::array<double, 4>& bounds,
    const FootprintNlpConfig& config,
    const std::string& ipopt_log_file) {
  OptimizationResult result;
  result.initial_guess_succeeded = true;
  result.configured_max_iterations = config.max_iterations;
  result.configured_max_cpu_time = config.max_cpu_time;
  result.configured_tolerance = config.tolerance;
  result.configured_linear_solver = config.linear_solver;
  result.configured_hessian_approximation = "exact";
  result.configured_mu_strategy = "adaptive";
  if (initial.nodes.size() < 2 ||
      initial.intervals.size() + 1 != initial.nodes.size()) {
    result.status = "Invalid initial trajectory dimensions";
    return result;
  }
  std::string geometry_error;
  if (!CheckVehicleGeometryCondition(vehicle, &geometry_error)) {
    result.status = geometry_error;
    return result;
  }

  std::string failure_stage = "NLP model construction";
  const auto model_begin = std::chrono::steady_clock::now();
  auto solve_begin = model_begin;
  try {

  using casadi::DM;
  using casadi::MX;
  const std::size_t node_count = initial.nodes.size();
  const std::size_t interval_count = node_count - 1;
  const std::size_t obstacle_count = obstacles.size();
  const std::size_t obca_column_count = interval_count * obstacle_count;
  result.model_obstacle_count = obstacle_count;
  result.model_obca_variable_count = 10 * obca_column_count;
  result.model_variable_count = 7 * node_count + 1 + 17 * interval_count +
                                result.model_obca_variable_count;
  result.model_constraint_count = 44 * interval_count + 17 + 8 * node_count +
                                  14 * obca_column_count;
  const double front = vehicle.wheel_base + vehicle.front_hang;
  const double rear = vehicle.rear_hang;
  const double half_width = vehicle.width / 2.0;
  const auto halfspaces = BuildHalfspaces(obstacles);

  casadi::Opti opti;
  MX state = opti.variable(7, node_count);
  MX dt = opti.variable(interval_count);
  MX tf = opti.variable();
  MX k = opti.variable(interval_count);
  MX s = opti.variable(interval_count);
  MX splus = opti.variable(interval_count);
  MX sminus = opti.variable(interval_count);
  MX up = opti.variable(interval_count);
  MX down = opti.variable(interval_count);
  MX left = opti.variable(interval_count);
  MX right = opti.variable(interval_count);
  MX AX = opti.variable(interval_count);
  MX AY = opti.variable(interval_count);
  MX BX = opti.variable(interval_count);
  MX BY = opti.variable(interval_count);
  MX CX = opti.variable(interval_count);
  MX CY = opti.variable(interval_count);
  MX DX = opti.variable(interval_count);
  MX DY = opti.variable(interval_count);
  MX lambda;
  MX mu;
  MX obca_qx;
  MX obca_qy;
  if (obstacle_count > 0) {
    lambda = opti.variable(4, obca_column_count);
    mu = opti.variable(4, obca_column_count);
    obca_qx = opti.variable(obca_column_count);
    obca_qy = opti.variable(obca_column_count);
  }

  for (std::size_t i = 0; i < interval_count; ++i) {
    opti.subject_to(dt(i) >= config.min_dt);
    opti.subject_to(dt(i) <= config.max_dt);
  }
  opti.subject_to(tf >= 0.0);
  opti.subject_to(tf == MX::sum1(dt));
  opti.subject_to(tf <= config.total_time_limit);
  for (std::size_t column = 0; column < obca_column_count; ++column) {
    for (std::size_t edge = 0; edge < 4; ++edge) {
      opti.subject_to(lambda(edge, column) >= 0.0);
      opti.subject_to(mu(edge, column) >= 0.0);
    }
  }

  MX objective = MX::sumsqr(dt);
  opti.minimize(objective);

  opti.subject_to(state(0, 0) == initial.nodes.front().x);
  opti.subject_to(state(1, 0) == initial.nodes.front().y);
  opti.subject_to(state(2, 0) == initial.nodes.front().theta);
  opti.subject_to(state(3, 0) == 0.0);
  opti.subject_to(state(4, 0) == 0.0);
  opti.subject_to(state(5, 0) == 0.0);
  opti.subject_to(state(6, 0) == 0.0);
  const std::size_t last = node_count - 1;
  opti.subject_to(state(0, last) == initial.nodes.back().x);
  opti.subject_to(state(1, last) == initial.nodes.back().y);
  opti.subject_to(state(2, last) == initial.nodes.back().theta);
  opti.subject_to(state(3, last) == 0.0);
  opti.subject_to(state(4, last) == 0.0);
  opti.subject_to(state(5, last) == 0.0);
  opti.subject_to(state(6, last) == 0.0);

  for (std::size_t i = 0; i < node_count; ++i) {
    opti.subject_to(state(3, i) <= vehicle.v_max);
    opti.subject_to(state(3, i) >= -vehicle.v_max);
    opti.subject_to(state(4, i) <= vehicle.a_max);
    opti.subject_to(state(4, i) >= -vehicle.a_max);
    opti.subject_to(state(5, i) <= vehicle.delta_max);
    opti.subject_to(state(5, i) >= -vehicle.delta_max);
    opti.subject_to(state(6, i) <= vehicle.omega_max);
    opti.subject_to(state(6, i) >= -vehicle.omega_max);
  }

  for (std::size_t i = 0; i < interval_count; ++i) {
    const MX x = state(0, i);
    const MX y = state(1, i);
    const MX theta = state(2, i);
    const MX v = state(3, i);
    const MX a = state(4, i);
    const MX phy = state(5, i);
    const MX w = state(6, i);
    const MX interval_dt = dt(i);
    opti.subject_to(state(0, i + 1) == x + v * interval_dt * cos(theta));
    opti.subject_to(state(1, i + 1) == y + v * interval_dt * sin(theta));
    opti.subject_to(state(3, i + 1) == v + interval_dt * a);
    opti.subject_to(state(2, i + 1) ==
                    theta + interval_dt * v * tan(phy) / vehicle.wheel_base);
    opti.subject_to(state(5, i + 1) == phy + interval_dt * w);

    const MX abs_k = fabs(k(i));
    opti.subject_to(k(i) == tan(phy) / vehicle.wheel_base);
    opti.subject_to(s(i) == v * interval_dt);
    opti.subject_to(splus(i) == fmax(s(i), 0.0));
    opti.subject_to(sminus(i) == fmax(-s(i), 0.0));
    opti.subject_to(up(i) == splus(i) + half_width * abs_k * splus(i));
    opti.subject_to(down(i) == sminus(i) + half_width * abs_k * sminus(i));
    opti.subject_to(
        left(i) ==
        fmax(-rear * k(i) * splus(i),
             (front + 0.5 * splus(i)) * k(i) * splus(i)) +
        fmax(-front * k(i) * sminus(i),
             (rear + 0.5 * sminus(i)) * k(i) * sminus(i)));
    opti.subject_to(
        right(i) ==
        fmax(rear * k(i) * splus(i),
             -(front + 0.5 * splus(i)) * k(i) * splus(i)) +
        fmax(front * k(i) * sminus(i),
             -(rear + 0.5 * sminus(i)) * k(i) * sminus(i)));

    opti.subject_to(abs_k * (splus(i) + sminus(i)) <= M_PI_2);
    opti.subject_to((1.0 + half_width * abs_k) * tan(abs_k * splus(i)) <=
                    rear * abs_k);
    opti.subject_to(abs_k * front * tan(abs_k * splus(i)) <=
                    1.0 + half_width * abs_k);
    opti.subject_to((1.0 + half_width * abs_k) * tan(abs_k * sminus(i)) <=
                    front * abs_k);
    opti.subject_to(abs_k * rear * tan(abs_k * sminus(i)) <=
                    1.0 + half_width * abs_k);

    const MX c = cos(theta);
    const MX sn = sin(theta);
    opti.subject_to(AX(i) == x + (front + up(i)) * c -
                                     (half_width + left(i)) * sn);
    opti.subject_to(BX(i) == x + (front + up(i)) * c +
                                     (half_width + right(i)) * sn);
    opti.subject_to(CX(i) == x - (rear + down(i)) * c +
                                     (half_width + right(i)) * sn);
    opti.subject_to(DX(i) == x - (rear + down(i)) * c -
                                     (half_width + left(i)) * sn);
    opti.subject_to(AY(i) == y + (front + up(i)) * sn +
                                     (half_width + left(i)) * c);
    opti.subject_to(BY(i) == y + (front + up(i)) * sn -
                                     (half_width + right(i)) * c);
    opti.subject_to(CY(i) == y - (rear + down(i)) * sn -
                                     (half_width + right(i)) * c);
    opti.subject_to(DY(i) == y - (rear + down(i)) * sn +
                                     (half_width + left(i)) * c);
    std::array<MX, 4> corner_x{AX(i), BX(i), CX(i), DX(i)};
    std::array<MX, 4> corner_y{AY(i), BY(i), CY(i), DY(i)};
    for (std::size_t corner = 0; corner < 4; ++corner) {
      opti.subject_to(corner_x[corner] >= bounds[0]);
      opti.subject_to(corner_x[corner] <= bounds[1]);
      opti.subject_to(corner_y[corner] >= bounds[2]);
      opti.subject_to(corner_y[corner] <= bounds[3]);
    }

    for (std::size_t obstacle = 0; obstacle < obstacle_count; ++obstacle) {
      const std::size_t column = i * obstacle_count + obstacle;
      MX qx_definition = 0.0;
      MX qy_definition = 0.0;
      MX certificate = 0.0;
      for (std::size_t edge = 0; edge < 4; ++edge) {
        qx_definition +=
            halfspaces[obstacle].a[edge][0] * lambda(edge, column);
        qy_definition +=
            halfspaces[obstacle].a[edge][1] * lambda(edge, column);
        certificate +=
            (halfspaces[obstacle].a[edge][0] * x +
             halfspaces[obstacle].a[edge][1] * y -
             halfspaces[obstacle].b[edge]) * lambda(edge, column);
      }
      opti.subject_to(obca_qx(column) == qx_definition);
      opti.subject_to(obca_qy(column) == qy_definition);
      certificate -= (front + up(i)) * mu(0, column) +
                     (rear + down(i)) * mu(1, column) +
                     (half_width + left(i)) * mu(2, column) +
                     (half_width + right(i)) * mu(3, column);
      opti.subject_to(certificate >= config.dmin);
      opti.subject_to(mu(0, column) - mu(1, column) +
                      c * obca_qx(column) + sn * obca_qy(column) == 0.0);
      opti.subject_to(mu(2, column) - mu(3, column) -
                      sn * obca_qx(column) + c * obca_qy(column) == 0.0);
      opti.subject_to(obca_qx(column) * obca_qx(column) +
                      obca_qy(column) * obca_qy(column) <= 1.0);
    }
  }

  DM state_initial = DM::zeros(7, node_count);
  DM dt_initial = DM::zeros(interval_count);
  for (std::size_t i = 0; i < node_count; ++i) {
    state_initial(0, i) = initial.nodes[i].x;
    state_initial(1, i) = initial.nodes[i].y;
    state_initial(2, i) = initial.nodes[i].theta;
    state_initial(3, i) = initial.nodes[i].v;
    state_initial(4, i) = initial.nodes[i].a;
    state_initial(5, i) = initial.nodes[i].phy;
    state_initial(6, i) = initial.nodes[i].w;
    if (i < interval_count) dt_initial(i) = initial.intervals[i].dt;
  }
  opti.set_initial(state, state_initial);
  opti.set_initial(dt, dt_initial);
  DM k_initial = DM::zeros(interval_count);
  DM s_initial = DM::zeros(interval_count);
  DM splus_initial = DM::zeros(interval_count);
  DM sminus_initial = DM::zeros(interval_count);
  DM up_initial = DM::zeros(interval_count);
  DM down_initial = DM::zeros(interval_count);
  DM left_initial = DM::zeros(interval_count);
  DM right_initial = DM::zeros(interval_count);
  DM AX_initial = DM::zeros(interval_count);
  DM AY_initial = DM::zeros(interval_count);
  DM BX_initial = DM::zeros(interval_count);
  DM BY_initial = DM::zeros(interval_count);
  DM CX_initial = DM::zeros(interval_count);
  DM CY_initial = DM::zeros(interval_count);
  DM DX_initial = DM::zeros(interval_count);
  DM DY_initial = DM::zeros(interval_count);
  double tf_initial = 0.0;
  for (std::size_t i = 0; i < interval_count; ++i) {
    const auto& node = initial.nodes[i];
    const double interval_dt = initial.intervals[i].dt;
    const double initial_k = std::tan(node.phy) / vehicle.wheel_base;
    const double abs_k = std::abs(initial_k);
    const double initial_s = node.v * interval_dt;
    const double initial_splus = std::max(initial_s, 0.0);
    const double initial_sminus = std::max(-initial_s, 0.0);
    const double initial_up =
        initial_splus + half_width * abs_k * initial_splus;
    const double initial_down =
        initial_sminus + half_width * abs_k * initial_sminus;
    const double initial_left =
        std::max(-rear * initial_k * initial_splus,
                 (front + 0.5 * initial_splus) * initial_k * initial_splus) +
        std::max(-front * initial_k * initial_sminus,
                 (rear + 0.5 * initial_sminus) * initial_k * initial_sminus);
    const double initial_right =
        std::max(rear * initial_k * initial_splus,
                 -(front + 0.5 * initial_splus) * initial_k * initial_splus) +
        std::max(front * initial_k * initial_sminus,
                 -(rear + 0.5 * initial_sminus) * initial_k * initial_sminus);
    const double c = std::cos(node.theta);
    const double sn = std::sin(node.theta);
    k_initial(i) = initial_k;
    s_initial(i) = initial_s;
    splus_initial(i) = initial_splus;
    sminus_initial(i) = initial_sminus;
    up_initial(i) = initial_up;
    down_initial(i) = initial_down;
    left_initial(i) = initial_left;
    right_initial(i) = initial_right;
    AX_initial(i) = node.x + (front + initial_up) * c -
                    (half_width + initial_left) * sn;
    BX_initial(i) = node.x + (front + initial_up) * c +
                    (half_width + initial_right) * sn;
    CX_initial(i) = node.x - (rear + initial_down) * c +
                    (half_width + initial_right) * sn;
    DX_initial(i) = node.x - (rear + initial_down) * c -
                    (half_width + initial_left) * sn;
    AY_initial(i) = node.y + (front + initial_up) * sn +
                    (half_width + initial_left) * c;
    BY_initial(i) = node.y + (front + initial_up) * sn -
                    (half_width + initial_right) * c;
    CY_initial(i) = node.y - (rear + initial_down) * sn -
                    (half_width + initial_right) * c;
    DY_initial(i) = node.y - (rear + initial_down) * sn +
                    (half_width + initial_left) * c;
    tf_initial += interval_dt;
  }
  opti.set_initial(tf, tf_initial);
  opti.set_initial(k, k_initial);
  opti.set_initial(s, s_initial);
  opti.set_initial(splus, splus_initial);
  opti.set_initial(sminus, sminus_initial);
  opti.set_initial(up, up_initial);
  opti.set_initial(down, down_initial);
  opti.set_initial(left, left_initial);
  opti.set_initial(right, right_initial);
  opti.set_initial(AX, AX_initial);
  opti.set_initial(AY, AY_initial);
  opti.set_initial(BX, BX_initial);
  opti.set_initial(BY, BY_initial);
  opti.set_initial(CX, CX_initial);
  opti.set_initial(CY, CY_initial);
  opti.set_initial(DX, DX_initial);
  opti.set_initial(DY, DY_initial);

  DM lambda_initial;
  DM mu_initial;
  DM obca_qx_initial;
  DM obca_qy_initial;
  if (obstacle_count > 0) {
    lambda_initial = DM::zeros(4, obca_column_count);
    mu_initial = DM::zeros(4, obca_column_count);
    obca_qx_initial = DM::zeros(obca_column_count);
    obca_qy_initial = DM::zeros(obca_column_count);
    for (std::size_t i = 0; i < interval_count; ++i) {
      const double c = std::cos(initial.nodes[i].theta);
      const double s = std::sin(initial.nodes[i].theta);
      const auto initial_footprint = ComputeEmbodiedFootprint(
          initial.nodes[i], initial.intervals[i].dt, vehicle);
      for (std::size_t obstacle = 0; obstacle < obstacle_count; ++obstacle) {
        const std::size_t column = i * obstacle_count + obstacle;
        common::math::Vec2d body_point;
        common::math::Vec2d obstacle_point;
        const double distance = ClosestPolygonPoints(
            initial_footprint.corners, obstacles[obstacle],
            &body_point, &obstacle_point);
        double qx = body_point.x() - obstacle_point.x();
        double qy = body_point.y() - obstacle_point.y();
        const double norm = std::hypot(qx, qy);
        if (norm > 1e-9) { qx /= norm; qy /= norm; } else { qx = 1.0; qy = 0.0; }
        auto lambda_values = RepresentDirection(
            halfspaces[obstacle], obstacle_point, qx, qy);
        const double longitudinal = c * qx + s * qy;
        const double lateral = -s * qx + c * qy;
        std::array<double, 4> mu_values{{
            std::max(-longitudinal, 0.0), std::max(longitudinal, 0.0),
            std::max(-lateral, 0.0), std::max(lateral, 0.0)}};
        const double rho = distance > 0.0
                               ? std::min(1.0, std::max(
                                     0.25, (config.dmin + 1e-3) / distance))
                               : 1.0;
        for (std::size_t edge = 0; edge < 4; ++edge) {
          lambda_initial(edge, column) = rho * lambda_values[edge];
          mu_initial(edge, column) = rho * mu_values[edge];
          obca_qx_initial(column) +=
              halfspaces[obstacle].a[edge][0] * rho * lambda_values[edge];
          obca_qy_initial(column) +=
              halfspaces[obstacle].a[edge][1] * rho * lambda_values[edge];
        }
      }
    }
    opti.set_initial(lambda, lambda_initial);
    opti.set_initial(mu, mu_initial);
    opti.set_initial(obca_qx, obca_qx_initial);
    opti.set_initial(obca_qy, obca_qy_initial);
  }

  auto extract_snapshot = [&](const auto& values,
                              NlpIterateSnapshot* snapshot) {
    const DM state_value = values.value(state);
    const DM dt_value = values.value(dt);
    snapshot->trajectory.nodes.resize(node_count);
    snapshot->trajectory.intervals.resize(interval_count);
    for (std::size_t i = 0; i < node_count; ++i) {
      auto& node = snapshot->trajectory.nodes[i];
      node.x = static_cast<double>(state_value(0, i));
      node.y = static_cast<double>(state_value(1, i));
      node.theta = static_cast<double>(state_value(2, i));
      node.v = static_cast<double>(state_value(3, i));
      node.a = static_cast<double>(state_value(4, i));
      node.phy = static_cast<double>(state_value(5, i));
      node.w = static_cast<double>(state_value(6, i));
    }
    snapshot->k.resize(interval_count);
    snapshot->s.resize(interval_count);
    snapshot->splus.resize(interval_count);
    snapshot->sminus.resize(interval_count);
    snapshot->up.resize(interval_count);
    snapshot->down.resize(interval_count);
    snapshot->left.resize(interval_count);
    snapshot->right.resize(interval_count);
    snapshot->corners.resize(interval_count);
    const DM k_value = values.value(k);
    const DM s_value = values.value(s);
    const DM splus_value = values.value(splus);
    const DM sminus_value = values.value(sminus);
    const DM up_value = values.value(up);
    const DM down_value = values.value(down);
    const DM left_value = values.value(left);
    const DM right_value = values.value(right);
    const DM ax_value = values.value(AX);
    const DM ay_value = values.value(AY);
    const DM bx_value = values.value(BX);
    const DM by_value = values.value(BY);
    const DM cx_value = values.value(CX);
    const DM cy_value = values.value(CY);
    const DM dx_value = values.value(DX);
    const DM dy_value = values.value(DY);
    for (std::size_t i = 0; i < interval_count; ++i) {
      const double interval_dt = static_cast<double>(dt_value(i));
      auto footprint = ComputeEmbodiedFootprint(
          snapshot->trajectory.nodes[i], interval_dt, vehicle);
      footprint.gear = initial.intervals[i].gear;
      snapshot->trajectory.intervals[i] = footprint;
      snapshot->k[i] = static_cast<double>(k_value(i));
      snapshot->s[i] = static_cast<double>(s_value(i));
      snapshot->splus[i] = static_cast<double>(splus_value(i));
      snapshot->sminus[i] = static_cast<double>(sminus_value(i));
      snapshot->up[i] = static_cast<double>(up_value(i));
      snapshot->down[i] = static_cast<double>(down_value(i));
      snapshot->left[i] = static_cast<double>(left_value(i));
      snapshot->right[i] = static_cast<double>(right_value(i));
      snapshot->corners[i] = {{
          static_cast<double>(ax_value(i)), static_cast<double>(ay_value(i)),
          static_cast<double>(bx_value(i)), static_cast<double>(by_value(i)),
          static_cast<double>(cx_value(i)), static_cast<double>(cy_value(i)),
          static_cast<double>(dx_value(i)), static_cast<double>(dy_value(i))}};
    }
    snapshot->total_time = static_cast<double>(values.value(tf));
    if (obstacle_count > 0) {
      const DM lambda_value = values.value(lambda);
      const DM mu_value = values.value(mu);
      const DM qx_value = values.value(obca_qx);
      const DM qy_value = values.value(obca_qy);
      snapshot->lambda.resize(obca_column_count);
      snapshot->mu.resize(obca_column_count);
      snapshot->obca_qx.resize(obca_column_count);
      snapshot->obca_qy.resize(obca_column_count);
      for (std::size_t column = 0; column < obca_column_count; ++column) {
        for (std::size_t edge = 0; edge < 4; ++edge) {
          snapshot->lambda[column][edge] =
              static_cast<double>(lambda_value(edge, column));
          snapshot->mu[column][edge] =
              static_cast<double>(mu_value(edge, column));
        }
        snapshot->obca_qx[column] = static_cast<double>(qx_value(column));
        snapshot->obca_qy[column] = static_cast<double>(qy_value(column));
      }
    }
    snapshot->available = true;
  };

  auto evaluate_snapshot = [&](const NlpIterateSnapshot& snapshot) {
    auto& diagnostics = result.diagnostics;
    double dt_sum = 0.0;
    for (std::size_t i = 0; i < node_count; ++i) {
      const auto& node = snapshot.trajectory.nodes[i];
      const std::string location = "node=" + std::to_string(i);
      RecordMaximum(PositiveViolation(std::abs(node.v) - vehicle.v_max),
                    location, &diagnostics.velocity_bound_mps);
      RecordMaximum(PositiveViolation(std::abs(node.a) - vehicle.a_max),
                    location, &diagnostics.acceleration_bound_mps2);
      RecordMaximum(PositiveViolation(std::abs(node.phy) - vehicle.delta_max),
                    location, &diagnostics.steering_bound_rad);
      RecordMaximum(PositiveViolation(std::abs(node.w) - vehicle.omega_max),
                    location, &diagnostics.steering_rate_bound_radps);
    }
    auto record_endpoint = [&](const TrajectoryNode& actual,
                               const TrajectoryNode& expected,
                               const std::string& endpoint) {
      RecordMaximum(std::abs(actual.x - expected.x), endpoint,
                    &diagnostics.endpoint_x_m);
      RecordMaximum(std::abs(actual.y - expected.y), endpoint,
                    &diagnostics.endpoint_y_m);
      RecordMaximum(std::abs(actual.theta - expected.theta), endpoint,
                    &diagnostics.endpoint_theta_rad);
      RecordMaximum(std::abs(actual.v), endpoint,
                    &diagnostics.endpoint_v_mps);
      RecordMaximum(std::abs(actual.a), endpoint,
                    &diagnostics.endpoint_acceleration_mps2);
      RecordMaximum(std::abs(actual.phy), endpoint,
                    &diagnostics.endpoint_phi_rad);
      RecordMaximum(std::abs(actual.w), endpoint,
                    &diagnostics.endpoint_steering_rate_radps);
    };
    record_endpoint(snapshot.trajectory.nodes.front(), initial.nodes.front(),
                    "start");
    record_endpoint(snapshot.trajectory.nodes.back(), initial.nodes.back(),
                    "goal");

    for (std::size_t i = 0; i < interval_count; ++i) {
      const auto& first = snapshot.trajectory.nodes[i];
      const auto& second = snapshot.trajectory.nodes[i + 1];
      const double interval_dt = snapshot.trajectory.intervals[i].dt;
      const auto& computed_footprint = snapshot.trajectory.intervals[i];
      const std::string interval = "interval=" + std::to_string(i);
      dt_sum += interval_dt;
      RecordMaximum(std::max(PositiveViolation(config.min_dt - interval_dt),
                             PositiveViolation(interval_dt - config.max_dt)),
                    interval, &diagnostics.time_step_bound_s);
      RecordMaximum(std::abs(second.x - first.x -
                             first.v * interval_dt * std::cos(first.theta)),
                    interval, &diagnostics.dynamics_x_m);
      RecordMaximum(std::abs(second.y - first.y -
                             first.v * interval_dt * std::sin(first.theta)),
                    interval, &diagnostics.dynamics_y_m);
      RecordMaximum(std::abs(second.theta - first.theta - interval_dt * first.v *
                             std::tan(first.phy) / vehicle.wheel_base),
                    interval, &diagnostics.dynamics_theta_rad);
      RecordMaximum(std::abs(second.v - first.v - interval_dt * first.a),
                    interval, &diagnostics.dynamics_v_mps);
      RecordMaximum(std::abs(second.phy - first.phy - interval_dt * first.w),
                    interval, &diagnostics.dynamics_phi_rad);
      const auto legacy_validity =
          EvaluateFootprintValidity(computed_footprint, vehicle);
      RecordMaximum(legacy_validity.max_violation(), interval,
                    &result.max_footprint_validity_violation,
                    &result.max_footprint_validity_location);
      for (std::size_t corner = 0; corner < 4; ++corner) {
        const auto& point = computed_footprint.corners[corner];
        RecordMaximum(std::max({PositiveViolation(bounds[0] - point.x()),
                                PositiveViolation(point.x() - bounds[1]),
                                PositiveViolation(bounds[2] - point.y()),
                                PositiveViolation(point.y() - bounds[3])}),
                      interval + ",corner=" + std::to_string(corner),
                      &result.max_map_boundary_violation,
                      &result.max_map_boundary_location);
      }

      const double computed_k = std::tan(first.phy) / vehicle.wheel_base;
      const double computed_s = first.v * interval_dt;
      const double computed_splus = std::max(computed_s, 0.0);
      const double computed_sminus = std::max(-computed_s, 0.0);
      const double abs_k = std::abs(snapshot.k[i]);
      const double computed_up = snapshot.splus[i] +
                                 half_width * abs_k * snapshot.splus[i];
      const double computed_down = snapshot.sminus[i] +
                                   half_width * abs_k * snapshot.sminus[i];
      const double computed_left =
          std::max(-rear * snapshot.k[i] * snapshot.splus[i],
                   (front + 0.5 * snapshot.splus[i]) * snapshot.k[i] *
                       snapshot.splus[i]) +
          std::max(-front * snapshot.k[i] * snapshot.sminus[i],
                   (rear + 0.5 * snapshot.sminus[i]) * snapshot.k[i] *
                       snapshot.sminus[i]);
      const double computed_right =
          std::max(rear * snapshot.k[i] * snapshot.splus[i],
                   -(front + 0.5 * snapshot.splus[i]) * snapshot.k[i] *
                       snapshot.splus[i]) +
          std::max(front * snapshot.k[i] * snapshot.sminus[i],
                   -(rear + 0.5 * snapshot.sminus[i]) * snapshot.k[i] *
                       snapshot.sminus[i]);
      RecordMaximum(std::abs(snapshot.k[i] - computed_k), interval,
                    &diagnostics.ef_curvature_equality_1pm);
      const std::array<std::pair<const char*, double>, 7> distance_equalities{{
          {"s", std::abs(snapshot.s[i] - computed_s)},
          {"splus", std::abs(snapshot.splus[i] - computed_splus)},
          {"sminus", std::abs(snapshot.sminus[i] - computed_sminus)},
          {"up", std::abs(snapshot.up[i] - computed_up)},
          {"down", std::abs(snapshot.down[i] - computed_down)},
          {"left", std::abs(snapshot.left[i] - computed_left)},
          {"right", std::abs(snapshot.right[i] - computed_right)}}};
      for (const auto& equality : distance_equalities) {
        RecordMaximum(equality.second, interval + ",variable=" + equality.first,
                      &diagnostics.ef_distance_equality_m);
      }
      FootprintInterval raw_footprint;
      raw_footprint.k = snapshot.k[i];
      raw_footprint.splus = snapshot.splus[i];
      raw_footprint.sminus = snapshot.sminus[i];
      raw_footprint.up = snapshot.up[i];
      raw_footprint.down = snapshot.down[i];
      raw_footprint.left = snapshot.left[i];
      raw_footprint.right = snapshot.right[i];
      const double c = std::cos(first.theta);
      const double sn = std::sin(first.theta);
      const std::array<double, 8> computed_corners{{
          first.x + (front + raw_footprint.up) * c -
              (half_width + raw_footprint.left) * sn,
          first.y + (front + raw_footprint.up) * sn +
              (half_width + raw_footprint.left) * c,
          first.x + (front + raw_footprint.up) * c +
              (half_width + raw_footprint.right) * sn,
          first.y + (front + raw_footprint.up) * sn -
              (half_width + raw_footprint.right) * c,
          first.x - (rear + raw_footprint.down) * c +
              (half_width + raw_footprint.right) * sn,
          first.y - (rear + raw_footprint.down) * sn -
              (half_width + raw_footprint.right) * c,
          first.x - (rear + raw_footprint.down) * c -
              (half_width + raw_footprint.left) * sn,
          first.y - (rear + raw_footprint.down) * sn +
              (half_width + raw_footprint.left) * c}};
      for (std::size_t coordinate = 0; coordinate < 8; ++coordinate) {
        RecordMaximum(std::abs(snapshot.corners[i][coordinate] -
                               computed_corners[coordinate]),
                      interval + ",corner_coordinate=" +
                          std::to_string(coordinate),
                      &diagnostics.ef_corner_equality_m);
      }
      const auto validity = EvaluateFootprintValidity(raw_footprint, vehicle);
      RecordMaximum(PositiveViolation(validity.arc), interval,
                    &diagnostics.ef_arc);
      RecordMaximum(PositiveViolation(validity.forward_cond1), interval,
                    &diagnostics.ef_forward_cond1);
      RecordMaximum(PositiveViolation(validity.forward_cond2), interval,
                    &diagnostics.ef_forward_cond2);
      RecordMaximum(PositiveViolation(validity.reverse_cond1), interval,
                    &diagnostics.ef_reverse_cond1);
      RecordMaximum(PositiveViolation(validity.reverse_cond2), interval,
                    &diagnostics.ef_reverse_cond2);
      for (std::size_t corner = 0; corner < 4; ++corner) {
        const double x = snapshot.corners[i][2 * corner];
        const double y = snapshot.corners[i][2 * corner + 1];
        RecordMaximum(std::max({PositiveViolation(bounds[0] - x),
                                PositiveViolation(x - bounds[1]),
                                PositiveViolation(bounds[2] - y),
                                PositiveViolation(y - bounds[3])}),
                      interval + ",corner=" + std::to_string(corner),
                      &diagnostics.map_boundary_m);
      }

      for (std::size_t obstacle = 0; obstacle < obstacle_count; ++obstacle) {
        const std::size_t column = i * obstacle_count + obstacle;
        const std::string location = interval + ",obstacle=" +
                                     std::to_string(obstacle);
        double qx_definition = 0.0;
        double qy_definition = 0.0;
        double certificate = 0.0;
        for (std::size_t edge = 0; edge < 4; ++edge) {
          const double lam = snapshot.lambda[column][edge];
          qx_definition += halfspaces[obstacle].a[edge][0] * lam;
          qy_definition += halfspaces[obstacle].a[edge][1] * lam;
          certificate += (halfspaces[obstacle].a[edge][0] * first.x +
                          halfspaces[obstacle].a[edge][1] * first.y -
                          halfspaces[obstacle].b[edge]) * lam;
          RecordMaximum(PositiveViolation(-lam),
                        location + ",lambda_edge=" + std::to_string(edge),
                        &diagnostics.obca_nonnegative);
          RecordMaximum(PositiveViolation(-snapshot.mu[column][edge]),
                        location + ",mu_edge=" + std::to_string(edge),
                        &diagnostics.obca_nonnegative);
        }
        const double qx = snapshot.obca_qx[column];
        const double qy = snapshot.obca_qy[column];
        RecordMaximum(std::max(std::abs(qx - qx_definition),
                               std::abs(qy - qy_definition)),
                      location, &diagnostics.obca_q_definition);
        certificate -= (front + snapshot.up[i]) * snapshot.mu[column][0] +
                       (rear + snapshot.down[i]) * snapshot.mu[column][1] +
                       (half_width + snapshot.left[i]) * snapshot.mu[column][2] +
                       (half_width + snapshot.right[i]) * snapshot.mu[column][3];
        RecordMaximum(PositiveViolation(config.dmin - certificate), location,
                      &diagnostics.obca_separation_m);
        const double dual_x = snapshot.mu[column][0] -
                              snapshot.mu[column][1] + c * qx + sn * qy;
        const double dual_y = snapshot.mu[column][2] -
                              snapshot.mu[column][3] - sn * qx + c * qy;
        RecordMaximum(std::max(std::abs(dual_x), std::abs(dual_y)), location,
                      &diagnostics.obca_dual_equality);
        RecordMaximum(PositiveViolation(qx * qx + qy * qy - 1.0), location,
                      &diagnostics.obca_norm);

        double legacy_certificate = 0.0;
        for (std::size_t edge = 0; edge < 4; ++edge) {
          legacy_certificate +=
              (halfspaces[obstacle].a[edge][0] * first.x +
               halfspaces[obstacle].a[edge][1] * first.y -
               halfspaces[obstacle].b[edge]) *
              snapshot.lambda[column][edge];
        }
        legacy_certificate -=
            (front + computed_footprint.up) * snapshot.mu[column][0] +
            (rear + computed_footprint.down) * snapshot.mu[column][1] +
            (half_width + computed_footprint.left) * snapshot.mu[column][2] +
            (half_width + computed_footprint.right) * snapshot.mu[column][3];
        RecordMaximum(PositiveViolation(config.dmin - legacy_certificate),
                      location + ",constraint=distance",
                      &result.max_obca_violation, &result.max_obca_location);
        const double legacy_dual_x = snapshot.mu[column][0] -
            snapshot.mu[column][1] + c * qx_definition + sn * qy_definition;
        const double legacy_dual_y = snapshot.mu[column][2] -
            snapshot.mu[column][3] - sn * qx_definition + c * qy_definition;
        RecordMaximum(std::max(std::abs(legacy_dual_x),
                               std::abs(legacy_dual_y)),
                      location + ",constraint=dual",
                      &result.max_obca_violation, &result.max_obca_location);
        RecordMaximum(PositiveViolation(qx_definition * qx_definition +
                                        qy_definition * qy_definition - 1.0),
                      location + ",constraint=norm",
                      &result.max_obca_violation, &result.max_obca_location);
      }
    }
    RecordMaximum(std::abs(snapshot.total_time - dt_sum), "tf=sum(dt)",
                  &diagnostics.total_time_equality_s);
    RecordMaximum(std::max(PositiveViolation(-snapshot.total_time),
                           PositiveViolation(snapshot.total_time -
                                             config.total_time_limit)),
                  "tf_bounds", &diagnostics.total_time_bound_s);

    const std::array<const MaximumViolation*, 5> dynamics{{
        &diagnostics.dynamics_x_m, &diagnostics.dynamics_y_m,
        &diagnostics.dynamics_theta_rad, &diagnostics.dynamics_v_mps,
        &diagnostics.dynamics_phi_rad}};
    for (const auto* item : dynamics) {
      CopyMaximum(*item, &result.max_dynamics_residual,
                  &result.max_dynamics_location);
    }
    const std::array<const MaximumViolation*, 4> bounds_diagnostics{{
        &diagnostics.velocity_bound_mps, &diagnostics.acceleration_bound_mps2,
        &diagnostics.steering_bound_rad,
        &diagnostics.steering_rate_bound_radps}};
    for (const auto* item : bounds_diagnostics) {
      CopyMaximum(*item, &result.max_variable_bound_violation,
                  &result.max_variable_bound_location);
    }
    result.dynamics_check_count = 5 * interval_count;
    result.variable_bound_check_count = 4 * node_count;
    result.footprint_validity_check_count = 5 * interval_count;
    result.obca_check_count = 4 * obca_column_count;
    result.map_boundary_check_count = 16 * interval_count;
  };

  casadi::Dict solver_options;
  solver_options["max_iter"] = config.max_iterations;
  solver_options["max_cpu_time"] = config.max_cpu_time;
  solver_options["tol"] = config.tolerance;
  solver_options["linear_solver"] = config.linear_solver;
  solver_options["mu_strategy"] = "adaptive";
  solver_options["print_level"] = 0;
  solver_options["sb"] = "yes";
  if (!ipopt_log_file.empty()) {
    solver_options["output_file"] = ipopt_log_file;
    solver_options["file_print_level"] = 5;
  }
  casadi::Dict plugin_options;
  plugin_options["expand"] = true;
  plugin_options["print_time"] = false;
  failure_stage = "IPOPT solver initialization";
  opti.solver("ipopt", plugin_options, solver_options);

  result.model_build_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - model_begin).count();
  failure_stage = "opti.solve()";
  solve_begin = std::chrono::steady_clock::now();
  auto read_stats = [&](const casadi::Dict& stats) {
    result.solver_stats_available = true;
    const auto status = stats.find("return_status");
    if (status != stats.end()) result.status = GenericToString(status->second);
    const auto iterations = stats.find("iter_count");
    if (iterations != stats.end()) {
      result.iterations = static_cast<int>(iterations->second);
    }
    const auto wall_time = stats.find("t_wall_total");
    if (wall_time != stats.end()) {
      const double stats_time = static_cast<double>(wall_time->second);
      if (std::isfinite(stats_time) && stats_time >= 0.0) {
        result.solve_time = stats_time;
        result.solve_time_from_stats = true;
      }
    }
  };
  try {
    const casadi::OptiSol solution = opti.solve();
    result.solve_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - solve_begin).count();
    read_stats(solution.stats());
    result.solver_succeeded =
        result.status.find("Solve_Succeeded") != std::string::npos ||
        result.status.find("Solved_To_Acceptable_Level") != std::string::npos;
    result.objective = static_cast<double>(solution.value(objective));
    NlpIterateSnapshot solution_snapshot;
    extract_snapshot(solution, &solution_snapshot);
    evaluate_snapshot(solution_snapshot);
    result.trajectory = solution_snapshot.trajectory;
    const double residual_limit = std::max(1e-4, 10.0 * config.tolerance);
    result.success = result.solver_succeeded &&
                     result.max_dynamics_residual <= residual_limit &&
                     result.max_variable_bound_violation <= residual_limit &&
                     result.max_footprint_validity_violation <= residual_limit &&
                     result.max_obca_violation <= residual_limit &&
                     result.max_map_boundary_violation <= residual_limit;
    if (!result.success) {
      result.last_iterate = std::move(solution_snapshot);
      result.last_iterate_available = result.last_iterate.available;
    }
  } catch (const std::exception& solve_exception) {
    result.solve_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - solve_begin).count();
    result.status = "opti.solve() failed";
    try {
      read_stats(opti.stats());
    } catch (const std::exception& stats_exception) {
      result.diagnostic_error =
          "solver stats unavailable: " + std::string(stats_exception.what());
    }
    if (result.status == "opti.solve() failed") {
      result.status += ": " + CompactMessage(solve_exception.what());
    }
    try {
      const casadi::OptiAdvanced debug = opti.debug();
      extract_snapshot(debug, &result.last_iterate);
      result.last_iterate_available = result.last_iterate.available;
      evaluate_snapshot(result.last_iterate);
    } catch (const std::exception& debug_exception) {
      result.last_iterate.extraction_error = debug_exception.what();
      if (!result.diagnostic_error.empty()) result.diagnostic_error += "; ";
      result.diagnostic_error +=
          "last iterate unavailable: " + std::string(debug_exception.what());
    } catch (...) {
      result.last_iterate.extraction_error = "unknown exception";
      if (!result.diagnostic_error.empty()) result.diagnostic_error += "; ";
      result.diagnostic_error += "last iterate unavailable: unknown exception";
    }
  } catch (...) {
    result.solve_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - solve_begin).count();
    result.status = "opti.solve() failed: unknown exception";
  }
  } catch (const std::exception& exception) {
    result.model_build_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - model_begin).count();
    result.status = failure_stage + " failed: " +
                    CompactMessage(exception.what());
  } catch (...) {
    result.model_build_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - model_begin).count();
    result.status = failure_stage + " failed: unknown exception";
  }
  return result;
}

bool SaveSuccessfulOptimizationResults(const std::string& result_directory,
                                       const OptimizationResult& result,
                                       std::string* error) {
  if (!result.success || result.trajectory.nodes.empty()) {
    if (error) *error = "Refusing to save an unsuccessful NLP trajectory";
    return false;
  }
  namespace fs = std::filesystem;
  std::error_code code;
  fs::create_directories(result_directory, code);
  if (code) {
    if (error) *error = code.message();
    return false;
  }
  auto write_trajectory = [&](const fs::path& path, const Trajectory& trajectory) {
    std::ofstream stream(path);
    if (!stream) return false;
    stream << std::setprecision(12)
           << "index,time_s,x_m,y_m,theta_rad,v_mps,a_mps2,phy_rad,w_radps,dt_s,gear\n";
    double time = 0.0;
    for (std::size_t i = 0; i < trajectory.nodes.size(); ++i) {
      const auto& node = trajectory.nodes[i];
      stream << i << ',' << time << ',' << node.x << ',' << node.y << ',' << node.theta << ','
             << node.v << ',' << node.a << ',' << node.phy << ',' << node.w;
      if (i < trajectory.intervals.size()) {
        stream << ',' << trajectory.intervals[i].dt << ','
               << (trajectory.intervals[i].gear ? 1 : -1);
      } else {
        stream << ",0,0";
      }
      stream << '\n';
      if (i < trajectory.intervals.size()) time += trajectory.intervals[i].dt;
    }
    return true;
  };
  if (!write_trajectory(fs::path(result_directory) / "optimized_trajectory.csv",
                        result.trajectory)) {
    if (error) *error = "Cannot write trajectory CSV";
    return false;
  }
  std::ofstream footprints(fs::path(result_directory) / "embodied_footprints.csv");
  footprints << std::setprecision(12)
             << "interval,gear,dt_s,s_m,k_1pm,up_m,down_m,left_m,right_m,"
                "AX_m,AY_m,BX_m,BY_m,CX_m,CY_m,DX_m,DY_m\n";
  for (std::size_t i = 0; i < result.trajectory.intervals.size(); ++i) {
    const auto& fp = result.trajectory.intervals[i];
    footprints << i << ',' << (fp.gear ? 1 : -1) << ',' << fp.dt << ','
               << fp.s << ',' << fp.k << ',' << fp.up << ',' << fp.down << ','
               << fp.left << ',' << fp.right;
    for (const auto& corner : fp.corners) {
      footprints << ',' << corner.x() << ',' << corner.y();
    }
    footprints << '\n';
  }
  return static_cast<bool>(footprints);
}

bool SaveNlpFailureDiagnostics(const std::string& debug_directory,
                               const OptimizationResult& result,
                               std::string* error) {
  namespace fs = std::filesystem;
  std::error_code code;
  fs::create_directories(debug_directory, code);
  if (code) {
    if (error) *error = code.message();
    return false;
  }
  std::ofstream report(fs::path(debug_directory) /
                       "nlp_failure_diagnostics.txt");
  if (!report) {
    if (error) *error = "Cannot create NLP failure diagnostics";
    return false;
  }
  report << std::setprecision(17) << std::boolalpha
         << "ipopt_return_status=" << result.status << '\n'
         << "solver_stats_available=" << result.solver_stats_available << '\n'
         << "solve_time_source="
         << (result.solve_time_from_stats ? "opti.stats.t_wall_total"
                                          : "measured_wall_clock") << '\n'
         << "iterations=" << result.iterations << '\n'
         << "solve_time_s=" << result.solve_time << '\n'
         << "last_iterate_available=" << result.last_iterate_available << '\n'
         << "last_iterate_is_executable=false\n"
         << "diagnostic_error=" << result.diagnostic_error << '\n';
  auto write = [&](const char* name, const MaximumViolation& violation) {
    report << name << "=" << violation.value << '\n'
           << name << "_location=" << violation.location << '\n';
  };
  const auto& d = result.diagnostics;
  write("max_dynamics_x_m", d.dynamics_x_m);
  write("max_dynamics_y_m", d.dynamics_y_m);
  write("max_dynamics_theta_rad", d.dynamics_theta_rad);
  write("max_dynamics_velocity_mps", d.dynamics_v_mps);
  write("max_dynamics_steering_rad", d.dynamics_phi_rad);
  write("max_velocity_bound_mps", d.velocity_bound_mps);
  write("max_acceleration_bound_mps2", d.acceleration_bound_mps2);
  write("max_steering_bound_rad", d.steering_bound_rad);
  write("max_steering_rate_bound_radps", d.steering_rate_bound_radps);
  write("max_time_step_bound_s", d.time_step_bound_s);
  write("max_endpoint_x_m", d.endpoint_x_m);
  write("max_endpoint_y_m", d.endpoint_y_m);
  write("max_endpoint_theta_rad", d.endpoint_theta_rad);
  write("max_endpoint_velocity_mps", d.endpoint_v_mps);
  write("max_endpoint_acceleration_mps2", d.endpoint_acceleration_mps2);
  write("max_endpoint_steering_rad", d.endpoint_phi_rad);
  write("max_endpoint_steering_rate_radps",
        d.endpoint_steering_rate_radps);
  write("max_total_time_equality_s", d.total_time_equality_s);
  write("max_total_time_bound_s", d.total_time_bound_s);
  write("max_ef_curvature_equality_1pm", d.ef_curvature_equality_1pm);
  write("max_ef_distance_equality_m", d.ef_distance_equality_m);
  write("max_ef_corner_equality_m", d.ef_corner_equality_m);
  write("max_ef_arc", d.ef_arc);
  write("max_ef_forward_cond1", d.ef_forward_cond1);
  write("max_ef_forward_cond2", d.ef_forward_cond2);
  write("max_ef_reverse_cond1", d.ef_reverse_cond1);
  write("max_ef_reverse_cond2", d.ef_reverse_cond2);
  write("max_obca_q_definition", d.obca_q_definition);
  write("max_obca_separation_m", d.obca_separation_m);
  write("max_obca_dual_equality", d.obca_dual_equality);
  write("max_obca_norm", d.obca_norm);
  write("max_obca_nonnegative", d.obca_nonnegative);
  write("max_map_boundary_m", d.map_boundary_m);
  if (!report) {
    if (error) *error = "Cannot write NLP failure diagnostics";
    return false;
  }
  if (!result.last_iterate_available) return true;

  const auto& snapshot = result.last_iterate;
  std::ofstream csv(fs::path(debug_directory) / "nlp_last_iterate.csv");
  if (!csv) {
    if (error) *error = "Cannot create last-iterate CSV";
    return false;
  }
  csv << std::setprecision(17)
      << "snapshot_status,record_type,node,interval,obstacle,total_time_s,"
         "x_m,y_m,theta_rad,v_mps,a_mps2,phy_rad,w_radps,dt_s,gear,"
         "k_1pm,s_m,splus_m,sminus_m,up_m,down_m,left_m,right_m,"
         "AX_m,AY_m,BX_m,BY_m,CX_m,CY_m,DX_m,DY_m,qx,qy,"
         "lambda0,lambda1,lambda2,lambda3,mu0,mu1,mu2,mu3\n";
  const std::size_t obstacle_count = result.model_obstacle_count;
  for (std::size_t i = 0; i < snapshot.trajectory.intervals.size(); ++i) {
    const std::size_t rows = std::max<std::size_t>(1, obstacle_count);
    for (std::size_t obstacle = 0; obstacle < rows; ++obstacle) {
      const auto& node = snapshot.trajectory.nodes[i];
      const auto& interval = snapshot.trajectory.intervals[i];
      csv << "UNCONVERGED_DIAGNOSTIC_DO_NOT_EXECUTE,interval," << i << ','
          << i << ',';
      if (obstacle_count) csv << obstacle; else csv << -1;
      csv << ',' << snapshot.total_time << ',' << node.x << ',' << node.y
          << ',' << node.theta << ',' << node.v << ',' << node.a << ','
          << node.phy << ',' << node.w << ',' << interval.dt << ','
          << (interval.gear ? 1 : -1) << ',' << snapshot.k[i] << ','
          << snapshot.s[i] << ',' << snapshot.splus[i] << ','
          << snapshot.sminus[i] << ',' << snapshot.up[i] << ','
          << snapshot.down[i] << ',' << snapshot.left[i] << ','
          << snapshot.right[i];
      for (double coordinate : snapshot.corners[i]) csv << ',' << coordinate;
      if (obstacle_count) {
        const std::size_t column = i * obstacle_count + obstacle;
        csv << ',' << snapshot.obca_qx[column] << ','
            << snapshot.obca_qy[column];
        for (double value : snapshot.lambda[column]) csv << ',' << value;
        for (double value : snapshot.mu[column]) csv << ',' << value;
      } else {
        csv << ",,,,,,,,,,";
      }
      csv << '\n';
    }
  }
  const std::size_t last = snapshot.trajectory.nodes.size() - 1;
  const auto& node = snapshot.trajectory.nodes[last];
  csv << "UNCONVERGED_DIAGNOSTIC_DO_NOT_EXECUTE,final_node," << last
      << ",-1,-1," << snapshot.total_time << ',' << node.x << ',' << node.y
      << ',' << node.theta << ',' << node.v << ',' << node.a << ','
      << node.phy << ',' << node.w << ",0,0";
  for (int i = 0; i < 26; ++i) csv << ',';
  csv << '\n';
  return static_cast<bool>(csv);
}

bool WriteCollisionValidation(
    const std::string& output_file, const Trajectory& trajectory,
    const VehicleParameter& vehicle,
    const std::vector<common::math::Polygon2d>& obstacles,
    const std::array<double, 4>& bounds, std::string* error) {
  std::ofstream stream(output_file);
  if (!stream) {
    if (error) *error = "Cannot create collision validation report";
    return false;
  }
  constexpr double evaluation_dt = 0.01;
  std::size_t frames = 0;
  std::size_t collision_frames = 0;
  std::size_t boundary_violation_frames = 0;
  double minimum_obstacle_distance = std::numeric_limits<double>::infinity();
  double maximum_boundary_violation = 0.0;
  if (trajectory.nodes.size() >= 2 &&
      trajectory.intervals.size() + 1 == trajectory.nodes.size()) {
    for (std::size_t i = 0; i < trajectory.intervals.size(); ++i) {
      const double dt = trajectory.intervals[i].dt;
      const std::size_t subdivisions = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(dt / evaluation_dt)));
      const std::size_t first_sample = i == 0 ? 0 : 1;
      for (std::size_t sample = first_sample; sample <= subdivisions; ++sample) {
        const double ratio = static_cast<double>(sample) /
                             static_cast<double>(subdivisions);
        const auto& first = trajectory.nodes[i];
        const auto& second = trajectory.nodes[i + 1];
        const double x = first.x + ratio * (second.x - first.x);
        const double y = first.y + ratio * (second.y - first.y);
        const double theta = first.theta + ratio *
                             (second.theta - first.theta);
        const auto box = vehicle.GenerateBox(common::math::Pose(x, y, theta));
        const common::math::Polygon2d body(box);
        bool collision = false;
        for (const auto& obstacle : obstacles) {
          const double distance = body.DistanceTo(obstacle);
          minimum_obstacle_distance = std::min(minimum_obstacle_distance,
                                                distance);
          if (body.HasOverlap(obstacle)) collision = true;
        }
        bool outside = false;
        for (const auto& corner : box.corners()) {
          const double violation = std::max({
              0.0, bounds[0] - corner.x(), corner.x() - bounds[1],
              bounds[2] - corner.y(), corner.y() - bounds[3]});
          maximum_boundary_violation = std::max(maximum_boundary_violation,
                                                 violation);
          outside = outside || violation > 0.0;
        }
        ++frames;
        if (collision) ++collision_frames;
        if (outside) ++boundary_violation_frames;
      }
    }
  }
  if (!std::isfinite(minimum_obstacle_distance)) {
    minimum_obstacle_distance = 0.0;
  }
  stream << std::setprecision(12) << std::boolalpha
         << "sampling_frequency_hz=100\n"
         << "sampling_period_s=" << evaluation_dt << '\n'
         << "obstacle_count=" << obstacles.size() << '\n'
         << "sample_frames=" << frames << '\n'
         << "collision_frames=" << collision_frames << '\n'
         << "collision_ratio="
         << (frames ? static_cast<double>(collision_frames) / frames : 0.0) << '\n'
         << "minimum_obstacle_distance_available=" << !obstacles.empty() << '\n'
         << "minimum_obstacle_distance_m=" << minimum_obstacle_distance << '\n'
         << "boundary_violation_frames=" << boundary_violation_frames << '\n'
         << "maximum_boundary_violation_m=" << maximum_boundary_violation << '\n'
         << "passed=" << (frames > 0 && collision_frames == 0 &&
                           boundary_violation_frames == 0) << '\n'
         << "continuous_time_safety_proof=false\n";
  return static_cast<bool>(stream);
}

}  // namespace optimizer
