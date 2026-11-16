#include <ros/ros.h>
#include <sandbox_msgs/AprilObject.h>
#include <sandbox_msgs/ChassisCommand.h>
#include <sandbox_msgs/Trajectory.h>
#include <sandbox_msgs/TrackingStatus.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "pure_pursuit/tracking_core.h"

namespace {
constexpr double kWheelBase = 0.143;
constexpr double kVehicleLength = 0.211;
constexpr double kVehicleWidth = 0.191;
constexpr double kRearAxleToCenter = 0.0735;

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}
const char* GearName(int gear) { return gear > 0 ? "FORWARD" : "REVERSE"; }

enum class Phase {
  kWaiting, kPreparing, kFollowing, kApproachingCusp, kCuspBraking,
  kCuspStopConfirm, kFinalApproach, kFinalStopConfirm, kCompleted, kFailed
};

const char* PhaseName(Phase phase) {
  switch (phase) {
    case Phase::kWaiting: return "WAITING";
    case Phase::kPreparing: return "PREPARING";
    case Phase::kFollowing: return "FOLLOWING";
    case Phase::kApproachingCusp: return "APPROACHING_CUSP";
    case Phase::kCuspBraking: return "CUSP_BRAKING";
    case Phase::kCuspStopConfirm: return "CUSP_STOP_CONFIRM";
    case Phase::kFinalApproach: return "FINAL_APPROACH";
    case Phase::kFinalStopConfirm: return "FINAL_STOP_CONFIRM";
    case Phase::kCompleted: return "COMPLETED";
    case Phase::kFailed: return "FAILED";
  }
  return "UNKNOWN";
}

class TrackingVisualization {
 public:
  explicit TrackingVisualization(const ros::NodeHandle& node) : node_(node) {
    node_.param<std::string>("visualization_frame", frame_, frame_);
    publisher_ = node_.advertise<visualization_msgs::Marker>(
        "/pure_pursuit", 20, true);
  }
  void SetTarget(int target) { target_ = target; }
  void Draw(const pure_pursuit::TrackingPoint& projected,
            const pure_pursuit::TrackingPoint& reference,
            double lookahead_x, double lookahead_y,
            const pure_pursuit::TrackingPoint& segment_end,
            const std::string& state, int segment, int gear,
            double actual_s, double reference_time) {
    const ros::Time now = ros::Time::now();
    const std::string ns = "Tracking_V" + std::to_string(target_);
    visualization_msgs::Marker points;
    points.header.frame_id = frame_;
    points.header.stamp = now;
    points.ns = ns; points.id = 0;
    points.type = visualization_msgs::Marker::SPHERE_LIST;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = points.scale.y = points.scale.z = 0.045;
    points.color.g = points.color.a = 1.0;
    geometry_msgs::Point point;
    point.x = projected.x; point.y = projected.y; point.z = 0.04;
    points.points.push_back(point);
    point.x = reference.x; point.y = reference.y; point.z = 0.06;
    points.points.push_back(point);
    point.x = lookahead_x; point.y = lookahead_y; point.z = 0.08;
    points.points.push_back(point);
    publisher_.publish(points);

    visualization_msgs::Marker cusp;
    cusp.header = points.header; cusp.ns = ns; cusp.id = 1;
    cusp.type = visualization_msgs::Marker::CYLINDER;
    cusp.action = visualization_msgs::Marker::ADD;
    cusp.pose.orientation.w = 1.0;
    cusp.pose.position.x = segment_end.x;
    cusp.pose.position.y = segment_end.y;
    cusp.pose.position.z = 0.025;
    cusp.scale.x = cusp.scale.y = 0.07; cusp.scale.z = 0.02;
    cusp.color.r = cusp.color.a = 1.0; cusp.color.g = 0.35;
    publisher_.publish(cusp);

    visualization_msgs::Marker arrow;
    arrow.header = points.header; arrow.ns = ns; arrow.id = 2;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.position.x = reference.x;
    arrow.pose.position.y = reference.y;
    arrow.pose.position.z = 0.08;
    arrow.pose.orientation = tf::createQuaternionMsgFromYaw(reference.yaw);
    arrow.scale.x = 0.12; arrow.scale.y = 0.025; arrow.scale.z = 0.04;
    arrow.color.b = arrow.color.a = 1.0; arrow.color.g = 0.45;
    publisher_.publish(arrow);

    visualization_msgs::Marker text;
    text.header = points.header; text.ns = ns; text.id = 3;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.pose.position.x = reference.x;
    text.pose.position.y = reference.y;
    text.pose.position.z = 0.20;
    text.scale.z = 0.05;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
    std::ostringstream label;
    label << "V" << target_ << " seg=" << segment << " "
          << GearName(gear) << " " << state << "\ns=" << std::fixed
          << std::setprecision(2) << actual_s << " t=" << reference_time;
    text.text = label.str();
    publisher_.publish(text);
  }
 private:
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  std::string frame_ = "map";
  int target_ = 0;
};

struct ObstacleObservation {
  std::vector<pure_pursuit::Point2d> polygon;
  ros::Time received;
};

class TrackingController {
 public:
  explicit TrackingController(const ros::NodeHandle& node)
      : node_(node), visualization_(node) {
    ReadParameters();
    if (target_bound_) BindTarget(tracking_object_);
    std::string trajectory_topic = "/traj_" + std::to_string(tracking_object_);
    node_.param<std::string>("trajectory_topic", trajectory_topic,
                             trajectory_topic);
    trajectory_subscriber_ = node_.subscribe<sandbox_msgs::Trajectory>(
        trajectory_topic, 1, &TrackingController::TrajectoryCallback, this);
    object_subscriber_ = node_.subscribe<sandbox_msgs::AprilObject>(
        "/object", 20, &TrackingController::ObjectCallback, this);
    estop_subscriber_ = node_.subscribe<std_msgs::Bool>(
        "/estop", 1, &TrackingController::EstopCallback, this);
    velocity_subscriber_ = node_.subscribe<std_msgs::Float64>(
        "/velocity", 1, &TrackingController::VelocityCallback, this);
    command_publisher_ = node_.advertise<sandbox_msgs::ChassisCommand>(
        "/chassis", 1, true);
    reached_publisher_ = node_.advertise<std_msgs::Int32>(
        "/reached", 1, false);
    status_publisher_ = node_.advertise<sandbox_msgs::TrackingStatus>(
        "/tracking_status", 5, true);
    timer_ = node_.createTimer(ros::Duration(0.1),
                               &TrackingController::ControlCallback, this);
  }

 private:
  void ReadParameters() {
    tracking_object_ = node_.param<int>("target", 0);
    auto_target_ = node_.param("auto_target", false);
    target_bound_ = !auto_target_;
    object_position_scale_ = node_.param("object_position_scale", 0.001);
    localization_timeout_ = node_.param("localization_timeout", 0.25);
    localization_jump_threshold_ = node_.param(
        "localization_jump_threshold", 0.50);
    execution_enabled_ = node_.param("execution_enabled", true);
    velocity_timeout_ = node_.param("velocity_timeout", 0.30);
    trust_velocity_topic_ = node_.param("trust_velocity_topic", false);
    preparation_timeout_ = node_.param("preparation_timeout", 3.0);
    debug_log_enable_ = node_.param("debug_log_enable", true);
    longitude_kp_ = node_.param("longitude_kp", longitude_kp_);
    longitude_ki_ = node_.param("longitude_ki", longitude_ki_);
    feedforward_gain_ = node_.param("velocity_feedforward_gain", 1.0);
    integral_limit_ = node_.param("longitude_integral_limit", 0.15);
    lookahead_scale_ = node_.param("lookahead_scale", lookahead_scale_);
    lookahead_min_ = node_.param("lookahead_distance_min", 0.10);
    lookahead_max_ = node_.param("lookahead_distance_max", 0.15);
    lookahead_ratio_ = node_.param("lookahead_distance_ratio", 0.8);
    max_steering_ = node_.param("pp_max_steer_angle", 0.60);
    projection_back_ = node_.param("projection_backward_window", 0.02);
    projection_forward_ = node_.param("projection_forward_window", 0.35);
    reference_lead_ = node_.param("reference_time_lead", 0.50);
    cusp_approach_ = node_.param("cusp_approach_distance", 0.15);
    cusp_position_tolerance_ = node_.param("cusp_position_tolerance", 0.035);
    cusp_yaw_tolerance_ = node_.param("cusp_yaw_tolerance", 0.12);
    cusp_stop_speed_ = node_.param("cusp_stop_speed", 0.02);
    cusp_stop_cycles_ = node_.param("cusp_stop_cycles", 3);
    cusp_timeout_ = node_.param("cusp_timeout", 10.0);
    final_lon_tolerance_ = node_.param("final_lon_tolerance", 0.02);
    final_lat_tolerance_ = node_.param("final_lat_tolerance", 0.015);
    final_yaw_tolerance_ = node_.param("final_yaw_tolerance", 0.0872665);
    final_cycles_ = node_.param("final_stable_cycles", 5);
    final_stop_speed_ = node_.param("final_stop_speed", 0.02);
    terminal_extension_ = node_.param("terminal_lookahead_extension", 0.12);
    terminal_slowdown_ = node_.param("terminal_slowdown_distance", 0.20);
    terminal_speed_ = node_.param("terminal_max_speed", 0.08);
    max_forward_speed_ = node_.param("tracking_max_forward_speed", 0.26);
    max_reverse_speed_ = node_.param("tracking_max_reverse_speed", 0.21);
    max_acceleration_ = node_.param("tracking_max_acceleration", 0.15);
    max_deceleration_ = node_.param("tracking_max_deceleration", 0.25);
    max_steering_rate_ = node_.param("tracking_max_steering_rate", 0.45);
    minimum_effective_speed_ = node_.param("minimum_effective_speed", 0.03);
    max_lateral_error_ = node_.param("max_lateral_error", 0.25);
    max_yaw_error_ = node_.param("max_yaw_error", 1.05);
    max_projection_distance_ = node_.param("max_projection_distance", 0.30);
    obstacle_timeout_ = node_.param("obstacle_timeout", 0.50);
    stagnation_timeout_ = node_.param("progress_stagnation_timeout", 4.0);
    stagnation_distance_ = node_.param("progress_stagnation_distance", 0.01);
    timeout_scale_ = node_.param("tracking_timeout_scale", 2.0);
    timeout_margin_ = node_.param("tracking_timeout_margin", 15.0);
  }

  void BindTarget(int target) {
    tracking_object_ = target;
    target_bound_ = true;
    visualization_.SetTarget(target);
    ros::NodeHandle root;
    const std::string prefix = "/controller_" + std::to_string(target);
    bool loaded = false;
    loaded = root.getParam(prefix + "/longitude_kp", longitude_kp_) || loaded;
    loaded = root.getParam(prefix + "/longitude_ki", longitude_ki_) || loaded;
    loaded = root.getParam(prefix + "/lookahead_scale", lookahead_scale_) || loaded;
    ROS_WARN("[PP] bound Vehicle%d gains=%s kp=%.3f ki=%.3f ff=%.3f "
             "lookahead_scale=%.3f", target,
             loaded ? prefix.c_str() : "generic", longitude_kp_,
             longitude_ki_, feedforward_gain_, lookahead_scale_);
  }

  int Gear() const {
    if (path_.empty() || segment_index_ < 0 ||
        segment_index_ >= static_cast<int>(path_.segments().size())) return 1;
    return path_.segment(segment_index_).gear;
  }
  double SignedVelocity() const {
    if (trust_velocity_topic_ && velocity_seen_ && !last_velocity_time_.isZero() &&
        (ros::Time::now() - last_velocity_time_).toSec() <= velocity_timeout_)
      return measured_velocity_;
    return estimated_velocity_;
  }
  double Speed() const { return std::abs(SignedVelocity()); }
  void ResetControl() {
    integral_error_ = 0.0;
    last_command_ = 0.0;
    last_steering_ = 0.0;
    stable_cycles_ = 0;
    direction_conflict_cycles_ = 0;
    motion_heading_valid_ = false;
  }
  void PublishZero() {
    sandbox_msgs::ChassisCommand command;
    command.target = tracking_object_;
    command_publisher_.publish(command);
    last_command_ = 0.0;
    last_steering_ = 0.0;
  }
  void PublishNormalStop(double dt) {
    sandbox_msgs::ChassisCommand command;
    command.target = tracking_object_;
    const double step = max_deceleration_ * dt;
    command.throttle = Clamp(0.0, last_command_ - step,
                             last_command_ + step);
    const double steer_step = max_steering_rate_ * dt;
    command.steering = Clamp(0.0, last_steering_ - steer_step,
                             last_steering_ + steer_step);
    command_publisher_.publish(command);
    last_command_ = command.throttle;
    last_steering_ = command.steering;
  }
  void Event(const std::string& text) {
    if (!result_directory_.empty()) {
      std::ofstream out(result_directory_ + "/events.log", std::ios::app);
      if (out) out << std::fixed << std::setprecision(3)
                   << ros::Time::now().toSec() << " task=" << task_id_
                   << " vehicle=" << tracking_object_ << " " << text << '\n';
    }
  }
  void PublishStatus(int status, const std::string& reason) {
    sandbox_msgs::TrackingStatus message;
    message.header.stamp = ros::Time::now();
    message.task_id = task_id_ < 0 ? 0 : static_cast<uint32_t>(task_id_);
    message.vehicle_id = tracking_object_;
    message.status = status;
    message.reason = reason;
    message.result_directory = result_directory_;
    status_publisher_.publish(message);
    Event("tracking_status=" + std::to_string(status) + " reason=" + reason);
    if (!result_directory_.empty() &&
        (status == sandbox_msgs::TrackingStatus::FAILED ||
         status == sandbox_msgs::TrackingStatus::REACHED)) {
      std::ofstream summary(result_directory_ + "/summary.txt", std::ios::app);
      if (summary)
        summary << "trajectory_validation_status="
                << (trajectory_validated_ ? "passed" : "failed") << '\n'
                << "tracking_status="
                << (status == sandbox_msgs::TrackingStatus::REACHED
                        ? "succeeded"
                        : (trajectory_validated_ ? "failed" : "not_started"))
                << '\n'
                << "tracking_reason=" << reason << '\n';
    }
  }
  void Fail(const std::string& reason) {
    if (phase_ == Phase::kFailed) return;
    failure_reason_ = reason;
    phase_ = Phase::kFailed;
    PublishZero();
    trajectory_armed_ = false;
    PublishStatus(sandbox_msgs::TrackingStatus::FAILED, reason);
    ROS_ERROR("[PP][FAILED] task=%d Vehicle%d: %s", task_id_,
              tracking_object_, reason.c_str());
    Log(pure_pursuit::TrackingPoint(), 0.0, 0.0, 0.0, 0.0, reason);
    CloseLogs();
  }

  void EstopCallback(const std_msgs::BoolConstPtr& message) {
    const bool next = !message || message->data;
    if (next && !estop_ && trajectory_armed_)
      Fail("emergency stop interrupted active task; a new task is required");
    estop_ = next;
    if (estop_) PublishZero();
  }
  void VelocityCallback(const std_msgs::Float64ConstPtr& message) {
    if (!message || !std::isfinite(message->data)) return;
    measured_velocity_ = message->data;
    velocity_seen_ = true;
    last_velocity_time_ = ros::Time::now();
  }
  void ObjectCallback(const sandbox_msgs::AprilObjectConstPtr& message) {
    if (!message) return;
    const ros::Time now = ros::Time::now();
    if (message->type == sandbox_msgs::AprilObject::OBSTACLE) {
      ObstacleObservation obstacle;
      for (const auto& point : message->polygon.points)
        obstacle.polygon.push_back({point.x * object_position_scale_,
                                    point.y * object_position_scale_});
      if (obstacle.polygon.size() >= 3) {
        obstacle.received = now;
        obstacles_[message->id] = obstacle;
      }
      return;
    }
    if (!target_bound_ || message->type != sandbox_msgs::AprilObject::VEHICLE ||
        message->id != tracking_object_) return;
    const double x = message->x * object_position_scale_;
    const double y = message->y * object_position_scale_;
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(message->yaw)) {
      if (!path_.empty()) Fail("non-finite localization sample");
      return;
    }
    const bool fresh = object_seen_ && !last_object_time_.isZero() &&
        (now - last_object_time_).toSec() <= localization_timeout_;
    if (fresh) {
      const double dx = x - object_.x, dy = y - object_.y;
      const double distance = std::hypot(dx, dy);
      if (distance > localization_jump_threshold_) {
        if (!path_.empty()) Fail("localization jump exceeds configured limit");
        return;
      }
      const double dt = (now - last_object_time_).toSec();
      if (dt > 1e-3) {
        const double signed_sample =
            (dx * std::cos(object_.yaw) + dy * std::sin(object_.yaw)) / dt;
        estimated_velocity_ = 0.65 * estimated_velocity_ +
                              0.35 * signed_sample;
        if (distance >= 0.003) {
          motion_heading_ = std::atan2(dy, dx);
          motion_heading_valid_ = true;
        }
      }
    } else {
      estimated_velocity_ = 0.0;
      motion_heading_valid_ = false;
    }
    object_ = *message;
    object_.x = x; object_.y = y;
    object_seen_ = true;
    last_object_time_ = now;
  }

  void TrajectoryCallback(const sandbox_msgs::TrajectoryConstPtr& message) {
    if (!message || message->points.size() < 2) return;
    if (auto_target_ && !target_bound_) BindTarget(message->target);
    if (!target_bound_ || message->target != tracking_object_) return;
    const int incoming_task = static_cast<int>(message->task_id);
    if (incoming_task <= last_accepted_task_) {
      ROS_WARN("[PP] ignored duplicate/stale trajectory task=%d Vehicle%d; "
               "last=%d", incoming_task, message->target,
               last_accepted_task_);
      return;
    }
    task_id_ = incoming_task;
    last_accepted_task_ = incoming_task;
    result_directory_ = message->result_directory;
    expected_obstacles_ = std::max(0, message->obstacle_count);
    trajectory_validated_ = false;
    InitializeLogs();
    PublishStatus(sandbox_msgs::TrackingStatus::PREPARING,
                  "trajectory received; waiting for fresh scene data");
    std::vector<pure_pursuit::TrackingPoint> points;
    for (const auto& source : message->points) {
      pure_pursuit::TrackingPoint point;
      point.time = source.time; point.x = source.x; point.y = source.y;
      point.yaw = source.yaw; point.velocity = source.velocity;
      point.acceleration = source.acceleration;
      point.gear = source.gear;
      points.push_back(point);
    }
    pure_pursuit::TrackingPath candidate;
    std::string error;
    if (!candidate.Build(points, &error)) {
      Fail("trajectory direction validation failed: " + error);
      return;
    }
    path_ = candidate;
    trajectory_validated_ = true;
    segment_index_ = 0;
    actual_s_ = high_water_s_ = path_.segment(0).s_begin;
    reference_time_ = path_.segment(0).time_begin;
    phase_ = Phase::kPreparing;
    trajectory_armed_ = true;
    failure_reason_.clear();
    preparation_start_ = ros::Time::now();
    last_progress_s_ = actual_s_;
    ResetControl();
    LogOpenLoop();
    PublishStatus(sandbox_msgs::TrackingStatus::VALIDATED,
                  "trajectory direction validation succeeded");
    ROS_WARN("[PP] task=%d Vehicle%d trajectory validated: points=%zu segments=%zu",
             task_id_, tracking_object_, points.size(), path_.segments().size());
  }

  bool UpdateProjection(double dt, pure_pursuit::TrackingPoint* projected) {
    const auto result = path_.Project(
        object_.x, object_.y, object_.yaw, actual_s_, Speed(), dt,
        segment_index_, projection_back_, projection_forward_,
        motion_heading_valid_, motion_heading_);
    if (!result.valid) { Fail("no local projection in active segment"); return false; }
    if (result.distance > max_projection_distance_) {
      Fail("vehicle too far from active trajectory segment"); return false;
    }
    high_water_s_ = std::max(high_water_s_, result.s);
    actual_s_ = std::max(result.s, high_water_s_ - projection_back_);
    actual_s_ = Clamp(actual_s_, path_.segment(segment_index_).s_begin,
                      path_.segment(segment_index_).s_end);
    *projected = path_.SampleAtS(actual_s_, segment_index_);
    projection_distance_ = result.distance;
    return true;
  }

  bool Safety(const pure_pursuit::TrackingPoint& projected) {
    const double dx = object_.x - projected.x, dy = object_.y - projected.y;
    lateral_error_ = -std::sin(projected.yaw) * dx +
                     std::cos(projected.yaw) * dy;
    yaw_error_ = pure_pursuit::NormalizeAngle(object_.yaw - projected.yaw);
    if (std::abs(lateral_error_) > max_lateral_error_) {
      Fail("lateral error exceeds safety limit"); return false;
    }
    if (std::abs(yaw_error_) > max_yaw_error_) {
      Fail("heading error exceeds safety limit"); return false;
    }
    minimum_clearance_ = std::numeric_limits<double>::infinity();
    const auto footprint = pure_pursuit::VehicleFootprint(
        object_.x, object_.y, object_.yaw, kRearAxleToCenter,
        kVehicleLength, kVehicleWidth);
    for (const auto& item : obstacles_) {
      if ((ros::Time::now() - item.second.received).toSec() > obstacle_timeout_) {
        Fail("obstacle geometry is stale"); return false;
      }
      minimum_clearance_ = std::min(minimum_clearance_,
          pure_pursuit::PolygonDistance(footprint, item.second.polygon));
    }
    if (minimum_clearance_ <= 1e-9) {
      Fail("actual vehicle footprint overlaps an obstacle");
      return false;
    }
    if (std::abs(SignedVelocity()) > 0.03 && SignedVelocity() * Gear() < 0.0) {
      if (++direction_conflict_cycles_ >= 3) {
        Fail("measured velocity conflicts with active gear"); return false;
      }
    } else direction_conflict_cycles_ = 0;
    return true;
  }

  bool UpdatePhase() {
    const auto& segment = path_.segment(segment_index_);
    const auto endpoint = path_.points()[segment.point_end];
    const double remaining = std::max(0.0, segment.s_end - actual_s_);
    const double position_error = std::hypot(endpoint.x - object_.x,
                                             endpoint.y - object_.y);
    const double heading_error = std::abs(pure_pursuit::NormalizeAngle(
        endpoint.yaw - object_.yaw));
    const bool final = segment_index_ + 1 ==
                       static_cast<int>(path_.segments().size());
    if (!final) {
      if (phase_ == Phase::kFollowing && remaining <= cusp_approach_) {
        phase_ = Phase::kApproachingCusp;
        phase_start_ = ros::Time::now();
        Event("approaching cusp segment=" + std::to_string(segment_index_));
        ROS_INFO("[PP] task=%d Vehicle%d segment=%d approaching cusp",
                 task_id_, tracking_object_, segment_index_);
      }
      if (phase_ == Phase::kApproachingCusp &&
          position_error <= cusp_position_tolerance_ &&
          heading_error <= cusp_yaw_tolerance_) {
        phase_ = Phase::kCuspBraking;
        integral_error_ = 0.0;
        Event("cusp pose reached; controlled braking");
      }
      if ((phase_ == Phase::kApproachingCusp ||
           phase_ == Phase::kCuspBraking ||
           phase_ == Phase::kCuspStopConfirm) &&
          (ros::Time::now() - phase_start_).toSec() > cusp_timeout_) {
        Fail("cusp transition timed out"); return false;
      }
      if (phase_ == Phase::kCuspBraking) {
        if (position_error > cusp_position_tolerance_ ||
            heading_error > cusp_yaw_tolerance_) {
          phase_ = Phase::kApproachingCusp;
          stable_cycles_ = 0;
        } else if (Speed() <= cusp_stop_speed_ &&
                   std::abs(last_command_) <= 0.005) {
          phase_ = Phase::kCuspStopConfirm; stable_cycles_ = 1;
          Event("cusp stationary confirmation started");
        }
        return false;
      }
      if (phase_ == Phase::kCuspStopConfirm) {
        if (position_error > cusp_position_tolerance_ ||
            heading_error > cusp_yaw_tolerance_) {
          phase_ = Phase::kApproachingCusp;
          stable_cycles_ = 0;
        } else if (Speed() <= cusp_stop_speed_ &&
                   std::abs(last_command_) <= 0.005) ++stable_cycles_;
        else { phase_ = Phase::kCuspBraking; stable_cycles_ = 0; }
        if (stable_cycles_ >= cusp_stop_cycles_) {
          ++segment_index_;
          const auto& next = path_.segment(segment_index_);
          actual_s_ = high_water_s_ = next.s_begin;
          reference_time_ = next.time_begin;
          phase_ = Phase::kFollowing;
          phase_start_ = last_progress_time_ = ros::Time::now();
          last_progress_s_ = actual_s_;
          ResetControl();
          PublishStatus(sandbox_msgs::TrackingStatus::SEGMENT_CHANGED,
                        "confirmed stop; activated segment " +
                        std::to_string(segment_index_));
          ROS_WARN("[PP] Vehicle%d segment=%d gear=%s after confirmed stop",
                   tracking_object_, segment_index_, GearName(next.gear));
        }
        return false;
      }
    } else if (phase_ == Phase::kFollowing &&
               remaining <= terminal_slowdown_) {
      phase_ = Phase::kFinalApproach;
      Event("entered final approach");
      ROS_INFO("[PP] task=%d Vehicle%d entered final approach",
               task_id_, tracking_object_);
    }

    if (final && phase_ == Phase::kFinalApproach) {
      const double c = std::cos(endpoint.yaw), s = std::sin(endpoint.yaw);
      const double dx = endpoint.x - object_.x, dy = endpoint.y - object_.y;
      const double e_lon = c * dx + s * dy;
      const double e_lat = -s * dx + c * dy;
      if (std::abs(e_lon) <= final_lon_tolerance_ &&
          std::abs(e_lat) <= final_lat_tolerance_ &&
          heading_error <= final_yaw_tolerance_) {
        phase_ = Phase::kFinalStopConfirm;
        phase_start_ = ros::Time::now();
        stable_cycles_ = Speed() <= final_stop_speed_ &&
                         std::abs(last_command_) <= 0.005 ? 1 : 0;
        integral_error_ = 0.0;
        Event("terminal pose reached; confirming stationary state");
        return false;
      }
    }
    if (final && phase_ == Phase::kFinalStopConfirm) {
      const double c = std::cos(endpoint.yaw), s = std::sin(endpoint.yaw);
      const double dx = endpoint.x - object_.x, dy = endpoint.y - object_.y;
      const bool pose_valid =
          std::abs(c * dx + s * dy) <= final_lon_tolerance_ &&
          std::abs(-s * dx + c * dy) <= final_lat_tolerance_ &&
          heading_error <= final_yaw_tolerance_;
      if (pose_valid && Speed() <= final_stop_speed_ &&
          std::abs(last_command_) <= 0.005) ++stable_cycles_;
      else if (!pose_valid) {
        phase_ = Phase::kFinalApproach;
        stable_cycles_ = 0;
      } else stable_cycles_ = 0;
      if (stable_cycles_ >= final_cycles_) {
        phase_ = Phase::kCompleted;
        PublishZero();
        std_msgs::Int32 reached; reached.data = tracking_object_;
        reached_publisher_.publish(reached);
        trajectory_armed_ = false;
        PublishStatus(sandbox_msgs::TrackingStatus::REACHED,
                      "terminal pose and stationary state confirmed");
        CloseLogs();
        ROS_INFO("[PP] task=%d Vehicle%d reached terminal pose",
                 task_id_, tracking_object_);
        return false;
      }
      return false;
    }
    return phase_ == Phase::kFollowing ||
           phase_ == Phase::kApproachingCusp ||
           phase_ == Phase::kFinalApproach;
  }

  pure_pursuit::TrackingPoint Reference(double dt) {
    const auto& segment = path_.segment(segment_index_);
    const double projected_time = path_.SampleAtS(actual_s_, segment_index_).time;
    const double upper = std::min(segment.time_end,
                                  projected_time + reference_lead_);
    reference_time_ = std::max(reference_time_,
                               std::min(reference_time_ + dt, upper));
    reference_time_ = Clamp(reference_time_, segment.time_begin, segment.time_end);
    return path_.SampleAtTime(reference_time_, segment_index_);
  }

  void Lookahead(const pure_pursuit::TrackingPoint& reference,
                 double* x, double* y) {
    const auto& segment = path_.segment(segment_index_);
    lookahead_distance_ = Clamp(std::abs(reference.velocity) * lookahead_ratio_,
                                lookahead_min_, lookahead_max_) * lookahead_scale_;
    const double target_s = std::min(segment.s_end, actual_s_ + lookahead_distance_);
    const auto target = path_.SampleAtS(target_s, segment_index_);
    *x = target.x; *y = target.y;
    if (target_s >= segment.s_end - 1e-8 &&
        segment.s_end - actual_s_ < lookahead_distance_) {
      const double extension = std::min(
          terminal_extension_, lookahead_distance_ - (segment.s_end - actual_s_));
      const double motion_yaw = target.yaw + (segment.gear < 0 ? M_PI : 0.0);
      *x += extension * std::cos(motion_yaw);
      *y += extension * std::sin(motion_yaw);
    }
  }

  double Longitudinal(const pure_pursuit::TrackingPoint& reference, double dt) {
    const double dx = reference.x - object_.x, dy = reference.y - object_.y;
    longitudinal_error_ = std::cos(reference.yaw) * dx +
                          std::sin(reference.yaw) * dy;
    const double old_integral = integral_error_;
    integral_error_ = Clamp(integral_error_ + longitudinal_error_ * dt,
                            -integral_limit_, integral_limit_);
    double command = feedforward_gain_ * reference.velocity +
                     longitude_kp_ * longitudinal_error_ +
                     longitude_ki_ * integral_error_;
    const double low = Gear() > 0 ? 0.0 : -max_reverse_speed_;
    const double high = Gear() > 0 ? max_forward_speed_ : 0.0;
    const double saturated = Clamp(command, low, high);
    if (saturated != command && command * longitudinal_error_ > 0.0)
      integral_error_ = old_integral;
    command = saturated;
    const auto& segment = path_.segment(segment_index_);
    const double remaining = std::max(0.0, segment.s_end - actual_s_);
    if (phase_ == Phase::kApproachingCusp) {
      const double maximum = std::max(minimum_effective_speed_,
          std::max(max_forward_speed_, max_reverse_speed_) *
          remaining / std::max(1e-3, cusp_approach_));
      command = Clamp(command, Gear() > 0 ? 0.0 : -maximum,
                      Gear() > 0 ? maximum : 0.0);
    }
    if (phase_ == Phase::kFinalApproach)
      command = Clamp(command, Gear() > 0 ? 0.0 : -terminal_speed_,
                      Gear() > 0 ? terminal_speed_ : 0.0);
    if (remaining > cusp_position_tolerance_ && std::abs(command) > 1e-6 &&
        std::abs(command) < minimum_effective_speed_)
      command = Gear() * minimum_effective_speed_;
    const double change = (std::abs(command) > std::abs(last_command_)
                               ? max_acceleration_ : max_deceleration_) * dt;
    return Clamp(Clamp(command, last_command_ - change, last_command_ + change),
                 low, high);
  }

  double Steering(double x, double y) const {
    const double dx = x - object_.x, dy = y - object_.y;
    const double relative_y = -std::sin(object_.yaw) * dx +
                              std::cos(object_.yaw) * dy;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < 1e-8) return 0.0;
    // Rear-axle model: kappa=2*y/Ld^2. Signed velocity handles reverse;
    // applying an extra reverse steering negation would be incorrect.
    return Clamp(std::atan2(2.0 * kWheelBase * relative_y, distance_squared),
                 -max_steering_, max_steering_);
  }

  bool ProgressAndTimeout(double command) {
    if (actual_s_ >= last_progress_s_ + stagnation_distance_) {
      last_progress_s_ = actual_s_;
      last_progress_time_ = ros::Time::now();
    } else if (std::abs(command) >= minimum_effective_speed_ &&
               (ros::Time::now() - last_progress_time_).toSec() >
                   stagnation_timeout_) {
      Fail("progress stalled with active motion command"); return false;
    }
    return true;
  }

  void ControlCallback(const ros::TimerEvent& event) {
    const double dt = Clamp((event.current_real - event.last_real).toSec(),
                            0.02, 0.25);
    if (!execution_enabled_ || estop_ || path_.empty() ||
        phase_ == Phase::kWaiting || phase_ == Phase::kCompleted ||
        phase_ == Phase::kFailed) { PublishZero(); return; }
    if (phase_ == Phase::kPreparing) {
      PublishZero();
      const bool vehicle_ready = object_seen_ && !last_object_time_.isZero() &&
          (ros::Time::now() - last_object_time_).toSec() <= localization_timeout_;
      bool obstacles_ready =
          static_cast<int>(obstacles_.size()) >= expected_obstacles_;
      for (const auto& item : obstacles_)
        obstacles_ready = obstacles_ready &&
            (ros::Time::now() - item.second.received).toSec() <= obstacle_timeout_;
      if (vehicle_ready && obstacles_ready) {
        phase_ = Phase::kFollowing;
        task_start_ = phase_start_ = last_progress_time_ = ros::Time::now();
        last_progress_s_ = actual_s_;
        ResetControl();
        PublishStatus(sandbox_msgs::TrackingStatus::STARTED,
                      "fresh vehicle localization and obstacle set received");
        ROS_WARN("[PP] task=%d Vehicle%d tracking started",
                 task_id_, tracking_object_);
      } else if ((ros::Time::now() - preparation_start_).toSec() >
                 preparation_timeout_) {
        Fail(!vehicle_ready ? "startup localization timeout"
                            : "startup obstacle-set timeout");
      }
      return;
    }
    const double duration = path_.points().back().time - path_.points().front().time;
    if ((ros::Time::now() - task_start_).toSec() >
        duration * timeout_scale_ + timeout_margin_) {
      Fail("overall tracking timed out in state " +
           std::string(PhaseName(phase_)));
      return;
    }
    if (!object_seen_ || last_object_time_.isZero() ||
        (ros::Time::now() - last_object_time_).toSec() > localization_timeout_) {
      Fail("vehicle localization missing or stale"); return;
    }
    pure_pursuit::TrackingPoint projected;
    if (!UpdateProjection(dt, &projected) || !Safety(projected)) return;
    if (!UpdatePhase()) {
      if (phase_ == Phase::kCompleted || phase_ == Phase::kFailed)
        PublishZero();
      else
        PublishNormalStop(dt);
      const auto reference = path_.SampleAtS(actual_s_, segment_index_);
      DrawAndLog(projected, reference, 0.0, 0.0, reference.x, reference.y, "");
      return;
    }
    const auto reference = Reference(dt);
    double lookahead_x = 0.0, lookahead_y = 0.0;
    Lookahead(reference, &lookahead_x, &lookahead_y);
    const double throttle = Longitudinal(reference, dt);
    const double desired_steering = Steering(lookahead_x, lookahead_y);
    const double steering = Clamp(
        desired_steering,
        last_steering_ - max_steering_rate_ * dt,
        last_steering_ + max_steering_rate_ * dt);
    if (!ProgressAndTimeout(throttle)) return;
    sandbox_msgs::ChassisCommand command;
    command.target = tracking_object_;
    command.throttle = throttle;
    command.steering = steering;
    command_publisher_.publish(command);
    last_command_ = throttle;
    last_steering_ = steering;
    DrawAndLog(projected, reference, throttle, steering,
               lookahead_x, lookahead_y, "");
  }

  void DrawAndLog(const pure_pursuit::TrackingPoint& projected,
                  const pure_pursuit::TrackingPoint& reference,
                  double throttle, double steering, double lookahead_x,
                  double lookahead_y, const std::string& reason) {
    const auto endpoint = path_.points()[path_.segment(segment_index_).point_end];
    visualization_.Draw(projected, reference, lookahead_x, lookahead_y,
                        endpoint, PhaseName(phase_), segment_index_, Gear(),
                        actual_s_, reference_time_);
    Log(reference, throttle, steering, lookahead_x, lookahead_y, reason);
  }

  void InitializeLogs() {
    if (!debug_log_enable_) return;
    if (result_directory_.empty()) return;
    if (open_loop_log_.is_open()) open_loop_log_.close();
    if (tracking_log_.is_open()) tracking_log_.close();
    const std::string prefix = result_directory_ + "/pp";
    open_loop_log_.open((prefix + "_open_loop.csv").c_str());
    tracking_log_.open((prefix + "_tracking.csv").c_str());
    if (open_loop_log_)
      open_loop_log_ << "index,time,x,y,yaw,velocity,acceleration,s\n";
    if (tracking_log_)
      tracking_log_ << "wall_time,target,task,segment,gear,phase,actual_s,"
                       "reference_time,actual_x,actual_y,actual_yaw,actual_velocity,"
                       "ref_x,ref_y,ref_yaw,ref_velocity,lookahead_x,lookahead_y,"
                       "longitudinal_error,lateral_error,yaw_error,projection_distance,"
                       "minimum_obstacle_distance,throttle,steering,failure_reason\n";
    std::ofstream parameters(result_directory_ +
                             "/effective_controller_parameters.yaml");
    if (parameters)
      parameters << "vehicle_id: " << tracking_object_ << '\n'
                 << "task_id: " << task_id_ << '\n'
                 << "longitude_kp: " << longitude_kp_ << '\n'
                 << "longitude_ki: " << longitude_ki_ << '\n'
                 << "velocity_feedforward_gain: " << feedforward_gain_ << '\n'
                 << "lookahead_scale: " << lookahead_scale_ << '\n'
                 << "collision_stop_condition: footprint_overlap\n";
    ROS_WARN("[PP] task=%d Vehicle%d logs=%s", task_id_, tracking_object_,
             result_directory_.c_str());
  }
  void CloseLogs() {
    if (open_loop_log_.is_open()) open_loop_log_.close();
    if (tracking_log_.is_open()) tracking_log_.close();
  }
  void LogOpenLoop() {
    if (!open_loop_log_) return;
    for (std::size_t i = 0; i < path_.points().size(); ++i) {
      const auto& p = path_.points()[i];
      open_loop_log_ << i << ',' << p.time << ',' << p.x << ',' << p.y << ','
                     << p.yaw << ',' << p.velocity << ',' << p.acceleration
                     << ',' << p.s << '\n';
    }
    open_loop_log_.flush();
  }
  void Log(const pure_pursuit::TrackingPoint& reference, double throttle,
           double steering, double lookahead_x, double lookahead_y,
           const std::string& reason) {
    if (!tracking_log_) return;
    tracking_log_ << ros::Time::now().toSec() << ',' << tracking_object_ << ','
                  << task_id_ << ',' << segment_index_ << ',' << Gear() << ','
                  << PhaseName(phase_) << ',' << actual_s_ << ','
                  << reference_time_ << ',' << object_.x << ',' << object_.y
                  << ',' << object_.yaw << ',' << SignedVelocity() << ','
                  << reference.x << ',' << reference.y << ',' << reference.yaw
                  << ',' << reference.velocity << ',' << lookahead_x << ','
                  << lookahead_y << ',' << longitudinal_error_ << ','
                  << lateral_error_ << ',' << yaw_error_ << ','
                  << projection_distance_ << ',' << minimum_clearance_ << ','
                  << throttle << ',' << steering << ',' << reason << '\n';
    tracking_log_.flush();
  }

  ros::NodeHandle node_;
  ros::Subscriber trajectory_subscriber_, object_subscriber_, estop_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Publisher command_publisher_, reached_publisher_, status_publisher_;
  ros::Timer timer_;
  TrackingVisualization visualization_;
  pure_pursuit::TrackingPath path_;
  sandbox_msgs::AprilObject object_;
  std::map<int, ObstacleObservation> obstacles_;
  int tracking_object_ = 0, task_id_ = -1, last_accepted_task_ = -1;
  int segment_index_ = 0, expected_obstacles_ = 0;
  bool auto_target_ = false, target_bound_ = false, execution_enabled_ = true;
  bool object_seen_ = false, estop_ = true, velocity_seen_ = false;
  bool trust_velocity_topic_ = false, trajectory_armed_ = false;
  bool trajectory_validated_ = false;
  bool motion_heading_valid_ = false;
  Phase phase_ = Phase::kWaiting;
  std::string failure_reason_;
  ros::Time last_object_time_, last_velocity_time_, task_start_, phase_start_;
  ros::Time preparation_start_;
  ros::Time last_progress_time_;
  double measured_velocity_ = 0.0, estimated_velocity_ = 0.0;
  double motion_heading_ = 0.0, actual_s_ = 0.0, high_water_s_ = 0.0;
  double reference_time_ = 0.0, last_progress_s_ = 0.0;
  double last_command_ = 0.0, last_steering_ = 0.0, integral_error_ = 0.0;
  int stable_cycles_ = 0, direction_conflict_cycles_ = 0;
  double object_position_scale_ = 0.001, localization_timeout_ = 0.25;
  double localization_jump_threshold_ = 0.50, velocity_timeout_ = 0.30;
  double preparation_timeout_ = 3.0;
  double longitude_kp_ = 1.0, longitude_ki_ = 0.1, feedforward_gain_ = 1.0;
  double integral_limit_ = 0.15, lookahead_scale_ = 1.0;
  double lookahead_min_ = 0.10, lookahead_max_ = 0.15, lookahead_ratio_ = 0.8;
  double lookahead_distance_ = 0.10, max_steering_ = 0.60;
  double projection_back_ = 0.02, projection_forward_ = 0.35;
  double reference_lead_ = 0.50, cusp_approach_ = 0.15;
  double cusp_position_tolerance_ = 0.035, cusp_yaw_tolerance_ = 0.12;
  double cusp_stop_speed_ = 0.02, cusp_timeout_ = 10.0;
  int cusp_stop_cycles_ = 3;
  double final_lon_tolerance_ = 0.02, final_lat_tolerance_ = 0.015;
  double final_yaw_tolerance_ = 0.0872665, final_stop_speed_ = 0.02;
  int final_cycles_ = 5;
  double terminal_extension_ = 0.12, terminal_slowdown_ = 0.20;
  double terminal_speed_ = 0.08, max_forward_speed_ = 0.26;
  double max_reverse_speed_ = 0.21, max_acceleration_ = 0.15;
  double max_deceleration_ = 0.25, max_steering_rate_ = 0.45;
  double minimum_effective_speed_ = 0.03;
  double max_lateral_error_ = 0.25, max_yaw_error_ = 1.05;
  double max_projection_distance_ = 0.30;
  double obstacle_timeout_ = 0.50, stagnation_timeout_ = 4.0;
  double stagnation_distance_ = 0.01, timeout_scale_ = 2.0;
  double timeout_margin_ = 15.0;
  double longitudinal_error_ = 0.0, lateral_error_ = 0.0, yaw_error_ = 0.0;
  double projection_distance_ = 0.0;
  double minimum_clearance_ = std::numeric_limits<double>::infinity();
  bool debug_log_enable_ = true;
  std::string result_directory_;
  std::ofstream open_loop_log_, tracking_log_;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_pursuit");
  ros::NodeHandle node("~");
  TrackingController controller(node);
  ros::spin();
  return 0;
}
