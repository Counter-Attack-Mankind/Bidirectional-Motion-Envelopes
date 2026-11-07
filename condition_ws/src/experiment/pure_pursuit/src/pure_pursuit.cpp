#include <ros/ros.h>
#include <fstream>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sandbox_msgs/Trajectory.h>
#include <sandbox_msgs/AprilObject.h>
#include <sandbox_msgs/ChassisCommand.h>
#include <visualization_msgs/Marker.h>

// Pure Pursuit Parameters
const double lfw_ = 0.0; // distance of the forward anchor point from the rear axle
const double lrv_ = 0.0;
const double wheel_base_ = 0.1445;

// Goal Approaching
const double approaching_tolerance_ = 0.04; // 5cm

using namespace sandbox_msgs;

class PurePursuitVisualization {
public:
  explicit PurePursuitVisualization(const ros::NodeHandle &nh): node_handle_(nh) {
    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/pure_pursuit", 1, true);
  }

  void set_target(int target) {
    target_ = target;
    init_marker();
  }

  void visualize_goal(double goal_x, double goal_y, double goal_yaw) {
    goal_.header.stamp = ros::Time::now();
    goal_.pose.position.x = goal_x;
    goal_.pose.position.y = goal_y;
    goal_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);

    marker_publisher_.publish(goal_);
  }

  void visualize(double pos_x, double pos_y, double car_x, double car_y, double waypoint_x, double waypoint_y) {
    points_.header.stamp = ros::Time::now();
    guide_line_.header.stamp = ros::Time::now();
    points_.points.clear();
    guide_line_.points.clear();

    points_.points.push_back(make_point(car_x, car_y));
    points_.points.push_back(make_point(pos_x, pos_y));
    points_.points.push_back(make_point(waypoint_x, waypoint_y));
    guide_line_.points.push_back(make_point(pos_x, pos_y));
    guide_line_.points.push_back(make_point(waypoint_x, waypoint_y));

    marker_publisher_.publish(points_);
    marker_publisher_.publish(guide_line_);
  }

  void reset_actual_path() {
    actual_path_.header.stamp = ros::Time::now();
    actual_path_.points.clear();
    last_path_x_ = std::numeric_limits<double>::quiet_NaN();
    last_path_y_ = std::numeric_limits<double>::quiet_NaN();
    marker_publisher_.publish(actual_path_);
  }

  void append_actual_path(double x, double y) {
    if (!std::isfinite(last_path_x_) || !std::isfinite(last_path_y_) ||
        std::hypot(x - last_path_x_, y - last_path_y_) >= 0.005) {
      actual_path_.header.stamp = ros::Time::now();
      actual_path_.points.push_back(make_point(x, y));
      last_path_x_ = x;
      last_path_y_ = y;
      marker_publisher_.publish(actual_path_);
    }
  }

  void visualize_pose(double x, double y, double theta) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    std::stringstream ss;
    ss << "PurePursuitTruth_" << target_;
    marker.ns = ss.str();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;

    marker.scale.x = 0.05;
    marker.scale.y = 0.01;
    marker.scale.z = 0.03;
    marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0;

    marker_publisher_.publish(marker);
  }

private:
  static inline geometry_msgs::Point make_point(double x, double y, double z = 0.0) {
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
  }

  void init_marker() {
    points_.header.frame_id = guide_line_.header.frame_id = goal_.header.frame_id = actual_path_.header.frame_id = "world";
    std::stringstream pp_ns;
    pp_ns << "PurePursuit_" << target_;
    points_.ns = guide_line_.ns = goal_.ns = pp_ns.str();
    std::stringstream track_ns;
    track_ns << "PurePursuitTrack_" << target_;
    actual_path_.ns = track_ns.str();
    points_.action = guide_line_.action = goal_.action = actual_path_.action = visualization_msgs::Marker::ADD;
    points_.pose.orientation.w = guide_line_.pose.orientation.w = goal_.pose.orientation.w = actual_path_.pose.orientation.w = 1.0;
    points_.id = 0;
    guide_line_.id = 1;
    goal_.id = 2;
    actual_path_.id = 3;

    points_.type = visualization_msgs::Marker::POINTS;
    guide_line_.type = visualization_msgs::Marker::LINE_STRIP;
    goal_.type = visualization_msgs::Marker::CYLINDER;
    actual_path_.type = visualization_msgs::Marker::LINE_STRIP;
    // POINTS markers use x and y scale for width/height respectively
    points_.scale.x = 0.02;
    points_.scale.y = 0.02;

    //LINE_STRIP markers use only the x component of scale, for the line width
    guide_line_.scale.x = 0.01;
    actual_path_.scale.x = 0.012;

    goal_.scale.x = approaching_tolerance_;
    goal_.scale.y = approaching_tolerance_;
    goal_.scale.z = 0.1;

    // Points are green
    points_.color.g = 1.0f;
    points_.color.a = 1.0;

    // Line strip is blue
    guide_line_.color.b = 1.0;
    guide_line_.color.a = 1.0;

    // Actual tracked path is gray to distinguish it from the planned green path.
    actual_path_.color.r = 0.45;
    actual_path_.color.g = 0.45;
    actual_path_.color.b = 0.45;
    actual_path_.color.a = 1.0;

    //goalCircle_ is yellow
    goal_.color.r = 1.0;
    goal_.color.g = 1.0;
    goal_.color.b = 0.0;
    goal_.color.a = 0.5;
  }

  int target_ = 0;
  ros::NodeHandle node_handle_;
  ros::Publisher marker_publisher_;
  visualization_msgs::Marker points_, guide_line_, goal_, actual_path_;
  double last_path_x_ = std::numeric_limits<double>::quiet_NaN();
  double last_path_y_ = std::numeric_limits<double>::quiet_NaN();
};

TrajectoryPoint interpolate(const TrajectoryPoint p0, TrajectoryPoint p1, double weight) {
  TrajectoryPoint pt;
  pt.time = (1 - weight) * p0.time + weight * p1.time;
  pt.x = (1 - weight) * p0.x + weight * p1.x;
  pt.y = (1 - weight) * p0.y + weight * p1.y;
  pt.yaw = (1 - weight) * p0.yaw + weight * p1.yaw;
  pt.velocity = (1 - weight) * p0.velocity + weight * p1.velocity;
  pt.acceleration = (1 - weight) * p0.acceleration + weight * p1.acceleration;
  return pt;
}

TrajectoryPoint interpolateWithTime(const TrajectoryPoint p0, TrajectoryPoint p1, double time) {
  double weight = (time - p0.time) / (p1.time - p0.time);
  return interpolate(p0, p1, weight);
}

class PurePursuit {
public:
  PurePursuit(const ros::NodeHandle &nh): node_handle_(nh), visualization_(nh) {
    tracking_object_ = node_handle_.param<int>("target", 0);
    K_e_          = node_handle_.param<double>("K_e",   15.0);
    K_yaw_        = node_handle_.param<double>("K_yaw", 1.8);
    longitude_kp_ = node_handle_.param<double>("Kp",    1.0);
    ROS_INFO("Gains: K_e=%.1f  K_yaw=%.2f  Kp=%.2f", K_e_, K_yaw_, longitude_kp_);
    visualization_.set_target(tracking_object_);

    traj_subscriber_ = node_handle_.subscribe<Trajectory>("/traj", 5, &PurePursuit::traj_callback, this);
    object_subscriber_ = node_handle_.subscribe<AprilObject>("/object", 5, &PurePursuit::object_callback, this);
    command_publisher_ = node_handle_.advertise<ChassisCommand>("/chassis", 1, true);
    reached_publisher_ = node_handle_.advertise<std_msgs::Int32>("/reached", 1, false);

    timer_ = node_handle_.createTimer(ros::Duration(0.02), &PurePursuit::control_callback, this);

    // CSV log for debugging
    pp_log_.open("/tmp/pp_tracking_log.csv");
    pp_log_ << "time,obj_x,obj_y,obj_yaw,ref_x,ref_y,ref_yaw,ref_v,"
            << "car_idx,lk_idx,relative_x,throttle,steering,"
            << "approach_dist,approached,elapsed" << std::endl;
    printf("PP log: /tmp/pp_tracking_log.csv\n");
  }

private:
  ros::NodeHandle node_handle_;
  ros::Publisher command_publisher_, reached_publisher_;
  ros::Subscriber traj_subscriber_, object_subscriber_, truth_subscriber_, execute_subscriber_;
  ros::Timer timer_;
  double start_time_ = 0.0;
  PurePursuitVisualization visualization_;
  Trajectory trajectory_;
  AprilObject object_;
  std::vector<std::pair<int, int>> ranges_;

  int current_range_ = 0;
  int tracking_object_ = 0;
  std::ofstream pp_log_;

  double lookahead_distance_ = 0.18, lookahead_gain_ = 1.0;
  double max_steering_ = 0.3491, min_steering_ = -0.3491;
  double longitude_kp_ = 1.0, longitude_ki_ = 0.0, longitude_output_ = 0.0, longitude_perror_ = 0.0;
  int car_index_ = 0, lookahead_index_ = 0;
  double K_e_ = 15.0, K_yaw_ = 1.8;    // 横向增益（从 ROS 参数读取）

  bool approached_ = false;

  void object_callback(const AprilObjectConstPtr &msg) {
    if(msg->id == tracking_object_ && msg->type == AprilObject::VEHICLE) {
      if(object_.x != 0 && object_.y != 0) {
        double distance = hypot(msg->x / 1000.0 - object_.x, msg->y / 1000.0 - object_.y);
        if(distance <= 0.5) object_ = *msg;
      } else {
        object_ = *msg;
      }
      object_.x = object_.x / 1000.0;  // mm → m
      object_.y = object_.y / 1000.0;
      visualization_.append_actual_path(object_.x, object_.y);
    }
  }

  void traj_callback(const TrajectoryConstPtr &msg) {
//    if(msg->target != tracking_object_) return;

    std::cout << tracking_object_ << " - Trajectory received" << std::endl;

    car_index_ = lookahead_index_ = 0;

    if(approached_) {
      current_range_ = 0;
    }

    // find ranges
    ranges_.clear();
    int start = 0;
    for(int i = 1; i < msg->points.size(); i++) {
      if(copysign(1.0, msg->points[i].velocity) != copysign(1.0, msg->points[i-1].velocity)) {
        ranges_.emplace_back(start, i);
        start = i;
      }
    }
    ranges_.emplace_back(start, msg->points.size());

    auto next_goal = msg->points[ranges_[current_range_].second];
    visualization_.visualize_goal(next_goal.x, next_goal.y, next_goal.yaw);

    if(trajectory_.points.empty() || approached_) {
      start_time_ = ros::Time::now().toSec();
      ROS_INFO("restart timer");
    }

    trajectory_ = *msg;
    approached_ = false;
    visualization_.reset_actual_path();
  }

  inline double distance_to_traj(int i, double x, double y) {
    return hypot(trajectory_.points[i].y - y, trajectory_.points[i].x - x);
  }

  void update_lookahead() {
    if(trajectory_.points.empty()) return;

    bool found_lookahead = false;
    int start = ranges_[current_range_].first, end = ranges_[current_range_].second;

    double min_distance = FLT_MAX;
    for(int i = car_index_; i < end; i++) {
      double distance = distance_to_traj(i, object_.x, object_.y);
      if(distance < min_distance) {
        car_index_ = i;
        min_distance = distance;
      }
    }

    double v_cmd = fabs(trajectory_.points[car_index_].velocity);
    if(v_cmd < 0.1) {
      lookahead_distance_ = 0.08;
    } else if(v_cmd > 0.2) {
      lookahead_distance_ = 0.16;
    } else {
      lookahead_distance_ = v_cmd * 0.5;
    }

    lookahead_distance_ *= lookahead_gain_;

    for(int i = std::max(car_index_, lookahead_index_); i < end; i++) {
      double distance = distance_to_traj(i, object_.x, object_.y);

      if(distance >= lookahead_distance_) {
        lookahead_index_ = i;
        found_lookahead = true;
        break;
      }
    }

    auto goal = trajectory_.points[end-1];
    if(!found_lookahead) {
      lookahead_index_ = end-1;
    }

    visualization_.visualize(object_.x, object_.y,
                             trajectory_.points[car_index_].x, trajectory_.points[car_index_].y,
                             trajectory_.points[lookahead_index_].x, trajectory_.points[lookahead_index_].y);
  }

  void switch_range() {
    // already final path
    int end = ranges_[current_range_].second;
    double approaching_distance = distance_to_traj(end-1, object_.x, object_.y);
    double tolerance = current_range_ == ranges_.size() - 1 ? 0.01 : approaching_tolerance_;

    if(approaching_distance <= tolerance) {
      if(current_range_ == ranges_.size() - 1) {
        approached_ = true;
        std_msgs::Int32 msg;
        msg.data = tracking_object_;
        reached_publisher_.publish(msg);
        longitude_output_ = longitude_perror_ = 0.0;
        std::cout << tracking_object_ << " - Goal reached" << std::endl;
      } else {
        // changing goal
        current_range_++;

        end = ranges_[current_range_].second;
        auto goal = trajectory_.points[end - 1];

        double traj_length = distance_to_traj(0, goal.x, goal.y);
        lookahead_gain_ = std::min(traj_length / lookahead_distance_, 1.0);

        visualization_.visualize_goal(goal.x, goal.y, goal.yaw);
        std::cout << tracking_object_ << " - Approached, changing goal to: (" << goal.x << ", " << goal.y << ')' << std::endl;
      }
    }
  }

  double longitude_controller() {
    double time = ros::Time::now().toSec() - start_time_;
    TrajectoryPoint truth = trajectory_.points.back();
    auto upper_point = std::lower_bound(trajectory_.points.begin(), trajectory_.points.end(), time, [](const TrajectoryPoint &tp, double t) {
      return tp.time < t;
    });

    if(upper_point != trajectory_.points.end()) {
      truth = *upper_point;
      if(upper_point > trajectory_.points.begin()) {
        truth = interpolateWithTime(*std::prev(upper_point), *upper_point, time);
      }
    }

    visualization_.visualize_pose(truth.x, truth.y, truth.yaw);

    double sinx = sin(truth.yaw), cosx = cos(truth.yaw);
    double relative_x = cosx * (truth.x - object_.x) + sinx * (truth.y - object_.y);

    // feedforward + proportional feedback (replaces incremental PI)
    double ff = truth.velocity;                     // feedforward: planned velocity
    double fb = longitude_kp_ * relative_x;         // feedback: proportional on position error
    longitude_output_ = ff + fb;

    longitude_output_ = std::min(std::max(longitude_output_, 0.0), 0.25);  // forward only
    return longitude_output_;
  }

  void controller() {
    TrajectoryPoint &lookahead = trajectory_.points[lookahead_index_];
    auto lookahead_distance = distance_to_traj(lookahead_index_, object_.x, object_.y);

    double sinx = sin(object_.yaw), cosx = cos(object_.yaw);
    double relative_x = cosx * (lookahead.x - object_.x) + sinx * (lookahead.y - object_.y);
    double relative_y = -sinx * (lookahead.x - object_.x) + cosx * (lookahead.y - object_.y);
    double eta = atan2(relative_y, relative_x);

    // feedforward steering from trajectory curvature (2DOF: ff + fb)
    double ff_steer = 0.0;
    if (car_index_ > 0 && car_index_ < (int)trajectory_.points.size() - 1) {
      double dx = trajectory_.points[car_index_ + 1].x - trajectory_.points[car_index_ - 1].x;
      double dy = trajectory_.points[car_index_ + 1].y - trajectory_.points[car_index_ - 1].y;
      double ds = sqrt(dx * dx + dy * dy);
      if (ds > 0.001) {
        double dyaw = trajectory_.points[car_index_ + 1].yaw - trajectory_.points[car_index_ - 1].yaw;
        while (dyaw > M_PI) dyaw -= 2 * M_PI;
        while (dyaw < -M_PI) dyaw += 2 * M_PI;
        double kappa = dyaw / ds;
        ff_steer = atan(wheel_base_ * kappa);
      }
    }

    // feedback: lateral error + heading error (replaces PP geometric formula)
    // compute signed lateral error to nearest trajectory point
    double nearest_dx = trajectory_.points[car_index_].x - object_.x;
    double nearest_dy = trajectory_.points[car_index_].y - object_.y;
    // lateral error: perpendicular distance (positive = car is to the right of path)
    double path_yaw = trajectory_.points[car_index_].yaw;
    double e_lat = -sin(path_yaw) * nearest_dx + cos(path_yaw) * nearest_dy;
    // heading error
    double e_yaw = path_yaw - object_.yaw;
    while (e_yaw > M_PI) e_yaw -= 2*M_PI;
    while (e_yaw < -M_PI) e_yaw += 2*M_PI;

    // 2DOF: feedforward + lateral/heading feedback（增益从 ROS 参数读取）
    double delta = ff_steer + K_e_ * e_lat + K_yaw_ * e_yaw;
    double velocity = longitude_controller();

//    ROS_INFO("[%d] velocity: %f", tracking_object_, velocity);
//    ROS_INFO("PP: %f, %f, %f", eta, lookahead_distance, delta);

    ChassisCommand command;
    command.target = tracking_object_;
    command.throttle = velocity;
    command.steering = std::min(max_steering_, std::max(min_steering_, delta));
//    for(auto &cmd : last_commands_) {
//      command.throttle += cmd.throttle;
//      command.steering += cmd.steering;
//    }
//    command.throttle /= last_commands_.size() + 1;
//    command.steering /= last_commands_.size() + 1;
//
//    if(last_commands_.size() >= 3) {
//      last_commands_.pop_front();
//    }
//    last_commands_.push_back(command);

    command_publisher_.publish(command);

    // Log
    if(pp_log_.is_open()) {
      double elapsed = ros::Time::now().toSec() - start_time_;
      int end = ranges_[current_range_].second;
      double approach_dist = distance_to_traj(end-1, object_.x, object_.y);
      pp_log_ << elapsed << ","
              << object_.x << "," << object_.y << "," << object_.yaw << ","
              << lookahead.x << "," << lookahead.y << "," << lookahead.yaw << "," << lookahead.velocity << ","
              << car_index_ << "," << lookahead_index_ << ","
              << relative_x << "," << command.throttle << "," << command.steering << ","
              << approach_dist << "," << approached_ << "," << elapsed << std::endl;
    }
  }

  void control_callback(const ros::TimerEvent &evt) {
    // Debug: log every callback
    if(pp_log_.is_open()) {
      double elapsed = (start_time_ > 0) ? ros::Time::now().toSec() - start_time_ : -1;
      pp_log_ << elapsed << ","
              << object_.x << "," << object_.y << "," << object_.yaw << ","
              << 0 << "," << 0 << "," << 0 << "," << 0 << ","
              << car_index_ << "," << lookahead_index_ << ","
              << 0 << "," << 0 << "," << 0 << ","
              << 0 << "," << approached_ << ","
              << "traj_sz=" << trajectory_.points.size()
              << std::endl;
    }
    if(!trajectory_.points.empty() && !approached_) {
      switch_range();
      update_lookahead();
      controller();
    } else {
      ChassisCommand command;
      command.target = tracking_object_;
      command.throttle = 0.0;
      command.steering = 0.0;

      command_publisher_.publish(command);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit");
  ros::NodeHandle nh("~");

  PurePursuit pp(nh);
  ros::spin();

  return 0;
}
