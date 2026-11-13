#include <cmath>

#include <ros/ros.h>
#include <sandbox_msgs/ChassisCommand.h>
#include <std_msgs/Bool.h>

class ChassisCommandGuard {
 public:
  ChassisCommandGuard() : private_node_("~") {
    private_node_.param("command_timeout", command_timeout_, 0.30);
    input_ = node_.subscribe("/chassis_raw", 1,
                             &ChassisCommandGuard::CommandCallback, this);
    estop_ = node_.subscribe("/estop", 1,
                             &ChassisCommandGuard::EstopCallback, this);
    output_ = node_.advertise<sandbox_msgs::ChassisCommand>("/chassis", 1,
                                                            true);
    timer_ = node_.createWallTimer(ros::WallDuration(0.05),
                                   &ChassisCommandGuard::Watchdog, this);
    PublishZero();
  }

 private:
  void CommandCallback(const sandbox_msgs::ChassisCommandConstPtr& message) {
    if (!message || !std::isfinite(message->throttle) ||
        !std::isfinite(message->steering)) {
      PublishZero();
      return;
    }
    target_ = message->target;
    last_command_ = ros::WallTime::now();
    command_seen_ = true;
    if (estop_active_) {
      PublishZero();
      return;
    }
    output_.publish(*message);
  }

  void EstopCallback(const std_msgs::BoolConstPtr& message) {
    estop_active_ = !message || message->data;
    if (estop_active_) PublishZero();
  }

  void Watchdog(const ros::WallTimerEvent&) {
    if (!command_seen_ ||
        (ros::WallTime::now() - last_command_).toSec() > command_timeout_) {
      PublishZero();
      command_seen_ = false;
    }
  }

  void PublishZero() {
    sandbox_msgs::ChassisCommand command;
    command.target = target_;
    command.throttle = 0.0;
    command.steering = 0.0;
    output_.publish(command);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber input_;
  ros::Subscriber estop_;
  ros::Publisher output_;
  ros::WallTimer timer_;
  ros::WallTime last_command_;
  double command_timeout_ = 0.30;
  int target_ = 0;
  bool command_seen_ = false;
  bool estop_active_ = true;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "chassis_command_guard");
  ChassisCommandGuard guard;
  ros::spin();
  return 0;
}
