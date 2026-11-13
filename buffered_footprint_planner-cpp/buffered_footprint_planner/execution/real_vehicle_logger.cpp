#include "execution/real_vehicle_logger.h"

#include <iomanip>

namespace execution {

bool RealVehicleLogger::Start(const std::string& result_directory,
                              std::uint64_t task_id, int vehicle_id,
                              std::string* error) {
  Stop();
  tracking_.open(result_directory + "/actual_tracking.csv");
  commands_.open(result_directory + "/control_commands.csv");
  if (!tracking_ || !commands_) {
    if (error) *error = "cannot create real-vehicle CSV logs in " + result_directory;
    Stop();
    return false;
  }
  tracking_ << "stamp,task_id,vehicle_id,x,y,yaw\n";
  commands_ << "stamp,task_id,vehicle_id,target,throttle,steering\n";
  tracking_ << std::setprecision(15);
  commands_ << std::setprecision(15);
  task_id_ = task_id;
  vehicle_id_ = vehicle_id;
  return true;
}

void RealVehicleLogger::LogPose(double stamp, const scene::Pose2d& pose) {
  if (tracking_) tracking_ << stamp << ',' << task_id_ << ',' << vehicle_id_
                           << ',' << pose.x << ',' << pose.y << ',' << pose.yaw
                           << '\n';
}

void RealVehicleLogger::LogCommand(
    double stamp, const sandbox_msgs::ChassisCommand& command) {
  if (commands_) commands_ << stamp << ',' << task_id_ << ',' << vehicle_id_
                           << ',' << command.target << ',' << command.throttle
                           << ',' << command.steering << '\n';
}

void RealVehicleLogger::Stop() {
  if (tracking_) tracking_.close();
  if (commands_) commands_.close();
}

}  // namespace execution
