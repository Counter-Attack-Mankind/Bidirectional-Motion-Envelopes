#pragma once

#include <fstream>
#include <cstdint>
#include <string>

#include <sandbox_msgs/ChassisCommand.h>

#include "scene/scene_snapshot.h"

namespace execution {

class RealVehicleLogger {
 public:
  bool Start(const std::string& result_directory, std::uint64_t task_id,
             int vehicle_id, std::string* error);
  void LogPose(double stamp, const scene::Pose2d& pose);
  void LogCommand(double stamp, const sandbox_msgs::ChassisCommand& command);
  void Stop();

 private:
  std::ofstream tracking_;
  std::ofstream commands_;
  std::uint64_t task_id_ = 0;
  int vehicle_id_ = -1;
};

}  // namespace execution
