#pragma once

#include <cstdint>
#include <string>

#include <sandbox_msgs/Trajectory.h>

#include "optimizer/trajectory.h"

namespace execution {

class TrajectoryAdapter {
 public:
  static bool Validate(const optimizer::Trajectory& trajectory,
                       std::string* error);
  static bool Convert(const optimizer::Trajectory& trajectory,
                      int vehicle_id, std::uint32_t task_id,
                      sandbox_msgs::Trajectory* message,
                      std::string* error);
};

}  // namespace execution
