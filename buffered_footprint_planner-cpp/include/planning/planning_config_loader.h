#pragma once

#include <array>
#include <string>

#include <ros/ros.h>

#include "planning/hybrid_astar/parking_planner_config.h"
#include "planning/hybrid_astar/vehicle_parameter.h"
#include "optimizer/footprint_nlp.h"
#include "optimizer/initial_guess.h"

namespace planning_platform {

struct PlanningConfiguration {
  VehicleParameter vehicle;
  ParkingPlannerConfig planner;
  optimizer::InitialGuessConfig initial_guess;
  optimizer::FootprintNlpConfig nlp;
  std::string results_root = "results";
};

bool LoadVehicleParameter(const ros::NodeHandle& node,
                          VehicleParameter* vehicle, std::string* error);

bool LoadPlanningConfiguration(const ros::NodeHandle& node,
                               const std::array<double, 4>& bounds,
                               PlanningConfiguration* configuration,
                               std::string* error);

}  // namespace planning_platform
