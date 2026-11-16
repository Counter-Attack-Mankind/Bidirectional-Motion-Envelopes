#pragma once

#include <string>
#include <vector>

#include "common/math/polygon2d.h"
#include "planning/hybrid_astar/hybrid_a_star.h"
#include "planning/hybrid_astar/parking_planner_config.h"
#include "planning/hybrid_astar/vehicle_parameter.h"
#include "scene/scene_snapshot.h"

namespace planning {

enum class HybridSearchDirection { kStartToGoal, kGoalToStart };

bool ParseHybridSearchDirection(const std::string& value,
                                HybridSearchDirection* direction);
const char* HybridSearchDirectionName(HybridSearchDirection direction);

ParkingPlannerConfig AdaptHybridSearchConfig(
    const ParkingPlannerConfig& configured,
    HybridSearchDirection direction);

struct HybridSearchAdapterResult {
  bool succeeded = false;
  bool timed_out = false;
  bool steering_recomputed = false;
  double timeout_seconds = 0.0;
  std::size_t gear_switch_count = 0;
  std::string error;
  ParkingPlannerConfig effective_config;
  HybridAStarStatistics statistics;
  HybridAStartResult path;
};

class HybridSearchAdapter {
 public:
  HybridSearchAdapter(HybridSearchDirection direction,
                      const ParkingPlannerConfig& configured,
                      const VehicleParameter& vehicle,
                      const std::vector<common::math::Polygon2d>& obstacles,
                      bool save_debug_data);

  HybridSearchAdapterResult Search(
      const scene::Pose2d& actual_start, const scene::Pose2d& actual_goal,
      HybridAStar::ProgressCallback progress_callback = {},
      HybridAStar::CancellationCallback cancellation_callback = {}) const;

 private:
  HybridSearchDirection direction_;
  ParkingPlannerConfig configured_;
  VehicleParameter vehicle_;
  std::vector<common::math::Polygon2d> obstacles_;
  bool save_debug_data_ = false;
};

}  // namespace planning
