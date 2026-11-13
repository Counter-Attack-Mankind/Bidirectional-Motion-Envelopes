#pragma once

#include <cstdint>

namespace execution {

// Pure predicate shared by async planning consumers and regression tests.
inline bool IsCurrentTask(std::uint64_t event_generation,
                          int event_vehicle_id,
                          std::uint64_t active_generation,
                          int planned_vehicle_id,
                          int observed_vehicle_id,
                          bool planning_state_active) {
  return planning_state_active && event_generation == active_generation &&
         event_vehicle_id == planned_vehicle_id &&
         event_vehicle_id == observed_vehicle_id;
}

}  // namespace execution
