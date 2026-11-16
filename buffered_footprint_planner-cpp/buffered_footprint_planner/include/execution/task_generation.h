#pragma once

#include <cstdint>
#include <algorithm>
#include <exception>
#include <filesystem>
#include <string>

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

inline std::uint64_t FindMaximumTaskId(
    const std::filesystem::path& results_root) {
  const std::filesystem::path real_root = results_root / "real_vehicle";
  std::error_code error;
  if (!std::filesystem::is_directory(real_root, error)) return 0;
  std::uint64_t maximum = 0;
  for (const auto& entry : std::filesystem::directory_iterator(real_root, error)) {
    if (error) break;
    if (!entry.is_directory(error)) continue;
    const std::string name = entry.path().filename().string();
    if (name.size() <= 4 || name.compare(0, 4, "task") != 0) continue;
    try {
      std::size_t consumed = 0;
      const std::uint64_t value = static_cast<std::uint64_t>(
          std::stoull(name.substr(4), &consumed));
      if (consumed == name.size() - 4) maximum = std::max(maximum, value);
    } catch (const std::exception&) {
    }
  }
  return maximum;
}

}  // namespace execution
