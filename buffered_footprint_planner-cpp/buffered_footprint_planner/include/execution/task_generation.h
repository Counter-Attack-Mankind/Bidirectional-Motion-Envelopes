#pragma once

#include <algorithm>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
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
  {
    std::ifstream sequence(real_root / "ros_task_sequence.txt");
    std::uint64_t persisted = 0;
    if (sequence >> persisted) maximum = persisted;
  }
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
  // New staged layouts keep the globally unique ROS task id in summary.txt;
  // stage/task directory numbers are deliberately local presentation ids.
  error.clear();
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(real_root, error)) {
    if (error) break;
    if (!entry.is_regular_file(error) ||
        entry.path().filename() != "summary.txt") continue;
    std::ifstream summary(entry.path());
    std::string line;
    while (std::getline(summary, line)) {
      constexpr char prefix[] = "task_id=";
      if (line.compare(0, sizeof(prefix) - 1, prefix) != 0) continue;
      try {
        std::size_t consumed = 0;
        const std::string value_text = line.substr(sizeof(prefix) - 1);
        const std::uint64_t value = static_cast<std::uint64_t>(
            std::stoull(value_text, &consumed));
        if (consumed == value_text.size()) maximum = std::max(maximum, value);
      } catch (const std::exception&) {
      }
      break;
    }
  }
  return maximum;
}

inline bool RecordAllocatedTaskId(
    const std::filesystem::path& results_root, std::uint64_t task_id,
    std::string* error_message) {
  const std::filesystem::path real_root = results_root / "real_vehicle";
  std::error_code error;
  std::filesystem::create_directories(real_root, error);
  if (error) {
    if (error_message) *error_message =
        "Cannot create real-vehicle results root: " + error.message();
    return false;
  }
  // Never move the high-water mark backwards if an older caller is still
  // draining an asynchronous result.
  const std::uint64_t persisted_task_id =
      std::max(task_id, FindMaximumTaskId(results_root));
  std::ofstream sequence(real_root / "ros_task_sequence.txt", std::ios::trunc);
  if (!sequence || !(sequence << persisted_task_id << '\n')) {
    if (error_message) *error_message =
        "Cannot persist global ROS task sequence";
    return false;
  }
  return true;
}

inline std::uint64_t FindMaximumSessionId(
    const std::filesystem::path& results_root) {
  const std::filesystem::path real_root = results_root / "real_vehicle";
  std::error_code error;
  if (!std::filesystem::is_directory(real_root, error)) return 0;
  std::uint64_t maximum = 0;
  for (const auto& entry : std::filesystem::directory_iterator(real_root, error)) {
    if (error) break;
    if (!entry.is_directory(error)) continue;
    const std::string name = entry.path().filename().string();
    if (name.size() <= 5 || name.compare(0, 5, "stage") != 0) continue;
    try {
      std::size_t consumed = 0;
      const std::uint64_t value = static_cast<std::uint64_t>(
          std::stoull(name.substr(5), &consumed));
      if (consumed == name.size() - 5) maximum = std::max(maximum, value);
    } catch (const std::exception&) {
    }
  }
  return maximum;
}

inline bool CreateRealVehicleSession(
    const std::filesystem::path& results_root, std::uint64_t* session_id,
    std::string* session_directory, std::string* error_message) {
  const std::filesystem::path real_root = results_root / "real_vehicle";
  std::error_code error;
  std::filesystem::create_directories(real_root, error);
  if (error) {
    if (error_message) *error_message =
        "Cannot create real-vehicle results root: " + error.message();
    return false;
  }
  std::uint64_t candidate_id = FindMaximumSessionId(results_root) + 1;
  for (;;) {
    const auto candidate = real_root / ("stage" + std::to_string(candidate_id));
    error.clear();
    if (std::filesystem::create_directory(candidate, error)) {
      if (session_id) *session_id = candidate_id;
      if (session_directory) *session_directory = candidate.string();
      return true;
    }
    if (error) {
      if (error_message) *error_message =
          "Cannot create real-vehicle stage directory: " + error.message();
      return false;
    }
    ++candidate_id;  // Another launch may have allocated the candidate.
  }
}

}  // namespace execution
