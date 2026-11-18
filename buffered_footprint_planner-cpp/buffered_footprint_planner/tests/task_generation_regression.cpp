#include <cassert>
#include <filesystem>
#include <fstream>

#include "execution/task_generation.h"

int main() {
  assert(execution::IsCurrentTask(8, 2, 8, 2, 2, true));
  assert(!execution::IsCurrentTask(7, 2, 8, 2, 2, true));
  assert(!execution::IsCurrentTask(8, 1, 8, 2, 2, true));
  assert(!execution::IsCurrentTask(8, 2, 8, 2, 3, true));
  assert(!execution::IsCurrentTask(8, 2, 8, 2, 2, false));
  const auto root = std::filesystem::temp_directory_path() /
                    "bfp_task_generation_regression";
  std::error_code error;
  std::filesystem::remove_all(root, error);
  std::filesystem::create_directories(root / "real_vehicle" / "task2");
  std::filesystem::create_directories(root / "real_vehicle" / "task17");
  std::filesystem::create_directories(root / "real_vehicle" / "task_bad");
  std::filesystem::create_directories(root / "real_vehicle" / "task18_extra");
  assert(execution::FindMaximumTaskId(root) == 17);
  std::filesystem::create_directories(
      root / "real_vehicle" / "stage3" / "task1");
  {
    std::ofstream summary(root / "real_vehicle" / "stage3" / "task1" /
                          "summary.txt");
    summary << "task_id=24\nsession_id=3\nsession_task_id=1\n";
  }
  assert(execution::FindMaximumSessionId(root) == 3);
  assert(execution::FindMaximumTaskId(root) == 24);
  std::uint64_t session_id = 0;
  std::string session_directory;
  std::string message;
  assert(execution::CreateRealVehicleSession(
      root, &session_id, &session_directory, &message));
  assert(session_id == 4);
  assert(std::filesystem::is_directory(session_directory));
  assert(execution::RecordAllocatedTaskId(root, 25, &message));
  assert(execution::FindMaximumTaskId(root) == 25);
  std::filesystem::remove_all(root, error);
  return 0;
}
