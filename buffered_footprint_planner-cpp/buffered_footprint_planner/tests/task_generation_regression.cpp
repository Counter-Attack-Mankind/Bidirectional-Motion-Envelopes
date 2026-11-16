#include <cassert>
#include <filesystem>

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
  std::filesystem::remove_all(root, error);
  return 0;
}
