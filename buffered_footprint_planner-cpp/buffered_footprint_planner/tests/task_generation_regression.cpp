#include <cassert>

#include "execution/task_generation.h"

int main() {
  assert(execution::IsCurrentTask(8, 2, 8, 2, 2, true));
  assert(!execution::IsCurrentTask(7, 2, 8, 2, 2, true));
  assert(!execution::IsCurrentTask(8, 1, 8, 2, 2, true));
  assert(!execution::IsCurrentTask(8, 2, 8, 2, 3, true));
  assert(!execution::IsCurrentTask(8, 2, 8, 2, 2, false));
  return 0;
}
