#pragma once

#include <fstream>
#include <string>

#include "optimizer/trajectory.h"
#include "scene/scene_snapshot.h"

namespace execution {

// Produces the compact paper-facing view without replacing detailed logs.
class PaperDataExporter {
 public:
  bool StartTask(const std::string& result_directory,
                 const scene::SceneSnapshot& snapshot,
                 const optimizer::Trajectory* open_loop,
                 const std::string& planning_status, std::string* error);
  void LogClosedLoop(const scene::Pose2d& pose);
  void FinishTask(const std::string& status,
                  const scene::Pose2d* actual_stop);
  bool active() const { return active_; }

 private:
  void WriteStartGoal(const std::string& status,
                      const scene::Pose2d* actual_stop);

  std::string paper_directory_;
  scene::SceneSnapshot snapshot_;
  std::ofstream closed_loop_;
  std::size_t closed_loop_index_ = 0;
  bool active_ = false;
};

}  // namespace execution
