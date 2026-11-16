#include <cassert>
#include <filesystem>
#include <fstream>
#include <string>

#include "execution/paper_data_exporter.h"
#include "planning/experiment_logger.h"

int main() {
  namespace fs = std::filesystem;
  const fs::path root = fs::temp_directory_path() /
                        "bfp_paper_data_exporter_regression";
  std::error_code code;
  fs::remove_all(root, code);
  fs::create_directories(root);

  experiment::ExperimentLogger run_logger;
  std::string run_directory;
  std::string error;
  assert(run_logger.CreateRunDirectory(root.string(), "real_vehicle", 7,
                                       &run_directory, &error));
  assert(fs::path(run_directory) == root / "real_vehicle" / "task7");
  assert(!run_logger.CreateRunDirectory(root.string(), "real_vehicle", 7,
                                        &run_directory, &error));

  scene::SceneSnapshot scene;
  scene.start = {0.1, 0.2, 0.3};
  scene.goal = {1.1, 1.2, 1.3};
  scene.obstacles.push_back({"box_1", 0.5, 0.6, 0.2, 0.3, 0.1});
  optimizer::Trajectory trajectory;
  optimizer::TrajectoryNode first;
  first.x = 0.1; first.y = 0.2;
  optimizer::TrajectoryNode second;
  second.x = 0.4; second.y = 0.5;
  trajectory.nodes = {first, second};

  execution::PaperDataExporter exporter;
  assert(exporter.StartTask((root / "real_vehicle" / "task7").string(),
                            scene, &trajectory,
                            "planned", &error));
  exporter.LogClosedLoop(scene.start);
  const scene::Pose2d stop{1.0, 1.1, 1.2};
  exporter.FinishTask("reached", &stop);

  for (const char* name : {"open_loop_path.csv", "closed_loop_path.csv",
                           "start_goal.csv", "obstacles.csv"}) {
    assert(fs::is_regular_file(root / "real_vehicle" / "task7" / "paper" / name));
    assert(fs::file_size(root / "real_vehicle" / "task7" / "paper" / name) > 0);
  }
  std::ifstream start_goal(root / "real_vehicle" / "task7" / "paper" /
                           "start_goal.csv");
  const std::string contents((std::istreambuf_iterator<char>(start_goal)),
                             std::istreambuf_iterator<char>());
  assert(contents.find("actual_stop,1,") !=
         std::string::npos);
  fs::remove_all(root, code);
  return 0;
}
