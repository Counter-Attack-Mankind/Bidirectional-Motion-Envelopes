#include "execution/paper_data_exporter.h"

#include <filesystem>
#include <iomanip>

namespace execution {

bool PaperDataExporter::StartTask(
    const std::string& result_directory, const scene::SceneSnapshot& snapshot,
    const optimizer::Trajectory* open_loop, const std::string& planning_status,
    std::string* error) {
  FinishTask("superseded", nullptr);
  namespace fs = std::filesystem;
  paper_directory_ = (fs::path(result_directory) / "paper").string();
  std::error_code code;
  fs::create_directories(paper_directory_, code);
  if (code) {
    if (error) *error = "Cannot create paper data directory: " + code.message();
    return false;
  }
  snapshot_ = snapshot;
  std::ofstream path(fs::path(paper_directory_) / "open_loop_path.csv");
  std::ofstream obstacles(fs::path(paper_directory_) / "obstacles.csv");
  closed_loop_.open(fs::path(paper_directory_) / "closed_loop_path.csv");
  if (!path || !obstacles || !closed_loop_) {
    if (error) *error = "Cannot create paper CSV files in " + paper_directory_;
    closed_loop_.close();
    return false;
  }
  path << std::setprecision(17) << "index,x_m,y_m\n";
  if (open_loop) {
    for (std::size_t i = 0; i < open_loop->nodes.size(); ++i)
      path << i << ',' << open_loop->nodes[i].x << ','
           << open_loop->nodes[i].y << '\n';
  }
  obstacles << std::setprecision(17)
            << "obstacle_id,center_x_m,center_y_m,yaw_rad,length_m,width_m,"
               "vertex1_x_m,vertex1_y_m,vertex2_x_m,vertex2_y_m,"
               "vertex3_x_m,vertex3_y_m,vertex4_x_m,vertex4_y_m\n";
  for (const auto& obstacle : snapshot.obstacles) {
    obstacles << obstacle.id << ',' << obstacle.x << ',' << obstacle.y << ','
              << obstacle.yaw << ',' << obstacle.length << ',' << obstacle.width;
    const auto polygon = obstacle.polygon();
    for (const auto& point : polygon.points())
      obstacles << ',' << point.x() << ',' << point.y();
    obstacles << '\n';
  }
  closed_loop_ << std::setprecision(17) << "index,x_m,y_m\n";
  closed_loop_index_ = 0;
  active_ = true;
  WriteStartGoal(planning_status, nullptr);
  return true;
}

void PaperDataExporter::LogClosedLoop(const scene::Pose2d& pose) {
  if (!active_ || !closed_loop_) return;
  closed_loop_ << closed_loop_index_++ << ',' << pose.x << ',' << pose.y << '\n';
  closed_loop_.flush();
}

void PaperDataExporter::WriteStartGoal(
    const std::string& status, const scene::Pose2d* actual_stop) {
  if (paper_directory_.empty()) return;
  std::ofstream out(std::filesystem::path(paper_directory_) / "start_goal.csv");
  if (!out) return;
  out << std::setprecision(17)
      << "role,x_m,y_m,theta_rad,status\n"
      << "start," << snapshot_.start.x << ',' << snapshot_.start.y << ','
      << snapshot_.start.yaw << ",recorded\n"
      << "goal," << snapshot_.goal.x << ',' << snapshot_.goal.y << ','
      << snapshot_.goal.yaw << ",recorded\n";
  if (actual_stop) {
    out << "actual_stop," << actual_stop->x << ',' << actual_stop->y << ','
        << actual_stop->yaw << ',' << status << '\n';
  } else {
    out << "actual_stop,,,," << status << '\n';
  }
}

void PaperDataExporter::FinishTask(const std::string& status,
                                   const scene::Pose2d* actual_stop) {
  if (!active_) return;
  if (closed_loop_) closed_loop_.close();
  WriteStartGoal(status, actual_stop);
  active_ = false;
}

}  // namespace execution
