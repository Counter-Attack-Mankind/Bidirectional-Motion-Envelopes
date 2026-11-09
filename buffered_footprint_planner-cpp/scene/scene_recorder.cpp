#include "scene/scene_recorder.h"

#include <algorithm>
#include <filesystem>
#include <regex>

#include "scene/scene_storage.h"

namespace scene {

SceneRecorder::SceneRecorder(std::string tasks_root)
    : tasks_root_(std::move(tasks_root)) {}

bool SceneRecorder::SaveNext(SceneSnapshot snapshot,
                             const VehicleParameter& vehicle,
                             std::string* saved_file,
                             std::string* error) const {
  namespace fs = std::filesystem;
  std::error_code code;
  fs::create_directories(tasks_root_, code);
  if (code) {
    if (error) *error = "Cannot create tasks directory: " + code.message();
    return false;
  }
  int next_id = 1;
  const std::regex pattern("task([0-9]+)");
  for (const auto& entry : fs::directory_iterator(tasks_root_, code)) {
    if (code) break;
    if (!entry.is_directory()) continue;
    std::smatch match;
    const std::string name = entry.path().filename().string();
    if (std::regex_match(name, match, pattern)) {
      next_id = std::max(next_id, std::stoi(match[1].str()) + 1);
    }
  }
  if (code) {
    if (error) *error = "Cannot enumerate tasks directory: " + code.message();
    return false;
  }
  snapshot.task_id = next_id;
  if (!snapshot.Validate(vehicle, error)) return false;
  fs::path directory;
  while (true) {
    directory = fs::path(tasks_root_) / ("task" + std::to_string(next_id));
    if (fs::create_directory(directory, code)) break;
    if (code) {
      if (error) *error = "Cannot reserve task directory: " + code.message();
      return false;
    }
    snapshot.task_id = ++next_id;
  }
  const fs::path file = directory / "scene.yaml";
  if (!SceneStorage::Save(file.string(), snapshot, error)) return false;
  if (saved_file) *saved_file = file.string();
  return true;
}

}  // namespace scene
