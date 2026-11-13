#pragma once

#include <string>

#include "scene/scene_snapshot.h"

namespace scene {

class SceneRecorder {
 public:
  explicit SceneRecorder(std::string tasks_root);

  bool SaveNext(SceneSnapshot snapshot, const VehicleParameter& vehicle,
                std::string* saved_file, std::string* error) const;

 private:
  std::string tasks_root_;
};

}  // namespace scene
