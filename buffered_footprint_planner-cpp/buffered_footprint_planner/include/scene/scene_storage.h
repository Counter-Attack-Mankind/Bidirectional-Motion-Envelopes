#pragma once

#include <string>

#include "scene/scene_snapshot.h"

namespace scene {

class SceneStorage {
 public:
  static bool Save(const std::string& file, const SceneSnapshot& snapshot,
                   std::string* error);
  static bool Load(const std::string& file, SceneSnapshot* snapshot,
                   std::string* error);
};

}  // namespace scene
