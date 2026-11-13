#include "scene/scene_storage.h"

#include <filesystem>
#include <fstream>

#include <yaml-cpp/yaml.h>

namespace scene {
namespace {

bool ReadPose(const YAML::Node& value, Pose2d* pose) {
  if (!value || !value.IsMap()) return false;
  try {
    pose->x = value["x"].as<double>();
    pose->y = value["y"].as<double>();
    pose->yaw = value["yaw"].as<double>();
    return true;
  } catch (const YAML::Exception&) {
    return false;
  }
}

YAML::Node WritePose(const Pose2d& pose) {
  YAML::Node value;
  value["x"] = pose.x;
  value["y"] = pose.y;
  value["yaw"] = pose.yaw;
  return value;
}

}  // namespace

bool SceneStorage::Save(const std::string& file, const SceneSnapshot& snapshot,
                        std::string* error) {
  try {
    YAML::Node root;
    root["task_id"] = snapshot.task_id;
    root["frame_id"] = snapshot.frame_id;
    root["map"]["x_min"] = snapshot.bounds[0];
    root["map"]["x_max"] = snapshot.bounds[1];
    root["map"]["y_min"] = snapshot.bounds[2];
    root["map"]["y_max"] = snapshot.bounds[3];
    root["start"] = WritePose(snapshot.start);
    root["goal"] = WritePose(snapshot.goal);
    root["obstacles"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& obstacle : snapshot.obstacles) {
      YAML::Node value;
      value["id"] = obstacle.id;
      value["x"] = obstacle.x;
      value["y"] = obstacle.y;
      value["yaw"] = obstacle.yaw;
      value["length"] = obstacle.length;
      value["width"] = obstacle.width;
      root["obstacles"].push_back(value);
    }
    std::ofstream stream(file);
    if (!stream) {
      if (error) *error = "Cannot create scene file '" + file + "'";
      return false;
    }
    YAML::Emitter emitter;
    emitter.SetDoublePrecision(17);
    emitter << root;
    stream << emitter.c_str() << '\n';
    if (!stream) {
      if (error) *error = "Failed while writing scene file '" + file + "'";
      return false;
    }
    return true;
  } catch (const std::exception& exception) {
    if (error) *error = exception.what();
    return false;
  }
}

bool SceneStorage::Load(const std::string& file, SceneSnapshot* snapshot,
                        std::string* error) {
  if (!snapshot) {
    if (error) *error = "Scene output pointer is null";
    return false;
  }
  try {
    const YAML::Node root = YAML::LoadFile(file);
    SceneSnapshot value;
    value.task_id = root["task_id"].as<int>();
    value.frame_id = root["frame_id"].as<std::string>();
    value.bounds = {{root["map"]["x_min"].as<double>(),
                     root["map"]["x_max"].as<double>(),
                     root["map"]["y_min"].as<double>(),
                     root["map"]["y_max"].as<double>()}};
    if (!ReadPose(root["start"], &value.start) ||
        !ReadPose(root["goal"], &value.goal) ||
        !root["obstacles"] || !root["obstacles"].IsSequence()) {
      if (error) *error = "Scene pose or obstacle structure is invalid";
      return false;
    }
    for (const auto& item : root["obstacles"]) {
      SceneObstacle obstacle;
      obstacle.id = item["id"].as<std::string>();
      obstacle.x = item["x"].as<double>();
      obstacle.y = item["y"].as<double>();
      obstacle.yaw = item["yaw"].as<double>();
      obstacle.length = item["length"].as<double>();
      obstacle.width = item["width"].as<double>();
      value.obstacles.push_back(obstacle);
    }
    *snapshot = std::move(value);
    return true;
  } catch (const std::exception& exception) {
    if (error) *error = "Cannot load scene '" + file + "': " + exception.what();
    return false;
  }
}

}  // namespace scene
