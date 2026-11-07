#include "environment/my_env.h"

#include <fstream>
#include <iomanip>
#include <utility>

#include <ros/ros.h>

bool MyEnvironment::Read(const std::string& env_file) {
  std::ifstream input(env_file);
  if (!input.is_open()) {
    ROS_ERROR("Cannot open environment file: %s", env_file.c_str());
    return false;
  }

  try {
    json document;
    input >> document;
    MyEnvironment parsed = document.get<MyEnvironment>();
    *this = std::move(parsed);
  } catch (const std::exception& error) {
    ROS_ERROR("Invalid environment file '%s': %s", env_file.c_str(), error.what());
    return false;
  }

  ROS_INFO("Loaded environment: %zu border points, %zu obstacles, start=%s, goal=%s",
           border.size(), obstacles.size(), has_start ? "yes" : "no", has_goal ? "yes" : "no");
  return true;
}

bool MyEnvironment::Save(const std::string& env_file) const {
  std::ofstream output(env_file);
  if (!output.is_open()) {
    ROS_ERROR("Cannot write environment file: %s", env_file.c_str());
    return false;
  }
  output << std::setw(2) << json(*this) << '\n';
  return output.good();
}
