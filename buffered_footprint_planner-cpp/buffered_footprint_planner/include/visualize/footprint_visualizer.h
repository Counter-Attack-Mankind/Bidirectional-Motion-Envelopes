#pragma once

#include <cstddef>
#include <string>
#include <utility>

#include "optimizer/trajectory.h"

namespace visualization {

class FootprintVisualizer {
 public:
  explicit FootprintVisualizer(std::string namespace_prefix = "trajectory")
      : namespace_prefix_(std::move(namespace_prefix)) {}
  void Update(const optimizer::FootprintInterval& interval,
              std::size_t interval_index, bool force = false);
  void Clear();

 private:
  std::size_t last_interval_ = static_cast<std::size_t>(-1);
  std::string namespace_prefix_;
};

}  // namespace visualization
