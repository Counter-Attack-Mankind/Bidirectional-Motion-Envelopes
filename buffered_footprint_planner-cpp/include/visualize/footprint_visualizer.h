#pragma once

#include <cstddef>

#include "optimizer/trajectory.h"

namespace visualization {

class FootprintVisualizer {
 public:
  void Update(const optimizer::FootprintInterval& interval,
              std::size_t interval_index, bool force = false);
  void Clear();

 private:
  std::size_t last_interval_ = static_cast<std::size_t>(-1);
};

}  // namespace visualization
