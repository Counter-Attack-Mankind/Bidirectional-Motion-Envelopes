#include "visualize/footprint_visualizer.h"

#include <iomanip>
#include <sstream>
#include <vector>

#include "common/util/color.h"
#include "visualize/visualization_plot.h"

namespace visualization {

void FootprintVisualizer::Update(
    const optimizer::FootprintInterval& interval,
    std::size_t interval_index, bool force) {
  if (!force && interval_index == last_interval_) return;
  last_interval_ = interval_index;

  std::vector<double> x;
  std::vector<double> y;
  x.reserve(interval.corners.size());
  y.reserve(interval.corners.size());
  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto& corner : interval.corners) {
    x.push_back(corner.x());
    y.push_back(corner.y());
    center_x += corner.x();
    center_y += corner.y();
  }
  center_x /= static_cast<double>(interval.corners.size());
  center_y /= static_cast<double>(interval.corners.size());

  Color outline = interval.gear ? Color::Blue : Color::Red;
  Color fill = outline;
  fill.set_a(0.28);
  VisualizationPlot::PlotFilledPolygon(
      x, y, fill, 0, "trajectory_footprint_fill", 0.022);
  VisualizationPlot::PlotPolygon(
      x, y, 0.012, outline, 0, "trajectory_footprint_outline");

  std::ostringstream text;
  text << std::fixed << std::setprecision(3)
       << "EF[" << interval_index << "]"
       << " up=" << interval.up << " down=" << interval.down
       << " left=" << interval.left << " right=" << interval.right;
  VisualizationPlot::PlotText(
      center_x, center_y + 0.16, text.str(), 0.055, outline, 0,
      "trajectory_footprint_scales");
}

void FootprintVisualizer::Clear() {
  VisualizationPlot::ClearMarkers("trajectory_footprint_fill", 0, 0);
  VisualizationPlot::ClearMarkers("trajectory_footprint_outline", 0, 0);
  VisualizationPlot::ClearMarkers("trajectory_footprint_scales", 0, 0);
  last_interval_ = static_cast<std::size_t>(-1);
}

}  // namespace visualization
