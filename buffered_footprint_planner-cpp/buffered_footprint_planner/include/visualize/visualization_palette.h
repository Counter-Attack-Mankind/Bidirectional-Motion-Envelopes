#pragma once

#include "common/util/color.h"

namespace visualization {
namespace palette {

inline common::util::Color WithAlpha(common::util::Color color, double alpha) {
  color.set_a(alpha);
  return color;
}

inline common::util::Color Forward() { return common::util::Color::fromRGB(0x17 / 255.0f, 0x64 / 255.0f, 0xa0 / 255.0f); }
inline common::util::Color Reverse() { return common::util::Color::fromRGB(0xcf / 255.0f, 0x6b / 255.0f, 0x24 / 255.0f); }
inline common::util::Color Goal() { return common::util::Color::fromRGB(0x28 / 255.0f, 0x74 / 255.0f, 0x5a / 255.0f); }
inline common::util::Color Error() { return common::util::Color::fromRGB(0xb5 / 255.0f, 0x3b / 255.0f, 0x39 / 255.0f); }
inline common::util::Color ObstacleFill() { return common::util::Color::fromRGB(0xd9 / 255.0f, 0xdc / 255.0f, 0xe0 / 255.0f); }
inline common::util::Color ObstacleEdge() { return common::util::Color::fromRGB(0x73 / 255.0f, 0x7b / 255.0f, 0x83 / 255.0f); }
inline common::util::Color HybridForward() { return common::util::Color::fromRGB(0x86 / 255.0f, 0xb5 / 255.0f, 0xd8 / 255.0f); }
inline common::util::Color HybridReverse() { return common::util::Color::fromRGB(0xd9 / 255.0f, 0x9a / 255.0f, 0x9a / 255.0f); }
inline common::util::Color Start() { return common::util::Color::fromRGB(0x25 / 255.0f, 0x2d / 255.0f, 0x34 / 255.0f); }

constexpr double kObstacleLineWidth = 0.012;
constexpr double kVehicleLineWidth = 0.015;
constexpr double kTrajectoryLineWidth = 0.03;
constexpr double kHybridPathLineWidth = 0.010;
constexpr double kSweptFillAlpha = 0.085;
constexpr double kHistoryOutlineAlpha = 0.34;
constexpr double kHistoryLineWidth = 0.006;
constexpr double kFootprintFillAlpha = 0.18;

}  // namespace palette
}  // namespace visualization
