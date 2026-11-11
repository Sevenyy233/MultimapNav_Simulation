#pragma once
#include "clearing_robot_plugins/model.hpp"
#include <cmath>

namespace clearing_robot_plugins
{
inline Cloud terrainBand(const Cloud &raw, const TerrainMap &support, double upper_height)
{
  Cloud filtered;
  for (const auto &p : raw)
  {
    double floor;
    if (!std::isfinite(p.x + p.y + p.z) || !support.height(p.x, p.y, floor))
      continue;
    if (p.z >= floor - .05 && p.z <= floor + upper_height)
      filtered.push_back(p);
  }
  return filtered;
}
} // namespace clearing_robot_plugins
