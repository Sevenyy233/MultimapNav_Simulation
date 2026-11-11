#pragma once
#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace multimap_nav_simulation {
inline bool isSupportContact(std::string name, double upward_normal_z)
{
  if (!std::isfinite(upward_normal_z) || upward_normal_z <= .15) {return false;}
  const auto split = name.rfind("::");
  if (split != std::string::npos) {name = name.substr(split + 2);}
  std::transform(name.begin(), name.end(), name.begin(),
    [](unsigned char c) {return std::tolower(c);});
  for (const auto & word : {"wall", "rail", "prop", "obstacle", "barrier"}) {
    if (name.find(word) != std::string::npos) {return false;}
  }
  // Exact slab aliases in the development worlds. Name exclusions above and
  // the upward-normal requirement also apply to these slabs.
  for (const auto & slab : {"lower_collision","west_lower_collision","south_lower_collision","landing_collision"}) {
    if(name==slab)return true;
  }
  for (const auto & word : {"ground", "floor", "upper", "ramp", "terrain", "surface", "platform", "bridge", "deck"}) {
    if (name.find(word) != std::string::npos) {return true;}
  }
  return false;
}
}  // namespace multimap_nav_simulation
