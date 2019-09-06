#pragma once

#include <string>
#include <vector>

#include "carla/occupancy/OccupancyMap.h"

namespace carla {
namespace landmark {

class Landmark {
public:
  
  static std::vector<occupancy::OccupancyMap> Load(const std::string& file, const geom::Vector2D& offset = geom::Vector2D(0, 0));

private:

  Landmark();

};

}
}



