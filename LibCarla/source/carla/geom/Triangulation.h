#pragma once

#include "carla/geom/Vector2D.h"
#include <vector>

namespace carla {
namespace geom {

class Triangulation {

public:
  
  static std::vector<size_t> Triangulate(const std::vector<geom::Vector2D>& polygon);

};

}
}
