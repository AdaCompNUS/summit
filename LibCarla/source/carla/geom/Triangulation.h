#pragma once

#include "carla/geom/Vector2D.h"
#include "carla/geom/Triangle2D.h"
#include <vector>

namespace carla {
namespace geom {

class Triangulation {

public:

  // Triangulates a given polygon with holes.
  // Input polygon and holes can have any winding order.
  // Triangulation triangles have clockwise winding order.
  static std::vector<std::pair<size_t, size_t>> Triangulate(const std::vector<std::vector<geom::Vector2D>>& polygon);

};

}
}
