#include "carla/geom/Triangulation.h"

#include <mapbox/earcut.hpp>

namespace carla {
namespace geom {

  std::vector<size_t> Triangulation::Triangulate(const std::vector<geom::Vector2D>& polygon) {
    std::vector<std::array<float, 2>> perimeter;
    for (const geom::Vector2D& vertex : polygon) {
      perimeter.push_back({vertex.x, vertex.y});
    }
    std::vector<std::vector<std::array<float, 2>>> polygon_mapbox{perimeter};
    return mapbox::earcut<size_t>(polygon_mapbox);
  }

}
}
