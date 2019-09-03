#include "carla/geom/Triangulation.h"

#include <mapbox/earcut.hpp>

namespace mapbox {
namespace util {

template <>
struct nth<0, carla::geom::Vector2D> {
    inline static auto get(const carla::geom::Vector2D &t) {
        return t.x;
    };
};
template <>
struct nth<1, carla::geom::Vector2D> {
    inline static auto get(const carla::geom::Vector2D &t) {
        return t.y;
    };
};

}
}

namespace carla {
namespace geom {

  std::vector<std::pair<size_t, size_t>> Triangulation::Triangulate(const std::vector<std::vector<geom::Vector2D>>& polygon) {
    
    auto expand_index = [&polygon] (size_t index) {
      size_t i = 0;
      while (polygon[i].size() <= index) {
        index -= polygon[i].size();
        i++;
      }
      return std::make_pair(i, index);
    };

    std::vector<size_t> triangulation_indices = mapbox::earcut<size_t>(polygon);
    std::vector<std::pair<size_t, size_t>> triangulation;
    for (size_t i = 0; i < triangulation_indices.size(); i++) {
      triangulation.emplace_back(expand_index(triangulation_indices[i]));
    }

    return triangulation;
  }

}
}
