#include "TriangleIndex.h"
#include <iterator>

namespace carla {
namespace index {

TriangleIndex::TriangleIndex(const std::vector<geom::Triangle> &triangles) : _triangles(triangles) {

  std::vector<rt_value> tree_entries;

  for (unsigned int i = 0; i < triangles.size(); i++) {
    const geom::Triangle& t = triangles[i];
    float minx = std::min(t.v0.x, std::min(t.v1.x, t.v2.x));
    float miny = std::min(t.v0.y, std::min(t.v1.y, t.v2.y));
    float maxx = std::max(t.v0.x, std::max(t.v1.x, t.v2.x));
    float maxy = std::max(t.v0.y, std::max(t.v1.y, t.v2.y));

    tree_entries.emplace_back(
        rt_box(rt_point(minx, miny), rt_point(maxx, maxy)),
        i);
  }

  _triangles_tree = rt_tree(tree_entries);

}

std::vector<geom::Triangle> TriangleIndex::QueryIntersect(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max) const {

  std::vector<rt_value> tree_entries;
  _triangles_tree.query(
      boost::geometry::index::intersects(rt_box(
          rt_point(bounds_min.x, bounds_min.x), rt_point(bounds_max.x, bounds_max.y))),
      std::back_inserter(tree_entries));

  std::vector<geom::Triangle> triangles;
  for (const rt_value& entry : tree_entries) {
    triangles.emplace_back(_triangles[entry.second]);
  }

  return triangles;
}

}
}
