#include "AABBMap.h"

namespace carla {
namespace aabb {

AABBMap::AABBMap() {

}

AABBMap::AABBMap(const std::vector<geom::AABB2D>& aabbs) {
  std::vector<b_box_t> boxes(aabbs.size());
  for (const geom::AABB2D& aabb : aabbs) {
    boxes.emplace_back(
        b_point_t(aabb.bounds_min.x, aabb.bounds_min.y),
        b_point_t(aabb.bounds_max.x, aabb.bounds_max.y));
  }
  _tree = b_rtree_t(boxes);
}
  
size_t AABBMap::Count() const {
  return _tree.size();
}
  
void AABBMap::Insert(const geom::AABB2D& aabb) {
  _tree.insert(b_box_t(
      b_point_t(aabb.bounds_min.x, aabb.bounds_min.y),
      b_point_t(aabb.bounds_max.x, aabb.bounds_max.y)));
}

bool AABBMap::Intersects(const geom::AABB2D& aabb) const {
  std::vector<b_box_t> intersections;

  b_box_t box(
      b_point_t(aabb.bounds_min.x, aabb.bounds_min.y),
      b_point_t(aabb.bounds_max.x, aabb.bounds_max.y));

  _tree.query(
      boost::geometry::index::intersects(box), 
      std::back_inserter(intersections));

  return intersections.size() > 0;
}

}
}
