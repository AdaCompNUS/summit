#pragma once

#include "carla/geom/AABB2D.h"
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>

namespace carla {
namespace aabb {

class AABBMap {
public:

  AABBMap();
  AABBMap(const std::vector<geom::AABB2D>& aabbs);

  void Insert(const geom::AABB2D& aabb);
  bool Intersects(const geom::AABB2D& aabb) const;
  size_t Count() const;

private:
    
  typedef boost::geometry::model::d2::point_xy<float> b_point_t;
  typedef boost::geometry::model::box<b_point_t> b_box_t;
  typedef boost::geometry::index::rtree<b_box_t, boost::geometry::index::quadratic<16>> b_rtree_t;

  b_rtree_t _tree;
};

}
}
