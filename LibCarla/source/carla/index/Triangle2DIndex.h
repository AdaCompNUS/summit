#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>
#include "carla/geom/Vector2D.h"
#include "carla/geom/Triangle2D.h"

namespace carla {
namespace index {

  class Triangle2DIndex {
  public:
    Triangle2DIndex() = default;
    Triangle2DIndex(const std::vector<geom::Triangle2D> &triangles);

    std::vector<geom::Triangle2D>::size_type Size() { return _triangles.size(); }

    std::vector<geom::Triangle2D> QueryIntersect(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max) const;

  private:
          
    typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point;
    typedef boost::geometry::model::box<rt_point> rt_box;
    typedef std::pair<rt_box, std::vector<geom::Triangle2D>::size_type> rt_value;
    typedef boost::geometry::index::rtree<rt_value, boost::geometry::index::rstar<16>> rt_tree;
      
    std::vector<geom::Triangle2D> _triangles;
    rt_tree _triangles_tree;
  };

}
}
