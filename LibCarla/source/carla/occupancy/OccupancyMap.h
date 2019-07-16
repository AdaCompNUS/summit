#pragma once

#include "carla/geom/Triangle2D.h"
#include "carla/occupancy/OccupancyGrid.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace carla {
namespace occupancy {

class OccupancyMap {
public:

  OccupancyMap() = default;

  OccupancyMap(const std::vector<geom::Triangle2D>& triangles);

  const std::vector<geom::Triangle2D>& Triangles() const { return _triangles; }

  OccupancyGrid CreateOccupancyGrid(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, float resolution) const;   

  PolygonTable CreatePolygonTable(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, float resolution) const;

private:
    
  typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point_t;
  typedef boost::geometry::model::box<rt_point_t> rt_box_t;
  typedef std::pair<rt_box_t, std::vector<geom::Triangle2D>::size_type> rt_value_t;
  typedef boost::geometry::index::rtree<rt_value_t, boost::geometry::index::rstar<16>> rt_index_t;

  std::vector<geom::Triangle2D> _triangles;
  rt_index_t _triangles_index;
  
  std::vector<rt_value_t> QueryIntersect(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max) const;
};

}
}
