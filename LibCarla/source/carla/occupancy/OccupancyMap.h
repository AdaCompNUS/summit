#pragma once

#include "carla/geom/Triangle2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/occupancy/OccupancyGrid.h"
#include "carla/occupancy/PolygonTable.h"
#include "carla/sidewalk/Sidewalk.h"
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace carla {
namespace sidewalk {

class Sidewalk;

}
}

namespace carla {
namespace occupancy {

class OccupancyMap {
public:

  OccupancyMap();
  static OccupancyMap FromLine(const std::vector<geom::Vector2D>& line, float width);
  static OccupancyMap FromPolygon(const std::vector<geom::Vector2D>& polygon);

  OccupancyMap Union(const OccupancyMap& occupancy_map) const;
  OccupancyMap Difference(const OccupancyMap& occupancy_map) const;
  OccupancyMap Buffer(float width) const;

  sidewalk::Sidewalk CreateSidewalk(float distance) const;

  std::vector<geom::Vector3D> GetMeshTriangles() const;

private:
    
  typedef boost::geometry::model::d2::point_xy<float> b_point_t;
  typedef boost::geometry::model::linestring<b_point_t> b_linestring_t;
  typedef boost::geometry::model::ring<b_point_t> b_ring_t;
  typedef boost::geometry::model::polygon<b_point_t> b_polygon_t;
  typedef boost::geometry::model::multi_polygon<b_polygon_t> b_multi_polygon_t;

  b_multi_polygon_t _multi_polygon;

};

}
}
