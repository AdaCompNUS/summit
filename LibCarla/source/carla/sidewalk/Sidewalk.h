#pragma once

#include "carla/geom/Vector2D.h"
#include "carla/occupancy/OccupancyMap.h"
#include "carla/Memory.h"
#include <boost/optional.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace carla {
namespace occupancy {

class OccupancyMap;

}
}

namespace carla {
namespace sidewalk {

struct SidewalkRoutePoint {
  size_t polygon_id;
  size_t segment_id;
  float offset;
};

class Sidewalk {

public:

  Sidewalk(const std::vector<std::vector<geom::Vector2D>>& polygons);

  occupancy::OccupancyMap CreateOccupancyMap(float width) const;
  geom::Vector2D GetRoutePointPosition(const SidewalkRoutePoint& route_point) const;
  SidewalkRoutePoint GetNearestRoutePoint(const geom::Vector2D& position) const;
  SidewalkRoutePoint GetNextRoutePoint(const SidewalkRoutePoint& route_point, float lookahead_distance) const;
  SidewalkRoutePoint GetPreviousRoutePoint(const SidewalkRoutePoint& route_point, float lookahead_distance) const;
  std::vector<SidewalkRoutePoint> GetAdjacentRoutePoints(const SidewalkRoutePoint& route_point, float max_cross_distance) const;
  bool Intersects(const geom::Vector2D& segment_start, const geom::Vector2D& segment_end) const;

private:
  
  typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point_t;
  typedef boost::geometry::model::segment<rt_point_t> rt_segment_t;
  typedef std::pair<rt_segment_t, std::pair<size_t, size_t>> rt_value_t;
  typedef boost::geometry::index::rtree<rt_value_t, boost::geometry::index::rstar<16> > rt_tree_t;
 
  std::vector<std::vector<geom::Vector2D>> _polygons;
  rt_tree_t _segments_index;

};

}
}
