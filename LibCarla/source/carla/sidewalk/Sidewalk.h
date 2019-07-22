#pragma once

#include "carla/geom/Vector2D.h"
#include "carla/occupancy/OccupancyMap.h"
#include "carla/Memory.h"
#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace carla {
namespace sidewalk {

struct SidewalkRoutePoint {

  size_t polygon_id;
  size_t segment_id;
  float offset;
  bool direction;

  SidewalkRoutePoint(size_t polygon_id, size_t segment_id, float offset, bool direction)
    : polygon_id(polygon_id), segment_id(segment_id), offset(offset), direction(direction) { }
  
  bool operator==(const SidewalkRoutePoint &rhs) const {
    return polygon_id == rhs.polygon_id &&
      segment_id == rhs.segment_id &&
      offset == rhs.offset &&
      direction == rhs.direction;
  }

  bool operator!=(const SidewalkRoutePoint &rhs) const {
    return !(*this == rhs);
  }

};

class Sidewalk {

public:

  Sidewalk(SharedPtr<const occupancy::OccupancyMap> occupancy_map, 
      const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, 
      float width, float resolution);

  occupancy::OccupancyMap CreateOccupancyMap() const;
  geom::Vector2D GetRoutePointPosition(const SidewalkRoutePoint& route_point) const;
  SidewalkRoutePoint GetNearestRoutePoint(const geom::Vector2D& position) const;
  SidewalkRoutePoint GetNextRoutePoint(const SidewalkRoutePoint& route_point, float lookahead_distance) const;

private:
  
  typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point_t;
  typedef boost::geometry::model::segment<rt_point_t> rt_segment_t;
  typedef std::pair<rt_segment_t, std::pair<size_t, size_t>> rt_value_t;
  typedef boost::geometry::index::rtree<rt_value_t, boost::geometry::index::rstar<16> > rt_tree_t;
  
  geom::Vector2D _bounds_min;
  geom::Vector2D _bounds_max;
  float _width;
  float _resolution;
  std::vector<std::vector<geom::Vector2D>> _polygons;
  rt_tree_t _segments_index;
  std::mt19937 _rng;

};

}
}
