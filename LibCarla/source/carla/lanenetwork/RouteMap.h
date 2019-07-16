#pragma once

#include "LaneNetwork.h"
#include "carla/Memory.h"
#include <cstdint>
#include <vector>
#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace carla {
namespace lanenetwork {

class LaneNetwork;

struct RoutePoint {
  int64_t segment_id;
  float offset;

  RoutePoint(int64_t segment_id, float offset) : segment_id(segment_id), offset(offset) { }
    
  bool operator==(const RoutePoint &route_point) const {
    return segment_id == route_point.segment_id && offset == route_point.offset;
  }

  bool operator!=(const RoutePoint &rhs) const {
    return !(*this == rhs);
  }
};

class RouteMap {
public:

  RouteMap(SharedPtr<const LaneNetwork> lane_network);

  RoutePoint RandRoutePoint();

  geom::Vector2D GetPosition(const RoutePoint& route_point) const;

  RoutePoint GetNearestRoutePoint(const geom::Vector2D& position) const;

  std::vector<RoutePoint> GetNextRoutePoints(const RoutePoint& route_point, float lookahead_distance) const;

private:
  
  typedef std::tuple<bool, int64_t, geom::Vector2D, geom::Vector2D> network_segment_t;
  typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point_t;
  typedef boost::geometry::model::segment<rt_point_t> rt_segment_t;
  typedef std::pair<rt_segment_t, int> rt_value_t;
  typedef boost::geometry::index::rtree<rt_value_t, boost::geometry::index::rstar<16> > rt_tree_t;

  SharedPtr<const LaneNetwork> _lane_network;
  std::vector<network_segment_t> _segments;
  std::unordered_map<int64_t, size_t> _lane_id_to_segment_id_map;
  std::unordered_map<int64_t, size_t> _lane_connection_id_to_segment_id_map;
  rt_tree_t _segments_index;
  
  std::mt19937 _rng;
};

}
}
