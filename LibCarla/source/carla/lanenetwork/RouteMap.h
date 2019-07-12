#pragma once

#include "LaneNetwork.h"
#include <cstdint>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace carla {
namespace lanenetwork {

class LaneNetwork;

struct RoutePoint {
  int64_t id;
  float offset;

  RoutePoint() = default;

  RoutePoint(int64_t id, float offset) : id(id), offset(offset) { }
};

class RouteMap {
public:

  RouteMap() { }

  RouteMap(const LaneNetwork* lane_network);

  geom::Vector2D GetPosition(const RoutePoint& route_point) const ;

  RoutePoint GetNearestRoutePoint(const geom::Vector2D& position) const;

  std::vector<RoutePoint> GetNextRoutePoints(const RoutePoint& route_point, float lookahead_distance) const;

private:
  
  typedef std::pair<bool, int64_t> network_segment_t;
  typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point_t;
  typedef boost::geometry::model::segment<rt_point_t> rt_segment_t;
  typedef std::pair<rt_segment_t, int> rt_value_t;
  typedef boost::geometry::index::rtree<rt_value_t, boost::geometry::index::rstar<16> > rt_tree_t;

  const LaneNetwork* _lane_network;
  std::vector<network_segment_t> _segments;
  std::unordered_map<int64_t, size_t> _lane_id_to_segment_id_map;
  std::unordered_map<int64_t, size_t> _lane_connection_id_to_segment_id_map;
  rt_tree_t _segments_index;

};

}
}
