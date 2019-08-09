#include "carla/geom/Vector2D.h"
#include "carla/occupancy/OccupancyMap.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace carla {
namespace sumonetwork {

struct Lane;
  
enum class Function {
  Normal,
  Internal,
  Connector,
  Crossing,
  WalkingArea
};

struct Edge {
  std::string id;
  std::string from;
  std::string to;
  int32_t priority;
  Function function;
  std::vector<Lane> lanes;
};

struct Lane {
  std::string id;
  uint32_t index;
  float speed;
  float length;
  std::vector<geom::Vector2D> shape;
};

struct Junction {
  std::string id;
  float x;
  float y;
  std::vector<std::string> inc_lanes;
  std::vector<std::string> int_lanes;
  std::vector<geom::Vector2D> shape;
};

struct Connection {
  std::string from;
  std::string to;
  uint32_t from_lane;
  uint32_t to_lane;
  std::string via;
};

struct RoutePoint {
  std::string edge;
  uint32_t lane;
  uint32_t segment;
  float offset;
};


class SumoNetwork {

public:

  static SumoNetwork Load(const std::string& data);
  
  const std::unordered_map<std::string, Edge>& Edges() const { return _edges; }
  const std::unordered_map<std::string, Junction>& Junctions() const { return _junctions; }
  const std::vector<Connection>& Connections() const { return _connections; }

  geom::Vector2D GetRoutePointPosition(const RoutePoint& route_point) const;
  RoutePoint GetNearestRoutePoint(const geom::Vector2D& position) const;
  std::vector<RoutePoint> GetNextRoutePoints(const RoutePoint& route_point, float distance) const;
  std::vector<std::vector<RoutePoint>> GetNextRoutePaths(const RoutePoint& route_point, float distance, float interval) const;

  occupancy::OccupancyMap CreateOccupancyMap() const;

private:
  
  typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point_t;
  typedef boost::geometry::model::segment<rt_point_t> rt_segment_t;
  typedef std::pair<rt_segment_t, std::tuple<std::string, uint32_t, uint32_t>> rt_value_t; // Segment -> (Edge, Lane Index, Segment Index)
  typedef boost::geometry::index::rtree<rt_value_t, boost::geometry::index::rstar<16> > rt_tree_t;

  std::unordered_map<std::string, Edge> _edges;
  std::unordered_map<std::string, Junction> _junctions;
  std::vector<Connection> _connections;

  rt_tree_t _segments_index;
  std::unordered_map<std::string, std::string> _lane_to_parent_edge_map;
  std::unordered_map<std::string, size_t> _internal_edge_to_connection_map;
  std::unordered_map<std::string, std::vector<size_t>> _outgoing_connections_map;

  void Build();
};

}
}
