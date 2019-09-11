#include "SumoNetwork.h"
#include "carla/geom/Math.h"
#include "carla/geom/Triangulation.h"
#include <boost/algorithm/string.hpp>
#include <pugixml/pugixml.hpp>
#include <string>
#include <sstream>
#include <fstream>

namespace carla {
namespace sumonetwork {

static std::vector<geom::Vector2D> parse_shape_list(const std::string& s) {
  if (s.empty()) return {};

  std::vector<std::string> split_list;
  boost::split(
      split_list, 
      s,
      boost::is_any_of(", "),
      boost::token_compress_on);

  std::vector<geom::Vector2D> position_list;
  for (size_t i = 0; i < split_list.size(); i += 2) {
    position_list.emplace_back(
        std::stof(split_list[i + 1]), // Swap for SUMO -> CARLA.
        std::stof(split_list[i]));    // Swap for SUMO -> CARLA.
  }

  return position_list;
}

static geom::Vector2D parse_coordinates(const std::string& s) {
  if (s.empty()) return geom::Vector2D(0, 0);

  std::vector<std::string> split_list;
  boost::split(
      split_list, 
      s,
      boost::is_any_of(","),
      boost::token_compress_on);

  // Swap x, y for SUMO -> CARLA.
  return geom::Vector2D(std::stof(split_list[1]), std::stof(split_list[0]));
}

static std::pair<geom::Vector2D, geom::Vector2D> parse_bounds(const std::string& s) {
  std::vector<std::string> split_list;
  boost::split(
      split_list, 
      s,
      boost::is_any_of(","),
      boost::token_compress_on);

  // Swap x, y for SUMO -> CARLA.
  return std::make_pair(
      geom::Vector2D(std::stof(split_list[1]), std::stof(split_list[0])),
      geom::Vector2D(std::stof(split_list[3]), std::stof(split_list[2])));
}

static std::vector<std::string> parse_string_list(const std::string& s) {
  if (s.empty()) return {};
  
  std::vector<std::string> split_list;
  boost::split(
      split_list, 
      s,
      boost::is_any_of(" "),
      boost::token_compress_on);
  return split_list;
}

SumoNetwork SumoNetwork::Load(const std::string& file) {
  std::ifstream ifs;
  ifs.open(file, std::ios::in);
  std::ostringstream ss;
  ss << ifs.rdbuf();
  ifs.close();

  pugi::xml_document xml;
  xml.load_string(ss.str().c_str());

  SumoNetwork sumo_network;
  
  pugi::xml_node net_node = xml.child("net");
 
  pugi::xml_node location_node = net_node.child("location");
  sumo_network._offset = parse_coordinates(location_node.attribute("netOffset").value());
  std::pair<geom::Vector2D, geom::Vector2D> bounds = parse_bounds(location_node.attribute("convBoundary").value());
  sumo_network._bounds_min = bounds.first;
  sumo_network._bounds_max = bounds.second;

  for (pugi::xml_node edge_node : net_node.children("edge")) {
    Edge edge;
    edge.id = edge_node.attribute("id").value();
    edge.from = edge_node.attribute("from").value();
    edge.to = edge_node.attribute("to").value();
    edge.priority = edge_node.attribute("priority").as_int();
    std::string function = edge_node.attribute("function").value();
    if (function == "internal") edge.function = Function::Internal;
    else if (function == "connector") edge.function = Function::Connector;
    else if (function == "crossing") edge.function = Function::Crossing;
    else if (function == "walkingarea") edge.function = Function::WalkingArea;
    else edge.function = Function::Normal;

    // Temp vector required because order of lanes read do not necessarily match
    // order of their indexes.
    std::vector<Lane> lanes_temp;
    for (pugi::xml_node lane_node : edge_node.children("lane")) {
      Lane lane;
      lane.id = lane_node.attribute("id").value();
      lane.index = lane_node.attribute("index").as_uint();
      lane.speed = lane_node.attribute("speed").as_float();
      lane.length = lane_node.attribute("length").as_float();
      lane.shape = parse_shape_list(lane_node.attribute("shape").value());
      lanes_temp.emplace_back(std::move(lane));
    }
    edge.lanes.resize(lanes_temp.size());
    // TODO: Check if sorting passes or there are invalid instances, e.g. indexes
    // that exceed the bounds of the number of lanes.
    for (size_t i = 0; i < lanes_temp.size(); i++) {
      edge.lanes[lanes_temp[i].index] = std::move(lanes_temp[i]);
    }

    sumo_network._edges.emplace(edge.id, std::move(edge));
  }

  for (pugi::xml_node junction_node : net_node.children("junction")) {
    Junction junction;
    junction.id = junction_node.attribute("id").value();
    junction.x = junction_node.attribute("y").as_float(); // Swap for SUMO -> CARLA.
    junction.y = junction_node.attribute("x").as_float(); // Swap for SUMO -> CARLA.
    junction.inc_lanes = parse_string_list(junction_node.attribute("incLanes").value());
    junction.int_lanes = parse_string_list(junction_node.attribute("intLanes").value());
    junction.shape = parse_shape_list(junction_node.attribute("shape").value());
    sumo_network._junctions.emplace(junction.id, std::move(junction));
  }

  for (pugi::xml_node connection_node : net_node.children("connection")) {
    Connection connection;
    connection.from = connection_node.attribute("from").value();
    connection.to = connection_node.attribute("to").value();
    connection.from_lane = connection_node.attribute("fromLane").as_uint();
    connection.to_lane = connection_node.attribute("toLane").as_uint();
    connection.via = connection_node.attribute("via").value();
    sumo_network._connections.emplace_back(std::move(connection));
  }
  
  sumo_network.Build();

  return sumo_network;
}

void SumoNetwork::Build() {
  std::vector<rt_value_t> index_entries;
  for (const auto& edge_entry : _edges) {
    const Edge& edge = edge_entry.second;
    for (const Lane& lane : edge.lanes) {
      for (size_t i = 0; i < lane.shape.size() - 1; i++) {
        index_entries.emplace_back(
            rt_segment_t(
              rt_point_t(lane.shape[i].x, lane.shape[i].y),
              rt_point_t(lane.shape[i + 1].x, lane.shape[i + 1].y)),
            std::make_tuple(edge.id, lane.index, i));
      }
      _lane_to_parent_edge_map[lane.id] = std::make_pair(edge.id, lane.index);
      _outgoing_connections_map.emplace(lane.id, std::vector<size_t>());
    }
  }
  _segments_index = rt_tree_t(index_entries);

  for (size_t i = 0; i < _connections.size(); i++) {
    const Connection& connection = _connections[i];

    if (!connection.via.empty()){
      _internal_edge_to_connection_map[connection.via] = i;
    }
    _outgoing_connections_map.at(_edges[connection.from].lanes[connection.from_lane].id).emplace_back(i); 
  }
}
  
geom::Vector2D SumoNetwork::GetRoutePointPosition(const RoutePoint& route_point) const {
  const geom::Vector2D& start = _edges.at(route_point.edge).lanes[route_point.lane].shape[route_point.segment];
  const geom::Vector2D& end = _edges.at(route_point.edge).lanes[route_point.lane].shape[route_point.segment + 1];
  return start + route_point.offset * (end - start).MakeUnitVector();
}
  
RoutePoint SumoNetwork::GetNearestRoutePoint(const geom::Vector2D& position) const {
  std::vector<rt_value_t> results;
  _segments_index.query(boost::geometry::index::nearest(rt_point_t(position.x, position.y), 1), std::back_inserter(results));
  rt_value_t& result = results[0];

  geom::Vector2D segment_start(
      boost::geometry::get<0, 0>(result.first),
      boost::geometry::get<0, 1>(result.first));
  geom::Vector2D segment_end(
      boost::geometry::get<1, 0>(result.first),
      boost::geometry::get<1, 1>(result.first));
  geom::Vector2D direction = (segment_end - segment_start).MakeUnitVector();
  
  float t = geom::Vector2D::DotProduct(
      position - segment_start,
      direction);
  t = std::max(0.0f, std::min((segment_end - segment_start).Length(), t));
  
  return RoutePoint{
    std::get<0>(result.second),
    std::get<1>(result.second),
    std::get<2>(result.second),
    t};
}
  
std::vector<RoutePoint> SumoNetwork::GetNextRoutePoints(const RoutePoint& route_point, float distance) const {
  const Edge& edge = _edges.at(route_point.edge);
  const Lane& lane = edge.lanes[route_point.lane];

  const geom::Vector2D& segment_start = lane.shape[route_point.segment];
  const geom::Vector2D& segment_end = lane.shape[route_point.segment + 1];
  float segment_length = (segment_end - segment_start).Length();

  if (route_point.offset + distance <= segment_length) {
    return { {route_point.edge, route_point.lane, route_point.segment, route_point.offset + distance} };
  } else if (route_point.segment < lane.shape.size() - 2) {
    return GetNextRoutePoints(
        RoutePoint{route_point.edge, route_point.lane, route_point.segment + 1, 0},
        distance - (segment_length - route_point.offset));
  } else {
    std::vector<RoutePoint> next_route_points;
    for (size_t connection_index : _outgoing_connections_map.at(lane.id)) {
      const Connection& connection = _connections[connection_index];
      if (connection.via == "") {
        std::vector<RoutePoint> results = GetNextRoutePoints(
            RoutePoint{connection.to, connection.to_lane, 0, 0},
            distance - (segment_length - route_point.offset));
        next_route_points.insert(next_route_points.end(), results.begin(), results.end());
      } else {
        const std::pair<std::string, uint32_t>& lane_parent = _lane_to_parent_edge_map.at(connection.via);
        std::vector<RoutePoint> results = GetNextRoutePoints(
            RoutePoint{lane_parent.first, lane_parent.second, 0, 0},
            distance - (segment_length - route_point.offset));
        next_route_points.insert(next_route_points.end(), results.begin(), results.end());
      }
    }

    return next_route_points;
  }
}

std::vector<std::vector<RoutePoint>> SumoNetwork::GetNextRoutePaths(const RoutePoint& route_point, size_t num_points, float interval) const {
  if (num_points == 0) return {{route_point}};

  std::vector<std::vector<RoutePoint>> result;
  for (const RoutePoint& next_route_point : GetNextRoutePoints(route_point, interval)) {
    std::vector<std::vector<RoutePoint>> next_route_paths = GetNextRoutePaths(next_route_point, num_points - 1, interval);
    result.reserve(next_route_paths.size());
    for (const std::vector<RoutePoint>& next_route_path : next_route_paths) {
      result.emplace_back();
      result.back().reserve(1 + next_route_path.size());
      result.back().emplace_back(route_point);
      result.back().insert(result.back().end(), next_route_path.begin(), next_route_path.end());
    }
  }
  return result;
}

occupancy::OccupancyMap SumoNetwork::CreateOccupancyMap() const {
  occupancy::OccupancyMap occupancy_map;

  for (const auto& edge_entry : _edges) {
    const Edge& edge = edge_entry.second;
    for (const Lane& lane : edge.lanes) {
      occupancy_map = occupancy_map.Union(occupancy::OccupancyMap(lane.shape, 4.10f));
    }
  }
  
  for (const auto& junction_entry : _junctions) {
    const Junction& junction = junction_entry.second;
    occupancy_map = occupancy_map.Union(occupancy::OccupancyMap(junction.shape));
  }

  return occupancy_map;
}
  
segments::SegmentMap SumoNetwork::CreateSegmentMap() const {
  std::vector<geom::Segment2D> segments;

  for (const auto& edge_entry : _edges) {
    const Edge& edge = edge_entry.second;
    for (const Lane& lane : edge.lanes) {
      for (size_t i = 0; i < lane.shape.size() - 1; i++) {
        segments.emplace_back(lane.shape[i], lane.shape[i + 1]);
      }
    }
  }

  return segments::SegmentMap(std::move(segments));
}

std::vector<geom::Vector3D> SumoNetwork::GetRoadmarkMeshTriangles() const {
  return {};
}

std::vector<RoutePoint> SumoNetwork::QueryIntersect(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max) const {

  rt_box_t box(
      rt_point_t(bounds_min.x, bounds_min.y),
      rt_point_t(bounds_max.x, bounds_max.y));

  std::vector<rt_value_t> results;
  _segments_index.query(
      boost::geometry::index::intersects(box), 
      std::back_inserter(results));

  std::vector<RoutePoint> route_points;
  for (const rt_value_t& result : results) {
    route_points.push_back({
        std::get<0>(result.second),
        std::get<1>(result.second),
        std::get<2>(result.second),
        0});
  }

  return route_points;
}

}
}
