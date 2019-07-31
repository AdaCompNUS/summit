#include "SumoNetwork.h"
#include "carla/geom/Math.h"
#include <boost/algorithm/string.hpp>
#include <mapbox/earcut.hpp>
#include <pugixml/pugixml.hpp>
#include <string>

namespace carla {
namespace sumonetwork {

static std::vector<geom::Vector2D> parse_position_list(const std::string& s) {
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
        std::stof(split_list[i]), 
        std::stof(split_list[i + 1]));
  }

  return position_list;
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

SumoNetwork SumoNetwork::Load(const std::string& data) {
  pugi::xml_document xml;
  xml.load_string(data.c_str());

  SumoNetwork sumo_network;
  
  pugi::xml_node net_node = xml.child("net");
  
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
    for (pugi::xml_node lane_node : edge_node.children("lane")) {
      Lane lane;
      lane.id = lane_node.attribute("id").value();
      lane.index = lane_node.attribute("index").as_uint();
      lane.speed = lane_node.attribute("speed").as_float();
      lane.length = lane_node.attribute("length").as_float();
      lane.shape = parse_position_list(lane_node.attribute("shape").value());
      edge.lanes.emplace_back(std::move(lane));
    }
    sumo_network._edges.emplace(edge.id, std::move(edge));
  }

  for (pugi::xml_node junction_node : net_node.children("junction")) {
    Junction junction;
    junction.id = junction_node.attribute("id").value();
    junction.x = junction_node.attribute("x").as_float();
    junction.y = junction_node.attribute("y").as_float();
    junction.inc_lanes = parse_string_list(junction_node.attribute("incLanes").value());
    junction.int_lanes = parse_string_list(junction_node.attribute("intLanes").value());
    junction.shape = parse_position_list(junction_node.attribute("shape").value());
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
      _lane_to_parent_edge_map[lane.id] = edge.id;
      _outgoing_connections_map.emplace(lane.id, std::initializer_list<const Connection*>());
    }
  }
  _segments_index = rt_tree_t(index_entries);

  for (const Connection& connection : _connections) {
    if (!connection.via.empty()){
      _internal_edge_to_connection_map[connection.via] = &connection;
    }
    _outgoing_connections_map.at(_edges[connection.from].lanes[connection.from_lane].id).emplace_back(&connection); 
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
  auto it_edge = _edges.find(route_point.edge);
  if (it_edge == _edges.end()) {
    std::cout << "Edge does not exist." << std::endl;
  }
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
    if (edge.function == Function::Internal) {
      for (const Connection* connection : _outgoing_connections_map.at(lane.id)) {
        std::vector<RoutePoint> results = GetNextRoutePoints(
            RoutePoint{connection->to, connection->to_lane, 0, 0},
            distance - (segment_length - route_point.offset));
        next_route_points.insert(next_route_points.end(), results.begin(), results.end());
      }
    } else {
      for (const Connection* connection : _outgoing_connections_map.at(lane.id)) {
        std::vector<RoutePoint> results = GetNextRoutePoints(
            RoutePoint{_lane_to_parent_edge_map.at(connection->via), 0, 0, 0},
            distance - (segment_length - route_point.offset));
        next_route_points.insert(next_route_points.end(), results.begin(), results.end());
      }
    }
    return next_route_points;
  }
}

std::vector<std::vector<RoutePoint>> SumoNetwork::GetNextRoutePaths(const RoutePoint& route_point, float distance, float interval) const {
  if (distance < interval) return {{route_point}};

  std::vector<std::vector<RoutePoint>> result;
  for (const RoutePoint& next_route_point : GetNextRoutePoints(route_point, interval)) {
    std::vector<std::vector<RoutePoint>> next_route_paths = GetNextRoutePaths(next_route_point, distance - interval, interval);
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
  std::vector<geom::Triangle2D> triangles;

  auto FromSegment = [&triangles](const geom::Vector2D& start, const geom::Vector2D& end, float width) {
    geom::Vector2D direction = (end - start).MakeUnitVector();
    geom::Vector2D normal = direction.Rotate(geom::Math::Pi<float>() / 2);

    geom::Vector2D v1 = start + normal * width / 2.0;
    geom::Vector2D v2 = start - normal * width / 2.0;
    geom::Vector2D v3 = end + normal * width / 2.0;
    geom::Vector2D v4 = end - normal * width / 2.0;

    triangles.emplace_back(v3, v2, v1);
    triangles.emplace_back(v2, v3, v4);

    for (int i = 0; i < 8; i++) {
      v1 = end;
      v2 = end + normal.Rotate(-geom::Math::Pi<float>() / 8.0f * (i + 1)) * width / 2.0;
      v3 = end + normal.Rotate(-geom::Math::Pi<float>() / 8.0f * i) * width / 2.0;
      triangles.emplace_back(v3, v2, v1);

      v1 = start;
      v2 = start + normal.Rotate(geom::Math::Pi<float>() / 8.0f * i) * width / 2.0;
      v3 = start + normal.Rotate(geom::Math::Pi<float>() / 8.0f * (i + 1)) * width / 2.0;
      triangles.emplace_back(v3, v2, v1);
    }
  };

  for (const auto& edge_entry : _edges) {
    const Edge& edge = edge_entry.second;
    for (const Lane& lane : edge.lanes) {
      for (size_t i = 0; i < lane.shape.size() - 1; i++) {
        FromSegment(lane.shape[i], lane.shape[i + 1], 3.40f); // Extra 0.20m to fill up gaps between roads.
      }
    }
  }

  for (const auto& junction_entry : _junctions) {
    const Junction& junction = junction_entry.second;
    std::vector<std::array<float, 2>> perimeter;
    for (const geom::Vector2D& vertex : junction.shape) {
      perimeter.push_back({vertex.x, vertex.y});
    }
    std::vector<std::vector<std::array<float, 2>>> polygon{perimeter};
    std::vector<size_t> triangle_indices = mapbox::earcut<size_t>(polygon);

    for (size_t i = 0; i < triangle_indices.size(); i += 3) {
      triangles.emplace_back(
          junction.shape[triangle_indices[i + 2]],
          junction.shape[triangle_indices[i + 1]],
          junction.shape[triangle_indices[i]]);
    }
  }
  
  return occupancy::OccupancyMap(triangles);
}

}
}
