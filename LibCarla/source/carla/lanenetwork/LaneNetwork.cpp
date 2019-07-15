#include "LaneNetwork.h"
#include <algorithm>
#include <fstream>
#include <carla/geom/Math.h>

namespace carla {
namespace lanenetwork{

LaneNetwork LaneNetwork::Load(const std::string& path) {
  std::ifstream file(path, std::ifstream::binary);

  // TODO: Error opening file.
  // WARNING: union conversion assumes little endian platform (e.g. x86) and big endian data.
  union {
    uint8_t b;
    int32_t i;
    int64_t l;
    double d;
    char buffer[8];
  } u;

  auto read_bool = [&u, &file]() -> boost::optional<bool> {
    if (file.read(u.buffer, 1)) {
      return boost::optional<bool>(u.b);
    } else {
      return boost::optional<bool>();
    }
  };
  auto read_uint8 = [&u, &file]() -> boost::optional<uint8_t> {
    if (file.read(u.buffer, 1)) {
      return boost::optional<uint8_t>(u.b);
    } else {
      return boost::optional<uint8_t>();
    }
  };
  auto read_uint32 = [&u, &file]() -> boost::optional<uint32_t> {
    if (file.read(u.buffer, 4)) {
      std::swap(u.buffer[0], u.buffer[3]);
      std::swap(u.buffer[1], u.buffer[2]);
      return boost::optional<uint32_t>(u.i);
    } else {
      return boost::optional<uint32_t>();
    }
  };
  auto read_int64 = [&u, &file]() -> boost::optional<int64_t> {
    if (file.read(u.buffer, 8)) {
      std::swap(u.buffer[0], u.buffer[7]);
      std::swap(u.buffer[1], u.buffer[6]);
      std::swap(u.buffer[2], u.buffer[5]);
      std::swap(u.buffer[3], u.buffer[4]);
      return boost::optional<int64_t>(u.l);
    } else {
      return boost::optional<int64_t>();
    }
  };
  auto read_double = [&u, &file]() -> boost::optional<double> {
    if (file.read(u.buffer, 8)) {
      std::swap(u.buffer[0], u.buffer[7]);
      std::swap(u.buffer[1], u.buffer[6]);
      std::swap(u.buffer[2], u.buffer[5]);
      std::swap(u.buffer[3], u.buffer[4]);
      return boost::optional<double>(u.d);
    } else {
      return boost::optional<double>();
    }
  };

  LaneNetwork lane_network(static_cast<float>((*read_double())));
  boost::optional<uint8_t> element_type = read_uint8();
  while (element_type) {
    switch(*element_type) {
      case 1: { // Node
        int64_t id = *read_int64();
        geom::Vector2D position(static_cast<float>((*read_double())), static_cast<float>((*read_double())));
        std::swap(position.x, position.y); // TODO change conversion to match.
        lane_network._nodes.emplace(id, Node(id, position));
        break;
      }
      case 2: { // Road
        int64_t id = *read_int64();
        int64_t source_node_id = *read_int64();
        int64_t destination_node_id = *read_int64();
        uint32_t num_forward_lanes = *read_uint32();
        std::vector<int64_t> forward_lane_ids; forward_lane_ids.reserve(static_cast<size_t>(num_forward_lanes));
        for (uint32_t i = 0; i < num_forward_lanes; i++) {
          forward_lane_ids.emplace_back(*read_int64());
        }
        uint32_t num_backward_lanes = *read_uint32();
        std::vector<int64_t> backward_lane_ids; backward_lane_ids.reserve(static_cast<size_t>(num_backward_lanes));
        for (uint32_t i = 0; i < num_backward_lanes; i++) {
          backward_lane_ids.emplace_back(*read_int64());
        }
        lane_network._roads.emplace(id, Road(id, source_node_id, destination_node_id, forward_lane_ids, backward_lane_ids));
        break;
      }
      case 3: { // Lane
        int64_t id = *read_int64();
        int64_t road_id = *read_int64();
        bool is_forward = *read_bool();
        uint32_t index = *read_uint32();
        lane_network._lanes.emplace(id, Lane(id, road_id, is_forward, index));
        lane_network._incoming_lane_connection_ids_map.emplace(id, 0);
        lane_network._outgoing_lane_connection_ids_map.emplace(id, 0);
        break;
      }
      case 4: { // Lane Connection
        int64_t id = *read_int64();
        int64_t source_lane_id = *read_int64();
        int64_t destination_lane_id = *read_int64();
        float source_offset = static_cast<float>(*read_double());
        float destination_offset = static_cast<float>(*read_double());
        lane_network._lane_connections.emplace(id, LaneConnection(id, source_lane_id, destination_lane_id, source_offset, destination_offset));
        lane_network._incoming_lane_connection_ids_map[destination_lane_id].emplace_back(id);
        lane_network._outgoing_lane_connection_ids_map[source_lane_id].emplace_back(id);
        break;
      }
    }
    element_type = read_uint8();
  }



  // Lookup optimizations.
  for (const std::pair<int64_t, Road>& road_entry : lane_network._roads) {
    lane_network._road_direction_map.emplace(
        road_entry.first,
        lane_network.GetRoadDirection(road_entry.second));
  }
  for (const std::pair<int64_t, Lane>& lane_entry : lane_network._lanes) {
    lane_network._lane_start_map.emplace(
        lane_entry.first,
        lane_network.GetLaneStart(lane_entry.second, 0));
    lane_network._lane_end_map.emplace(
        lane_entry.first,
        lane_network.GetLaneEnd(lane_entry.second, 0));
    lane_network._lane_start_min_offset_map.emplace(
        lane_entry.first, 
        lane_network.GetLaneStartMinOffset(lane_entry.second));
    lane_network._lane_end_min_offset_map.emplace(
        lane_entry.first, 
        lane_network.GetLaneEndMinOffset(lane_entry.second));
  }

  return lane_network;
}
  
float LaneNetwork::GetRoadLength(const Road& road) const {
  return (_nodes.at(road.destination_node_id).position - _nodes.at(road.source_node_id).position).Length();
}

geom::Vector2D LaneNetwork::GetRoadDirection(const Road& road) const {
  auto map_value = _road_direction_map.find(road.id);
  if (map_value != _road_direction_map.end()) return map_value->second;

  geom::Vector2D direction = _nodes.at(road.destination_node_id).position - _nodes.at(road.source_node_id).position;
  return direction.MakeUnitVector();
}

geom::Vector2D LaneNetwork::GetLaneDirection(const Lane& lane) const {
  geom::Vector2D direction = GetRoadDirection(_roads.at(lane.road_id));
  if (!lane.is_forward)
    direction = -direction;
  return direction;
}

geom::Vector2D LaneNetwork::GetLaneStart(const Lane& lane, float offset) const {
  auto map_value = _lane_start_map.find(lane.id);
  if (map_value != _lane_start_map.end()) return map_value->second + offset * GetLaneDirection(lane);
  
  const Road& road = _roads.at(lane.road_id);
  geom::Vector2D direction = GetRoadDirection(road);
  geom::Vector2D normal = direction.Rotate(geom::Math::Pi<float>() / 2);
  
  geom::Vector2D center;
  size_t index;

  if (lane.is_forward) {
    center = _nodes.at(road.source_node_id).position + offset * direction;
    index = lane.index + road.backward_lane_ids.size();
  } else {
    center = _nodes.at(road.destination_node_id).position - offset * direction;
    index = road.backward_lane_ids.size() - 1 - lane.index;
  }

  size_t num_lanes = road.forward_lane_ids.size() + road.backward_lane_ids.size();

  return center + static_cast<float>(_lane_width * (index - 0.5f * (num_lanes - 1))) * normal;
}

geom::Vector2D LaneNetwork::GetLaneEnd(const Lane& lane, float offset) const {
  auto map_value = _lane_end_map.find(lane.id);
  if (map_value != _lane_end_map.end()) return map_value->second + offset * (-GetLaneDirection(lane));
  
  const Road& road = _roads.at(lane.road_id);
  geom::Vector2D direction = GetRoadDirection(road);
  geom::Vector2D normal = direction.Rotate(geom::Math::Pi<float>() / 2);
  
  geom::Vector2D center;
  size_t index;

  if (lane.is_forward) {
    center = _nodes.at(road.destination_node_id).position - offset * direction;
    index = lane.index + road.backward_lane_ids.size();
  } else {
    center = _nodes.at(road.source_node_id).position + offset * direction;
    index = road.backward_lane_ids.size() - 1 - lane.index;
  }

  size_t num_lanes = road.forward_lane_ids.size() + road.backward_lane_ids.size();

  return center + static_cast<float>(_lane_width * (index - 0.5 * (num_lanes - 1))) * normal;
}

const std::vector<int64_t>& LaneNetwork::GetIncomingLaneConnectionIds(const Lane& lane) const {
  return _incoming_lane_connection_ids_map.at(lane.id);
}
  
const std::vector<int64_t>& LaneNetwork::GetOutgoingLaneConnectionIds(const Lane& lane) const {
  return _outgoing_lane_connection_ids_map.at(lane.id);
}

float LaneNetwork::GetLaneStartMinOffset(const Lane& lane) const {
  auto map_value = _lane_start_min_offset_map.find(lane.id);
  if (map_value != _lane_start_min_offset_map.end()) return map_value->second;

  boost::optional<float> min_offset;
  for (int64_t lane_connection_id : GetIncomingLaneConnectionIds(lane)) {
    const LaneConnection& lane_connection = _lane_connections.at(lane_connection_id);
    if (!min_offset || *min_offset > lane_connection.destination_offset) {
      min_offset = boost::optional<float>(lane_connection.destination_offset);
    }
  }
  return min_offset.get_value_or(0);
}

float LaneNetwork::GetLaneEndMinOffset(const Lane& lane) const {
  auto map_value = _lane_end_min_offset_map.find(lane.id);
  if (map_value != _lane_end_min_offset_map.end()) return map_value->second;

  boost::optional<float> min_offset;
  for (int64_t lane_connection_id : GetOutgoingLaneConnectionIds(lane)) {
    const LaneConnection& lane_connection = _lane_connections.at(lane_connection_id);
    if (!min_offset || *min_offset > lane_connection.source_offset) {
      min_offset = boost::optional<float>(lane_connection.source_offset);
    }
  }
  return min_offset.get_value_or(0);
}

RouteMap LaneNetwork::CreateRouteMap() const {
  return RouteMap(this);
}

occupancy::OccupancyMap LaneNetwork::CreateOccupancyMap() const {
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

    for (int i = 0; i < 16; i++) {
      v1 = end;
      v2 = end + normal.Rotate(-geom::Math::Pi<float>() / 16.0f * (i + 1)) * width / 2.0;
      v3 = end + normal.Rotate(-geom::Math::Pi<float>() / 16.0f * i) * width / 2.0;
      triangles.emplace_back(v3, v2, v1);

      v1 = start;
      v2 = start + normal.Rotate(geom::Math::Pi<float>() / 16.0f * i) * width / 2.0;
      v3 = start + normal.Rotate(geom::Math::Pi<float>() / 16.0f * (i + 1)) * width / 2.0;
      triangles.emplace_back(v3, v2, v1);
    }
  };

  for (const auto& entry : _lanes) {
    FromSegment(
        GetLaneStart(
          entry.second, 
          GetLaneStartMinOffset(entry.second)),
        GetLaneEnd(
          entry.second, 
          GetLaneEndMinOffset(entry.second)),
        _lane_width);
  }

  for (const auto& entry : _lane_connections) {
    FromSegment(
        GetLaneEnd(
          _lanes.at(entry.second.source_lane_id),
          entry.second.source_offset),
        GetLaneStart(
          _lanes.at(entry.second.destination_lane_id),
          entry.second.destination_offset),
        _lane_width);
  }

  // TODO Move semantics.
  return occupancy::OccupancyMap(triangles);
}

}
}
