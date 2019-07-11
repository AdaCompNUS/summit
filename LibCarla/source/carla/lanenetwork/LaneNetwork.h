#pragma once

#include <boost/optional.hpp>
#include <carla/geom/Vector2D.h>
#include <vector>
#include <unordered_map>
#include <string>
#include <cstdint>

namespace carla {
namespace lanenetwork {

struct Node {
  int64_t id;
  geom::Vector2D position;

  Node() = default;

  Node(int64_t id, const geom::Vector2D& position) 
    : id(id), position(position) { }
};

struct Road {
  int64_t id;
  int64_t source_node_id;
  int64_t destination_node_id;

  std::vector<int64_t> forward_lane_ids;
  std::vector<int64_t> backward_lane_ids;

  Road() = default;

  Road(int64_t id, int64_t source_node_id, int64_t destination_node_id, 
      const std::vector<int64_t>& forward_lane_ids = std::vector<int64_t>(),
      const std::vector<int64_t>& backward_lane_ids = std::vector<int64_t>()) 
    : id(id),
    source_node_id(source_node_id),
    destination_node_id(destination_node_id),
    forward_lane_ids(forward_lane_ids),
    backward_lane_ids(backward_lane_ids) { }
};

struct Lane {
  int64_t id;
  int64_t road_id;
  bool is_forward;
  uint32_t index;

  Lane() = default;
  
  Lane(int64_t id, int64_t road_id, bool is_forward, uint32_t index)
    : id(id),
    road_id(road_id),
    is_forward(is_forward),
    index(index) { }
};

struct LaneConnection {
  int64_t id;
  int64_t source_lane_id;
  int64_t destination_lane_id;
  float source_offset;
  float destination_offset;

  LaneConnection() = default;

  LaneConnection(int64_t id, int64_t source_lane_id, int64_t destination_lane_id, float source_offset = 0, float destination_offset = 0) 
    : id(id),
    source_lane_id(source_lane_id),
    destination_lane_id(destination_lane_id),
    source_offset(source_offset),
    destination_offset(destination_offset) { }
};

class LaneNetwork {
public:  

  LaneNetwork(float lane_width=3) : _lane_width(lane_width) { }

  static LaneNetwork Load(const std::string& path);

  float LaneWidth() const { return _lane_width; }

  const std::unordered_map<int64_t, Node>& Nodes() const { return _nodes; }
  
  const std::unordered_map<int64_t, Road>& Roads() const { return _roads; }
  
  const std::unordered_map<int64_t, Lane>& Lanes() const { return _lanes; }
  
  const std::unordered_map<int64_t, LaneConnection>& LaneConnections() const { return _lane_connections; }
  
  float GetRoadLength(const Road& road) const;

  geom::Vector2D GetRoadDirection(const Road& road) const;

  geom::Vector2D GetLaneDirection(const Lane& lane) const;

  geom::Vector2D GetLaneStart(const Lane& lane, float offset=0.0f) const;

  geom::Vector2D GetLaneEnd(const Lane& lane, float offset=0.0f) const;

  const std::vector<int64_t>& GetIncomingLaneConnectionIds(const Lane& lane) const;
  
  const std::vector<int64_t>& GetOutgoingLaneConnectionIds(const Lane& lane) const;

  float GetLaneStartMinOffset(const Lane& lane) const;

  float GetLaneEndMinOffset(const Lane& lane) const;

private:
  
  float _lane_width;
  std::unordered_map<int64_t, Node> _nodes;
  std::unordered_map<int64_t, Road> _roads;
  std::unordered_map<int64_t, Lane> _lanes;
  std::unordered_map<int64_t, LaneConnection> _lane_connections;
  
  // Lookup optimizations.

  std::unordered_map<int64_t, geom::Vector2D> _road_direction_map;

  std::unordered_map<int64_t, geom::Vector2D> _lane_start_map;

  std::unordered_map<int64_t, geom::Vector2D> _lane_end_map;

  std::unordered_map<int64_t, std::vector<int64_t>> _incoming_lane_connection_ids_map;
  
  std::unordered_map<int64_t, std::vector<int64_t>> _outgoing_lane_connection_ids_map;

  std::unordered_map<int64_t, float> _lane_start_min_offset_map;

  std::unordered_map<int64_t, float> _lane_end_min_offset_map;
};

}
}
