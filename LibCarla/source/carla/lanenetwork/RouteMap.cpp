#include "RouteMap.h"
#include "queue"

namespace carla {
namespace lanenetwork {

RouteMap::RouteMap(const LaneNetwork* lane_network)
  : _lane_network(lane_network) {
  
  std::vector<rt_value_t> index_entries; 
  
  for (const auto& entry : _lane_network->Lanes()) {
    geom::Vector2D start = _lane_network->GetLaneStart(
        entry.second,
        _lane_network->GetLaneStartMinOffset(entry.second));
    geom::Vector2D end = _lane_network->GetLaneEnd(
        entry.second,
        _lane_network->GetLaneEndMinOffset(entry.second));

    size_t segment_id = _segments.size();
    _segments.emplace_back(true, entry.first, start, end);

    index_entries.emplace_back(
        rt_segment_t(rt_point_t(start.x, start.y), rt_point_t(end.x, end.y)),
        segment_id);
    _lane_id_to_segment_id_map.emplace(entry.first, static_cast<int64_t>(segment_id));
  }

  for (const auto& entry : _lane_network->LaneConnections()) {
    geom::Vector2D source = _lane_network->GetLaneEnd(
        _lane_network->Lanes().at(entry.second.source_lane_id),
        entry.second.source_offset);
    geom::Vector2D destination = _lane_network->GetLaneStart(
        _lane_network->Lanes().at(entry.second.destination_lane_id),
        entry.second.destination_offset);

    size_t segment_id = _segments.size();
    _segments.emplace_back(false, entry.first, source, destination);
    index_entries.emplace_back(
        rt_segment_t(rt_point_t(source.x, source.y), rt_point_t(destination.x, destination.y)),
        segment_id);
    _lane_connection_id_to_segment_id_map.emplace(entry.first, static_cast<int64_t>(segment_id));
  }

  _segments_index = rt_tree_t(index_entries);
}

geom::Vector2D RouteMap::GetPosition(const RoutePoint& route_point) const {
  const network_segment_t& segment = _segments[static_cast<size_t>(route_point.segment_id)];
  return std::get<2>(segment) + route_point.offset * (std::get<3>(segment) - std::get<2>(segment)).MakeUnitVector();
}
  
RoutePoint RouteMap::RandRoutePoint() {
  size_t segment_id = std::uniform_int_distribution<size_t>(0, _segments.size() - 1)(_rng);
  const network_segment_t& segment = _segments[static_cast<size_t>(segment_id)];

  return RoutePoint(
      static_cast<int64_t>(segment_id),
      std::uniform_real_distribution<float>(0.0, (std::get<3>(segment) - std::get<2>(segment)).Length())(_rng));
}

RoutePoint RouteMap::GetNearestRoutePoint(const geom::Vector2D& position) const {
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

  return RoutePoint(static_cast<int64_t>(result.second), t);
}

std::vector<RoutePoint> RouteMap::GetNextRoutePoints(const RoutePoint& route_point, float lookahead_distance) const {
  std::vector<RoutePoint> next_route_points;

  std::queue<std::pair<RoutePoint, float>> queue;
  queue.push(std::pair<RoutePoint, float>(route_point, lookahead_distance));

  while (!queue.empty()) {
    std::pair<RoutePoint, float> queue_item = queue.front();
    queue.pop();

    const RoutePoint& current_route_point = queue_item.first;
    network_segment_t current_segment = _segments[static_cast<size_t>(current_route_point.segment_id)];

    float offset = current_route_point.offset;
    float distance = queue_item.second;

    if (std::get<0>(current_segment)) { // Segment is lane.
      const Lane& lane = _lane_network->Lanes().at(std::get<1>(current_segment));

      const geom::Vector2D& start = std::get<2>(current_segment);
      const geom::Vector2D& end = std::get<3>(current_segment);

      if (offset + distance <= (end - start).Length()) {
        next_route_points.emplace_back(current_route_point.segment_id, offset + distance);
      }

      for (int64_t outgoing_lane_connection_id : _lane_network->GetOutgoingLaneConnectionIds(lane)) {
        const LaneConnection& outgoing_lane_connection = _lane_network->LaneConnections().at(outgoing_lane_connection_id);

        float outgoing_offset = (end - start).Length() - outgoing_lane_connection.source_offset - _lane_network->GetLaneEndMinOffset(lane);

        if (outgoing_offset >= offset && outgoing_offset - offset < distance) {
          queue.push(std::pair<RoutePoint, float>(
                RoutePoint(
                  static_cast<int64_t>(_lane_connection_id_to_segment_id_map.at(outgoing_lane_connection_id)), 
                  0.0f),
                distance - (outgoing_offset - offset)));
        }
      }
    } else {
      const LaneConnection& lane_connection = _lane_network->LaneConnections().at(std::get<1>(current_segment));

      geom::Vector2D source = std::get<2>(current_segment);
      geom::Vector2D destination = std::get<3>(current_segment);

      if (offset + distance <= (destination - source).Length()) {
        next_route_points.emplace_back(current_route_point.segment_id, offset + distance);
      } else {
        queue.push(std::pair<RoutePoint, float>(
              RoutePoint(
                static_cast<int64_t>(_lane_id_to_segment_id_map.at(lane_connection.destination_lane_id)),
                lane_connection.destination_offset - _lane_network->GetLaneStartMinOffset(_lane_network->Lanes().at(lane_connection.destination_lane_id))),
              distance - ((destination - source).Length() - offset)));
      }
    }
  }

  return next_route_points;
}

}
}
