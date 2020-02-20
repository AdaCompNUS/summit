#include "Sidewalk.h"
#include "carla/geom/Math.h"
#include <opencv2/opencv.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace carla {
namespace sidewalk {

Sidewalk::Sidewalk(const std::vector<std::vector<geom::Vector2D>>& polygons)
  : _polygons(polygons) {

  std::vector<rt_value_t> index_entries;

  for (size_t i = 0; i < _polygons.size(); i++) {
    const std::vector<geom::Vector2D>& polygon = _polygons[i];
    for (size_t j = 0; j < polygon.size(); j++) {
      const geom::Vector2D& v_start = polygon[j];
      const geom::Vector2D& v_end = polygon[(j + 1) % polygon.size()];

      index_entries.emplace_back(
          rt_segment_t(
            rt_point_t(v_start.x, v_start.y),
            rt_point_t(v_end.x, v_end.y)),
          std::pair<size_t, size_t>(i, j));
    }
  }

  _segments_index = rt_tree_t(index_entries);
}
  
occupancy::OccupancyMap Sidewalk::CreateOccupancyMap(float width) const {
  occupancy::OccupancyMap occupancy_map;
  for (const std::vector<geom::Vector2D>& polygon : _polygons) {
    occupancy::OccupancyMap polygon_occupancy_map(polygon);
    occupancy::OccupancyMap outer_buffer = polygon_occupancy_map.Buffer(width / 2);
    occupancy::OccupancyMap inner_buffer = polygon_occupancy_map.Buffer(-width / 2);
    occupancy::OccupancyMap buffer_difference = outer_buffer.Difference(inner_buffer);
    occupancy_map = occupancy_map.Union(buffer_difference);
  }
  return occupancy_map;
}

segments::SegmentMap Sidewalk::CreateSegmentMap() const {
  std::vector<geom::Segment2D> segments;

  for (const auto& polygon : _polygons) {
    for (size_t i = 0; i < polygon.size(); i++) {
      segments.emplace_back(
          polygon[i],
          polygon[(i + 1) % polygon.size()]);
    }
  }

  return segments::SegmentMap(std::move(segments));
}
  
geom::Vector2D Sidewalk::GetRoutePointPosition(const SidewalkRoutePoint& route_point) const {
  const geom::Vector2D segment_start = _polygons[route_point.polygon_id][route_point.segment_id]; 
  const geom::Vector2D& segment_end = _polygons[route_point.polygon_id][(route_point.segment_id + 1) % _polygons[route_point.polygon_id].size()];
  return segment_start + (segment_end - segment_start).MakeUnitVector() * route_point.offset;
}

SidewalkRoutePoint Sidewalk::GetNearestRoutePoint(const geom::Vector2D& position) const {
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

  float offset = geom::Vector2D::DotProduct(
      position - segment_start,
      direction);
  offset = std::max(0.0f, std::min((segment_end - segment_start).Length(), offset));

  return SidewalkRoutePoint{
      result.second.first,
      result.second.second,
      offset};
}
  
SidewalkRoutePoint Sidewalk::GetNextRoutePoint(const SidewalkRoutePoint& route_point, float lookahead_distance) const {
  const geom::Vector2D& segment_start = _polygons[route_point.polygon_id][route_point.segment_id];
  const geom::Vector2D& segment_end = _polygons[route_point.polygon_id][(route_point.segment_id + 1) % _polygons[route_point.polygon_id].size()];
  float segment_length = (segment_end - segment_start).Length();
    
  if (route_point.offset + lookahead_distance <= segment_length) {
    return SidewalkRoutePoint{
        route_point.polygon_id,
        route_point.segment_id,
        route_point.offset + lookahead_distance};
  } else {
    return GetNextRoutePoint(
        SidewalkRoutePoint{
          route_point.polygon_id, 
          (route_point.segment_id + 1) % _polygons[route_point.polygon_id].size(),
          0},
        lookahead_distance - (segment_length - route_point.offset));
  }
}

SidewalkRoutePoint Sidewalk::GetPreviousRoutePoint(const SidewalkRoutePoint& route_point, float lookahead_distance) const {
  if (route_point.offset - lookahead_distance >= 0) {
    return SidewalkRoutePoint{
        route_point.polygon_id,
        route_point.segment_id,
        route_point.offset - lookahead_distance};
  } else {
    const geom::Vector2D& segment_start = _polygons[route_point.polygon_id][route_point.segment_id];
    size_t previous_segment_id = (route_point.segment_id == 0 ? _polygons[route_point.polygon_id].size() : route_point.segment_id) - 1;
    const geom::Vector2D& previous_segment_start = _polygons[route_point.polygon_id][previous_segment_id];

    return GetPreviousRoutePoint(
        SidewalkRoutePoint{
          route_point.polygon_id,
          previous_segment_id,
          (segment_start - previous_segment_start).Length()},
        lookahead_distance - route_point.offset);
  }
}
  
boost::optional<SidewalkRoutePoint> Sidewalk::GetAdjacentRoutePoint(const SidewalkRoutePoint& route_point, float max_cross_distance) const {
  const geom::Vector2D& segment_start = _polygons[route_point.polygon_id][route_point.segment_id];
  const geom::Vector2D& segment_end = _polygons[route_point.polygon_id][(route_point.segment_id + 1) % _polygons[route_point.polygon_id].size()];
  geom::Vector2D direction = (segment_end - segment_start).MakeUnitVector();
  geom::Vector2D normal = direction.Rotate(geom::Math::Pi<float>() / 2);

  geom::Vector2D ray_start = segment_start + route_point.offset * direction;
  geom::Vector2D ray_end = ray_start + max_cross_distance * normal;
  
  rt_segment_t ray(
      rt_point_t(ray_start.x, ray_start.y),
      rt_point_t(ray_end.x, ray_end.y));

  std::vector<rt_value_t> results;
  _segments_index.query(
      boost::geometry::index::intersects(ray), 
      std::back_inserter(results));

  boost::optional<float> best_distance;
  boost::optional<rt_value_t> best_result;
  boost::optional<rt_point_t> best_intersection;

  for (const rt_value_t& result : results) {
    if (result.second.first == route_point.polygon_id && result.second.second == route_point.segment_id) continue;

    std::vector<rt_point_t> intersection;
    boost::geometry::intersection(result.first, ray, intersection);
    float distance = static_cast<float>(boost::geometry::distance(ray.first, intersection[0]));
    if (!best_distance || distance < best_distance) {
      best_distance = boost::optional<float>(distance);
      best_result = boost::optional<rt_value_t>(result);
      best_intersection = boost::optional<rt_point_t>(intersection[0]);
    }
  }

  if (best_distance) {
    size_t best_polygon_id = best_result->second.first;
    size_t best_segment_id = best_result->second.second;
    float best_offset = (geom::Vector2D(best_intersection->get<0>(), best_intersection->get<1>()) - _polygons[best_polygon_id][best_segment_id]).Length();

    return boost::optional<SidewalkRoutePoint>({
        best_polygon_id, best_segment_id, best_offset});
  } else {
    return boost::optional<SidewalkRoutePoint>();
  }

}
  
bool Sidewalk::Intersects(const geom::Segment2D& segment) const {
  rt_segment_t segment_(
      rt_point_t(segment.start.x, segment.start.y),
      rt_point_t(segment.end.x, segment.end.y));

  std::vector<rt_value_t> results;
  _segments_index.query(
      boost::geometry::index::intersects(segment_), 
      std::back_inserter(results));

  return results.size() > 0;
}

}
}
