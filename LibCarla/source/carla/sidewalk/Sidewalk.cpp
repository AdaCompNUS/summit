#include "Sidewalk.h"
#include "carla/geom/Math.h"
#include <opencv2/opencv.hpp>

namespace carla {
namespace sidewalk {

Sidewalk::Sidewalk(const occupancy::OccupancyMap& occupancy_map, 
    const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, 
    float width, float resolution,
    float max_cross_distance)
  : _bounds_min(bounds_min), _bounds_max(bounds_max),
  _width(width), _resolution(resolution),
  _max_cross_distance(max_cross_distance),
  _rng(std::random_device()()) {

  occupancy::OccupancyGrid occupancy_grid = occupancy_map.CreateOccupancyGrid(bounds_min, bounds_max, resolution);
  cv::bitwise_not(occupancy_grid.Mat(), occupancy_grid.Mat());
  int kernel_size = static_cast<int>(std::ceil(width / resolution));
  cv::erode(
      occupancy_grid.Mat(), 
      occupancy_grid.Mat(), 
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size)));
      
  std::vector<std::vector<cv::Point>> contours;
  findContours(occupancy_grid.Mat(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_KCOS);

  std::vector<rt_value_t> index_entries;
  for (size_t i = 0; i < contours.size(); i++) {
    const std::vector<cv::Point>& contour = contours[i];

    std::vector<geom::Vector2D> polygon(contour.size());
    
    for (size_t j = 0; j < contour.size(); j++) {
      polygon[j].x = bounds_max.x - (contour[j].y + 0.5f) * resolution;
      polygon[j].y = bounds_min.y + (contour[j].x + 0.5f) * resolution;
    }

    for (size_t j = 0; j < polygon.size(); j++) {
      const geom::Vector2D& v_start = polygon[j];
      const geom::Vector2D& v_end = polygon[(j + 1) % polygon.size()];

      index_entries.emplace_back(
          rt_segment_t(
            rt_point_t(v_start.x, v_start.y),
            rt_point_t(v_end.x, v_end.y)),
          std::pair<size_t, size_t>(i, j));
    }

    _polygons.emplace_back(std::move(polygon));
  }

  _segments_index = rt_tree_t(index_entries);
}
  
occupancy::OccupancyMap Sidewalk::CreateOccupancyMap() const {
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

  for (const std::vector<geom::Vector2D>& polygon : _polygons) {
    for (size_t i = 0; i < polygon.size(); i++) {
      FromSegment(polygon[i], polygon[(i + 1) % polygon.size()], _width);
    }
  }
  
  return occupancy::OccupancyMap(std::move(triangles));
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
  
std::vector<SidewalkRoutePoint> Sidewalk::GetAdjacentRoutePoints(const SidewalkRoutePoint& route_point) const {
  const geom::Vector2D& segment_start = _polygons[route_point.polygon_id][route_point.segment_id];
  const geom::Vector2D& segment_end = _polygons[route_point.polygon_id][(route_point.segment_id + 1) % _polygons[route_point.polygon_id].size()];
  geom::Vector2D direction = (segment_end - segment_start).MakeUnitVector();
  geom::Vector2D normal = direction.Rotate(geom::Math::Pi<float>() / 2);

  geom::Vector2D ray_start = segment_start + route_point.offset * direction;
  geom::Vector2D ray_end = ray_start + _max_cross_distance * normal;
  
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

  std::vector<SidewalkRoutePoint> adjacent_route_points;

  if (best_distance) {
    size_t best_polygon_id = best_result->second.first;
    size_t best_segment_id = best_result->second.second;
    float best_offset = (geom::Vector2D(best_intersection->get<0>(), best_intersection->get<1>()) - _polygons[best_polygon_id][best_segment_id]).Length();

    adjacent_route_points.push_back(SidewalkRoutePoint{
        best_polygon_id,
        best_segment_id,
        best_offset});
  }

  return adjacent_route_points;
}
  
bool Sidewalk::Intersects(const geom::Vector2D& segment_start, const geom::Vector2D& segment_end) const {
  rt_segment_t segment(
      rt_point_t(segment_start.x, segment_start.y),
      rt_point_t(segment_end.x, segment_end.y));

  std::vector<rt_value_t> results;
  _segments_index.query(
      boost::geometry::index::intersects(segment), 
      std::back_inserter(results));

  return results.size() > 0;
}

}
}
