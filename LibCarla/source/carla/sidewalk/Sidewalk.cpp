#include "Sidewalk.h"
#include "carla/geom/Math.h"
#include <opencv2/opencv.hpp>

namespace carla {
namespace sidewalk {

Sidewalk::Sidewalk(SharedPtr<const occupancy::OccupancyMap> occupancy_map, 
    const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, 
    float width, float resolution)
  : _bounds_min(bounds_min), _bounds_max(bounds_max),
  _width(width), _resolution(resolution),
  _rng(std::random_device()()) {

  occupancy::OccupancyGrid occupancy_grid = occupancy_map->CreateOccupancyGrid(bounds_min, bounds_max, resolution);
  cv::bitwise_not(occupancy_grid.Mat(), occupancy_grid.Mat());
  int kernel_size = static_cast<int>(std::ceil(width / resolution));
  cv::erode(
      occupancy_grid.Mat(), 
      occupancy_grid.Mat(), 
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size)));
      
  std::vector<std::vector<cv::Point>> contours;
  findContours(occupancy_grid.Mat(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const std::vector<cv::Point>& contour : contours) {
    std::vector<geom::Vector2D> polygon(contour.size());
    for (size_t i = 0; i < contour.size(); i++) {
      polygon[i].x = bounds_max.x - (contour[i].y + 0.5f) * resolution;
      polygon[i].y = bounds_min.y + (contour[i].x + 0.5f) * resolution;
    }
    _polygons.emplace_back(std::move(polygon));
  }

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
  const geom::Vector2D segment_end = _polygons[route_point.polygon_id][(route_point.segment_id + 1) % _polygons[route_point.polygon_id].size()]; 

  return segment_start + (segment_end - segment_start).MakeUnitVector() * route_point.offset;
}

SidewalkRoutePoint Sidewalk::RandRoutePoint() {
  size_t polygon_id = std::uniform_int_distribution<size_t>(0, _polygons.size() - 1)(_rng);
  size_t segment_id = std::uniform_int_distribution<size_t>(0, _polygons[polygon_id].size() - 1)(_rng);

  const geom::Vector2D& start = _polygons[polygon_id][segment_id];
  const geom::Vector2D& end = _polygons[polygon_id][(segment_id + 1) % _polygons[polygon_id].size()];

  float offset = std::uniform_real_distribution<float>(0, (end - start).Length())(_rng);
  bool direction = std::uniform_int_distribution<uint8_t>(0, 1)(_rng) == 1;

  return SidewalkRoutePoint(polygon_id, segment_id, offset, direction);
}

}
}
