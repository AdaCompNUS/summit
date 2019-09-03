#include "OccupancyMap.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "carla/geom/Triangulation.h"
#include <algorithm>

namespace carla {
namespace occupancy {

OccupancyMap::OccupancyMap() {

}

OccupancyMap OccupancyMap::FromLine(const std::vector<geom::Vector2D>& line, float width) {
  OccupancyMap result;

  // Convert into b_linestring_t.
  b_linestring_t linestring;
  for (const geom::Vector2D& vertex : line) {
    boost::geometry::append(linestring, b_point_t(vertex.x, vertex.y));
  }

  // Correct geometry.
  boost::geometry::correct(linestring);

  // Calculate buffer.
  boost::geometry::strategy::buffer::distance_symmetric<float> distance_strategy(width / 2);
  boost::geometry::strategy::buffer::join_round join_strategy(18);
  boost::geometry::strategy::buffer::end_round end_strategy(18);
  boost::geometry::strategy::buffer::point_circle circle_strategy(18);
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::buffer(linestring, result._multi_polygon,
      distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

  boost::geometry::buffer(linestring, result._multi_polygon,
      distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

  return result;
}
  
OccupancyMap OccupancyMap::FromPolygon(const std::vector<geom::Vector2D>& polygon) {
  OccupancyMap result;

  // Convert into b_multi_polygon_t.
  result._multi_polygon.emplace_back();
  for (const geom::Vector2D& vertex : polygon) {
    boost::geometry::append(result._multi_polygon[0].outer(), b_point_t(vertex.x, vertex.y));
  }

  // Correct geometry.
  boost::geometry::correct(result._multi_polygon);

  return result;
}

OccupancyMap OccupancyMap::Union(const OccupancyMap& occupancy_map) const {
  OccupancyMap result;
  boost::geometry::union_(_multi_polygon, occupancy_map._multi_polygon, result._multi_polygon);
  return result;
}

OccupancyMap OccupancyMap::Difference(const OccupancyMap& occupancy_map) const {
  OccupancyMap result;
  boost::geometry::difference(_multi_polygon, occupancy_map._multi_polygon, result._multi_polygon);
  return result;
}

OccupancyMap OccupancyMap::Buffer(float width) const {
  OccupancyMap result;

  boost::geometry::strategy::buffer::distance_symmetric<float> distance_strategy(width / 2);
  boost::geometry::strategy::buffer::join_round join_strategy(18);
  boost::geometry::strategy::buffer::end_round end_strategy(18);
  boost::geometry::strategy::buffer::point_circle circle_strategy(18);
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::buffer(_multi_polygon, result._multi_polygon,
      distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

  return result;
}

sidewalk::Sidewalk OccupancyMap::CreateSidewalk(float distance) const {
  std::vector<std::vector<geom::Vector2D>> polygons;

  for (const b_polygon_t& polygon : _multi_polygon) {

    // Calculate outer buffer.
    OccupancyMap outer_map; 
    outer_map._multi_polygon.emplace_back(b_polygon_t({polygon.outer()}));
    boost::geometry::correct(outer_map._multi_polygon);
    outer_map = outer_map.Buffer(distance);

    for (const b_polygon_t& buffer_polygon : outer_map._multi_polygon) {
      
      // Add outer of outer buffer in clockwise order.
      polygons.emplace_back();
      for (const b_point_t& vertex : buffer_polygon.outer()) {
        polygons.back().emplace_back(vertex.x(), vertex.y());
      }

      // Add inners of outer buffer in anticlockwise order.
      // These possibly form positive buffering a convex polygon, leading to overlaps.
      for (const b_ring_t& buffer_polygon_inner : buffer_polygon.inners()) {
        polygons.emplace_back();
        for (const b_point_t& vertex : buffer_polygon_inner) {
          polygons.back().emplace_back(vertex.x(), vertex.y());
        }
        std::reverse(polygons.back().begin(), polygons.back().end());
      }
    }

    for (const b_ring_t& inner : polygon.inners()) {

      // Calculate inner buffer.
      OccupancyMap inner_map; 
      inner_map._multi_polygon.emplace_back(b_polygon_t({inner}));
      boost::geometry::correct(inner_map._multi_polygon);
      inner_map = inner_map.Buffer(-distance);

      // Add outer of inner buffer in anticlockwise order.
      // If polygon is too small for negative buffering, then this will be empty.
      for (const b_polygon_t& buffer_polygon : inner_map._multi_polygon) {
        polygons.emplace_back();
        for (const b_point_t& vertex : buffer_polygon.outer()) {
          polygons.back().emplace_back(vertex.x(), vertex.y());
        }
        std::reverse(polygons.back().begin(), polygons.back().end());
      }
    }
    
  }
  
  return sidewalk::Sidewalk(std::move(polygons));
}
  
std::vector<geom::Vector3D> OccupancyMap::GetMeshTriangles() const {
  std::vector<geom::Vector3D> triangles;

  for (const b_polygon_t& polygon : _multi_polygon) {

    // Calculate polygon with holes.
    std::vector<std::vector<geom::Vector2D>> polygon_with_holes(
        1 + polygon.inners().size(),
        std::vector<geom::Vector2D>());
    for (const b_point_t& vertex : polygon.outer()) {
      polygon_with_holes[0].emplace_back(vertex.x(), vertex.y());
    }
    for (size_t i = 0; i < polygon.inners().size(); i++) {
      const b_ring_t& inner = polygon.inners()[i];
      for (const b_point_t& vertex : inner) {
        polygon_with_holes[1 + i].emplace_back(vertex.x(), vertex.y());
      }
    }

    // Triangulate.
    std::vector<std::pair<size_t, size_t>> polygon_triangulation = geom::Triangulation::Triangulate(polygon_with_holes);
    for (size_t i = 0; i < polygon_triangulation.size(); i += 3) {
      // Flip to get anticlockwise winding order required in UI.
      triangles.emplace_back(
          polygon_with_holes[polygon_triangulation[i + 2].first][polygon_triangulation[i + 2].second].x,
          polygon_with_holes[polygon_triangulation[i + 2].first][polygon_triangulation[i + 2].second].y,
          0);
      triangles.emplace_back(
          polygon_with_holes[polygon_triangulation[i + 1].first][polygon_triangulation[i + 1].second].x,
          polygon_with_holes[polygon_triangulation[i + 1].first][polygon_triangulation[i + 1].second].y,
          0);
      triangles.emplace_back(
          polygon_with_holes[polygon_triangulation[i].first][polygon_triangulation[i].second].x,
          polygon_with_holes[polygon_triangulation[i].first][polygon_triangulation[i].second].y,
          0);
    }
  }
  
  return triangles;
}

}
}
