#include "OccupancyMap.h"
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "carla/geom/Triangulation.h"
#include <algorithm>
#include <sstream>
#include <fstream>

namespace carla {
namespace occupancy {

OccupancyMap::OccupancyMap() {

}

OccupancyMap::OccupancyMap(const std::vector<geom::Vector2D>& line, float width) {
  // Convert into b_linestring_t.
  b_linestring_t linestring;
  for (const geom::Vector2D& vertex : line) {
    boost::geometry::append(linestring, b_point_t(vertex.x, vertex.y));
  }

  // Correct geometry.
  boost::geometry::correct(linestring);

  // Calculate buffer.
  boost::geometry::strategy::buffer::distance_symmetric<float> distance_strategy(width / 2);
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::strategy::buffer::join_round join_strategy(18);
  boost::geometry::strategy::buffer::end_round end_strategy(18);
  boost::geometry::strategy::buffer::point_circle circle_strategy(18);
  boost::geometry::buffer(linestring, _multi_polygon,
      distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
}

OccupancyMap::OccupancyMap(const std::vector<geom::Vector2D>& line, float width, float thickness) {
  // Convert into b_linestring_t.
  b_linestring_t linestring;
  for (const geom::Vector2D& vertex : line) {
    boost::geometry::append(linestring, b_point_t(vertex.x, vertex.y));
  }

  // Correct geometry.
  boost::geometry::correct(linestring);

  // Calculate outline.
  b_multi_polygon_t outline;
  boost::geometry::buffer(linestring, outline,
      boost::geometry::strategy::buffer::distance_symmetric<float>(width / 2),
      boost::geometry::strategy::buffer::side_straight(),
      boost::geometry::strategy::buffer::join_round(18),
      boost::geometry::strategy::buffer::end_flat(),
      boost::geometry::strategy::buffer::point_circle(18));

  b_multi_polygon_t outline_outer;
  boost::geometry::buffer(outline[0].outer(), outline_outer,
      boost::geometry::strategy::buffer::distance_symmetric<float>(thickness / 2),
      boost::geometry::strategy::buffer::side_straight(),
      boost::geometry::strategy::buffer::join_round(18),
      boost::geometry::strategy::buffer::end_flat(),
      boost::geometry::strategy::buffer::point_circle(18));
  
  b_multi_polygon_t outline_inner;
  boost::geometry::buffer(outline[0].outer(), outline_inner,
      boost::geometry::strategy::buffer::distance_symmetric<float>(-thickness / 2),
      boost::geometry::strategy::buffer::side_straight(),
      boost::geometry::strategy::buffer::join_round(18),
      boost::geometry::strategy::buffer::end_flat(),
      boost::geometry::strategy::buffer::point_circle(18));
  
  boost::geometry::difference(outline_outer, outline_inner, _multi_polygon);
}
  
OccupancyMap::OccupancyMap(const std::vector<geom::Vector2D>& polygon) {
  // Convert into b_multi_polygon_t.
  _multi_polygon.emplace_back();
  for (const geom::Vector2D& vertex : polygon) {
    boost::geometry::append(_multi_polygon[0].outer(), b_point_t(vertex.x, vertex.y));
  }

  // Correct geometry.
  boost::geometry::correct(_multi_polygon);
}

OccupancyMap::OccupancyMap(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max)
  : OccupancyMap({
      bounds_min, geom::Vector2D(bounds_min.x, bounds_max.y),
      bounds_max, geom::Vector2D(bounds_max.x, bounds_min.y)}) {

}
  
OccupancyMap OccupancyMap::Load(const std::string& file) {
  std::ifstream ifs;
  ifs.open(file, std::ios::in);
  std::ostringstream ss_data;
  ss_data << ifs.rdbuf();
  ifs.close();
  std::stringstream ss(ss_data.str());

  auto read_point = [&ss] (b_point_t& point) {
    std::string text;
    while (true) {
      char c; ss.get(c);
      if (c == ')') {
        break;
      } else {
        text += c;
      }
    }

    std::vector<std::string> text_split;
    boost::split(text_split, text, boost::is_any_of(" "));
    point.x(std::stof(text_split[0]));
    point.y(std::stof(text_split[1]));
  };

  auto read_ring = [&ss, &read_point] (b_ring_t& ring) {
    while (true) {
      char c; ss.get(c);
      if (c == '(') {
        b_point_t point; read_point(point);
        boost::geometry::append(ring, point);
      } else if (c == ')') {
        break;
      } else {
        // Error
      }
    }
  };

  auto read_polygon = [&ss, &read_ring] (b_polygon_t& polygon) {
    char c; ss.get(c);
    read_ring(polygon.outer());

    while (true) {
      ss.get(c);
      if (c == '(') {
        polygon.inners().resize(polygon.inners().size() + 1);
        read_ring(polygon.inners()[polygon.inners().size() - 1]);
      } else if (c == ')') {
        break;
      } else {
        // Error
      }
    }
  };

  auto read_multi_polygon = [&ss, &read_polygon] (b_multi_polygon_t& multi_polygon) {
    char c; ss.get(c); // ( of multi_polygon.

    while (true) {
      ss.get(c); // ( of polygon, or ).
      if (c == '(') {
        multi_polygon.resize(multi_polygon.size() + 1);
        read_polygon(multi_polygon[multi_polygon.size() - 1]);
      } else if (c == ')') {
        break;
      } else {
        // Error
      }
    }
  };

  OccupancyMap occupancy_map;
  read_multi_polygon(occupancy_map._multi_polygon);
  boost::geometry::correct(occupancy_map._multi_polygon);

  return occupancy_map;
}

void OccupancyMap::Save(const std::string& file) const {
  std::ofstream ofs;
  ofs.open(file, std::ios::out | std::ios::trunc);
  
  ofs << '(';
  for (const b_polygon_t& polygon : _multi_polygon) {
    ofs << '(';
    
    ofs << '(';
    for (const b_point_t& point : polygon.outer()) {
      ofs << '(';
      ofs << std::to_string(point.x()) << ' ' << std::to_string(point.y());
      ofs << ')';
    }
    ofs << ')';

    for (const b_ring_t& inner : polygon.inners()) {
      ofs << '(';
      for (const b_point_t& point : inner) {
        ofs << '(';
        ofs << std::to_string(point.x()) << ' ' << std::to_string(point.y());
        ofs << ')';
      }
      ofs << ')';
    }

    ofs << ')';
  }
  ofs << ')';

  ofs.close();
}

bool OccupancyMap::IsEmpty() const {
  return boost::geometry::is_empty(_multi_polygon);
}
  
bool OccupancyMap::operator==(const OccupancyMap& occupancy_map) const {
  return boost::geometry::equals(_multi_polygon, occupancy_map._multi_polygon);
}

bool OccupancyMap::operator!=(const OccupancyMap& occupancy_map) const {
  return !(*this == occupancy_map);
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

OccupancyMap OccupancyMap::Intersection(const OccupancyMap& occupancy_map) const {
  OccupancyMap result;
  boost::geometry::intersection(_multi_polygon, occupancy_map._multi_polygon, result._multi_polygon);
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
  
bool OccupancyMap::Contains(const geom::Vector2D& point) const {
  return boost::geometry::covered_by(b_point_t(point.x, point.y), _multi_polygon);
}

bool OccupancyMap::Intersects(const OccupancyMap& occupancy_map) const {
  return boost::geometry::intersects(_multi_polygon, occupancy_map._multi_polygon);
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
      std::reverse(polygons.back().begin(), polygons.back().end());

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
      }
    }
    
  }
  
  return sidewalk::Sidewalk(std::move(polygons));
}
 
std::vector<std::vector<std::vector<geom::Vector2D>>> OccupancyMap::GetPolygons() const {
  std::vector<std::vector<std::vector<geom::Vector2D>>> vertices;

  for (const b_polygon_t polygon : _multi_polygon) {  
    vertices.emplace_back(); // New polygon.

    vertices.back().emplace_back(); // New ring.
    for (const b_point_t vertex : polygon.outer()) {
      vertices.back().back().emplace_back(vertex.x(), vertex.y()); // New point.
    }

    for (const b_ring_t ring : polygon.inners()) {
      vertices.back().emplace_back(); // New ring.
      for (const b_point_t vertex : ring) {
        vertices.back().back().emplace_back(vertex.x(), vertex.y()); // New point.
      }
    }
  }

  return vertices;
}


std::vector<geom::Triangle2D> OccupancyMap::GetTriangles() const {
  std::vector<geom::Triangle2D> triangles;

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
      triangles.emplace_back(
          geom::Vector2D(
            polygon_with_holes[polygon_triangulation[i].first][polygon_triangulation[i].second].x,
            polygon_with_holes[polygon_triangulation[i].first][polygon_triangulation[i].second].y),
          geom::Vector2D(
            polygon_with_holes[polygon_triangulation[i + 1].first][polygon_triangulation[i + 1].second].x,
            polygon_with_holes[polygon_triangulation[i + 1].first][polygon_triangulation[i + 1].second].y),
          geom::Vector2D(
            polygon_with_holes[polygon_triangulation[i + 2].first][polygon_triangulation[i + 2].second].x,
            polygon_with_holes[polygon_triangulation[i + 2].first][polygon_triangulation[i + 2].second].y));
    }
  }
  
  return triangles;
}

std::vector<geom::Vector3D> OccupancyMap::GetMeshTriangles(float height) const {
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
      
      // Counterclockwise for upward facing triangle.
      triangles.emplace_back(
          polygon_with_holes[polygon_triangulation[i + 2].first][polygon_triangulation[i + 2].second].x,
          polygon_with_holes[polygon_triangulation[i + 2].first][polygon_triangulation[i + 2].second].y,
          height);
      triangles.emplace_back(
          polygon_with_holes[polygon_triangulation[i + 1].first][polygon_triangulation[i + 1].second].x,
          polygon_with_holes[polygon_triangulation[i + 1].first][polygon_triangulation[i + 1].second].y,
          height);
      triangles.emplace_back(
          polygon_with_holes[polygon_triangulation[i].first][polygon_triangulation[i].second].x,
          polygon_with_holes[polygon_triangulation[i].first][polygon_triangulation[i].second].y,
          height);
      
      // Clockwise for downward facing triangle.
      triangles.emplace_back(
          polygon_with_holes[polygon_triangulation[i].first][polygon_triangulation[i].second].x,
          polygon_with_holes[polygon_triangulation[i].first][polygon_triangulation[i].second].y,
          height);
      triangles.emplace_back(
          polygon_with_holes[polygon_triangulation[i + 1].first][polygon_triangulation[i + 1].second].x,
          polygon_with_holes[polygon_triangulation[i + 1].first][polygon_triangulation[i + 1].second].y,
          height);
      triangles.emplace_back(
          polygon_with_holes[polygon_triangulation[i + 2].first][polygon_triangulation[i + 2].second].x,
          polygon_with_holes[polygon_triangulation[i + 2].first][polygon_triangulation[i + 2].second].y,
          height);
    }
  }
  
  return triangles;
}

std::vector<geom::Vector3D> OccupancyMap::GetWallMeshTriangles(float height) const {
  std::vector<geom::Vector3D> triangles;
  for (const b_polygon_t& polygon : _multi_polygon) {
    for (size_t i = 0; i < polygon.outer().size(); i++) {
      const b_point_t& start = polygon.outer()[i];
      const b_point_t& end = polygon.outer()[(i + 1) % polygon.outer().size()];

      triangles.emplace_back(end.x(), end.y(), height);
      triangles.emplace_back(end.x(), end.y(), 0);
      triangles.emplace_back(start.x(), start.y(), 0);

      triangles.emplace_back(start.x(), start.y(), 0);
      triangles.emplace_back(end.x(), end.y(), 0);
      triangles.emplace_back(end.x(), end.y(), height);

      triangles.emplace_back(start.x(), start.y(), 0);
      triangles.emplace_back(start.x(), start.y(), height);
      triangles.emplace_back(end.x(), end.y(), height);

      triangles.emplace_back(end.x(), end.y(), height);
      triangles.emplace_back(start.x(), start.y(), height);
      triangles.emplace_back(start.x(), start.y(), 0);
    }
    for (const b_ring_t& inner : polygon.inners()) {
      for (size_t i = 0; i < inner.size(); i++) {
        const b_point_t& start = inner[i];
        const b_point_t& end = inner[(i + 1) % inner.size()];

        triangles.emplace_back(end.x(), end.y(), height);
        triangles.emplace_back(end.x(), end.y(), 0);
        triangles.emplace_back(start.x(), start.y(), 0);

        triangles.emplace_back(start.x(), start.y(), 0);
        triangles.emplace_back(end.x(), end.y(), 0);
        triangles.emplace_back(end.x(), end.y(), height);

        triangles.emplace_back(start.x(), start.y(), 0);
        triangles.emplace_back(start.x(), start.y(), height);
        triangles.emplace_back(end.x(), end.y(), height);

        triangles.emplace_back(end.x(), end.y(), height);
        triangles.emplace_back(start.x(), start.y(), height);
        triangles.emplace_back(start.x(), start.y(), 0);
      }
    }
  }
  return triangles;
}

}
}
