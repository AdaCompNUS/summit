#pragma once

#include "carla/geom/Triangle2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/occupancy/OccupancyGrid.h"
#include "carla/sidewalk/Sidewalk.h"
#include "carla/segments/SegmentMap.h"
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace carla {
namespace sidewalk {

class Sidewalk;

}
}

namespace carla {
namespace segments {

class SegmentMap;

}
}

namespace carla {
namespace occupancy {

class OccupancyMap {
public:

  // Empty constructor.
  OccupancyMap();
  
  // Filled buffered line.
  OccupancyMap(const std::vector<geom::Vector2D>& line, float width);
  
  // Buffered outline.
  OccupancyMap(const std::vector<geom::Vector2D>& line, float width, float thickness);

  // Polygon.
  OccupancyMap(const std::vector<geom::Vector2D>& polygon);

  // Rectangle.
  OccupancyMap(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max);

  static OccupancyMap Load(const std::string& file);
  void Save(const std::string& file) const;

  bool IsEmpty() const; 
  bool operator==(const OccupancyMap& occupancy_map) const;
  bool operator!=(const OccupancyMap& occupancy_map) const;

  OccupancyMap Union(const OccupancyMap& occupancy_map) const;
  OccupancyMap Difference(const OccupancyMap& occupancy_map) const;
  OccupancyMap Intersection(const OccupancyMap& occupancy_map) const;
  OccupancyMap Buffer(float width) const;
  bool Contains(const geom::Vector2D& point) const;

  sidewalk::Sidewalk CreateSidewalk(float distance) const;
  std::vector<std::vector<std::vector<geom::Vector2D>>> GetPolygons() const;
  std::vector<geom::Triangle2D> GetTriangles() const;
  std::vector<geom::Vector3D> GetMeshTriangles(float height=0) const;
  std::vector<geom::Vector3D> GetWallMeshTriangles(float height) const;

  friend class segments::SegmentMap;

private:
    
  typedef boost::geometry::model::d2::point_xy<float> b_point_t;
  typedef boost::geometry::model::linestring<b_point_t> b_linestring_t;
  typedef boost::geometry::model::ring<b_point_t> b_ring_t;
  typedef boost::geometry::model::polygon<b_point_t> b_polygon_t;
  typedef boost::geometry::model::multi_polygon<b_polygon_t> b_multi_polygon_t;

  b_multi_polygon_t _multi_polygon;

};

}
}
