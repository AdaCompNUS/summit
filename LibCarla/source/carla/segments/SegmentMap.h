#pragma once

#include "carla/geom/Segment2D.h"
#include "carla/occupancy/OccupancyMap.h"
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <random>

namespace carla {
namespace occupancy {

class OccupancyMap;

}
}

namespace carla {
namespace segments {

class SegmentMap {

public:

  SegmentMap();
  SegmentMap(const std::vector<geom::Segment2D>& segments);

  SegmentMap Union(const SegmentMap& segment_map) const;
  SegmentMap Difference(const occupancy::OccupancyMap& occupancy_map) const;
  SegmentMap Intersection(const occupancy::OccupancyMap& occupancy_map) const;

  void SeedRand(uint32_t seed);
  geom::Vector2D RandPoint();

  std::vector<geom::Segment2D> GetSegments() const;

  friend class occupancy::OccupancyMap;

private:
  
  typedef boost::geometry::model::d2::point_xy<float> b_point_t;
  typedef boost::geometry::model::linestring<b_point_t> b_linestring_t;
  typedef boost::geometry::model::multi_linestring<b_linestring_t> b_multi_linestring_t;

  b_multi_linestring_t _multi_linestring;

  std::vector<std::pair<size_t, size_t>> _index_to_segment_map;
  std::discrete_distribution<size_t> _index_distribution;

  std::mt19937 _rng;

  void Build();
};

}
}
