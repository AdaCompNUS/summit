#include "SegmentMap.h"

namespace carla {
namespace segments {

SegmentMap::SegmentMap() 
    : _rng(std::random_device()()) {
  Build();
}

SegmentMap::SegmentMap(const std::vector<geom::Segment2D>& segments)
    : _rng(std::random_device()()) { 
  _multi_linestring.resize(segments.size());
  for (size_t i = 0; i < segments.size(); i++) {
    boost::geometry::append(_multi_linestring[i], b_point_t(segments[i].start.x, segments[i].start.y));
    boost::geometry::append(_multi_linestring[i], b_point_t(segments[i].end.x, segments[i].end.y));
  }

  Build();
}

void SegmentMap::Build() {
  std::vector<float> weights;

  for (size_t i = 0; i < _multi_linestring.size(); i++) {
    const b_linestring_t& linestring = _multi_linestring[i];
    for (size_t j = 0; j < linestring.size() - 1; j++) {
      weights.emplace_back(boost::geometry::distance(linestring[j], linestring[j + 1]));
      _index_to_segment_map.emplace_back(std::make_pair(i, j));
    }
  }

  _index_distribution = std::discrete_distribution<size_t>(weights.begin(), weights.end());
}
  
bool SegmentMap::IsEmpty() const {
  return boost::geometry::is_empty(_multi_linestring);
}

SegmentMap SegmentMap::Union(const SegmentMap& segment_map) const {
  SegmentMap result;
  boost::geometry::intersection(_multi_linestring, segment_map._multi_linestring, result._multi_linestring);
  result.Build();
  return result;
}

SegmentMap SegmentMap::Difference(const occupancy::OccupancyMap& occupancy_map) const {
  SegmentMap result;
  boost::geometry::difference(_multi_linestring, occupancy_map._multi_polygon, result._multi_linestring);
  result.Build();
  return result;
}

SegmentMap SegmentMap::Intersection(const occupancy::OccupancyMap& occupancy_map) const {
  SegmentMap result;
  boost::geometry::intersection(_multi_linestring, occupancy_map._multi_polygon, result._multi_linestring);
  result.Build();
  return result;
}

void SegmentMap::SeedRand(uint32_t seed) {
  _rng.seed(seed);
}

geom::Vector2D SegmentMap::RandPoint() {
  const std::pair<size_t, size_t>& segment = _index_to_segment_map[_index_distribution(_rng)];
  geom::Vector2D start(
      _multi_linestring[segment.first][segment.second].x(), 
      _multi_linestring[segment.first][segment.second].y());
  geom::Vector2D end(
      _multi_linestring[segment.first][segment.second + 1].x(),
      _multi_linestring[segment.first][segment.second + 1].y());

  return start + std::uniform_real_distribution<float>(0.0f, 1.0f)(_rng) * (end - start);
}
  
std::vector<geom::Segment2D> SegmentMap::GetSegments() const {
  std::vector<geom::Segment2D> segments; 
  for (const b_linestring_t& linestring : _multi_linestring) {
    segments.emplace_back(
        geom::Vector2D(linestring[0].x(), linestring[0].y()),
        geom::Vector2D(linestring[1].x(), linestring[1].y()));
  }
  return segments;
}

}
}
