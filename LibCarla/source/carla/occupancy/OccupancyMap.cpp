#include "OccupancyMap.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace carla {
namespace occupancy {
  
OccupancyMap::OccupancyMap(const std::vector<geom::Triangle2D>& triangles) : _triangles(triangles) {
  std::vector<rt_value_t> index_entries;

  for (unsigned int i = 0; i < _triangles.size(); i++) {
    const geom::Triangle2D& t = _triangles[i];
    float minx = std::min(t.v0.x, std::min(t.v1.x, t.v2.x));
    float miny = std::min(t.v0.y, std::min(t.v1.y, t.v2.y));
    float maxx = std::max(t.v0.x, std::max(t.v1.x, t.v2.x));
    float maxy = std::max(t.v0.y, std::max(t.v1.y, t.v2.y));

    index_entries.emplace_back(
        rt_box_t(rt_point_t(minx, miny), rt_point_t(maxx, maxy)),
        i);
  }

  _triangles_index = rt_index_t(index_entries);
}

std::vector<OccupancyMap::rt_value_t> OccupancyMap::QueryIntersect(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max) const {
  std::vector<rt_value_t> index_entries;
  _triangles_index.query(
      boost::geometry::index::intersects(rt_box_t(
          rt_point_t(bounds_min.x, bounds_min.x), rt_point_t(bounds_max.x, bounds_max.y))),
      std::back_inserter(index_entries));
  return index_entries;
}
  
OccupancyGrid OccupancyMap::CreateOccupancyGrid(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, float resolution) const {

  cv::Mat mat = cv::Mat::zeros(
      static_cast<int>(std::ceil((bounds_max.x - bounds_min.x) / resolution)),
      static_cast<int>(std::ceil((bounds_max.y - bounds_min.y) / resolution)),
      CV_8UC1);
  std::vector<std::vector<cv::Point>> triangles_mat;

  for (const rt_value_t& index_entry : QueryIntersect(bounds_min, bounds_max)) {
    const geom::Triangle2D& triangle = _triangles[index_entry.second];

    // TODO: Any better way other than casting?
    cv::Point v0(
        static_cast<int>((bounds_max.x - triangle.v0.x) / resolution), 
        static_cast<int>((triangle.v0.y - bounds_min.y) / resolution));
    cv::Point v1(
        static_cast<int>((bounds_max.x - triangle.v1.x) / resolution), 
        static_cast<int>((triangle.v1.y - bounds_min.y) / resolution));
    cv::Point v2(
        static_cast<int>((bounds_max.x - triangle.v2.x) / resolution), 
        static_cast<int>((triangle.v2.y - bounds_min.y) / resolution));

    triangles_mat.emplace_back(std::initializer_list<cv::Point>{ v0, v1, v2 });
  }
  
  cv::fillPoly(mat, triangles_mat, cv::Scalar(255), cv::LINE_AA);

  return OccupancyGrid(mat);
}

}
}
