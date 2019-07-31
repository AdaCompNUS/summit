#include "OccupancyMap.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace carla {
namespace occupancy {
  
OccupancyMap::OccupancyMap(const std::vector<geom::Triangle2D>& triangles) : _triangles(triangles) {
  if (triangles.empty()) {
    throw_exception(std::invalid_argument("empty occupancy map not allowed"));
  }

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
  _bounds_min.x = boost::geometry::get<0>(_triangles_index.bounds().min_corner());
  _bounds_min.y = boost::geometry::get<1>(_triangles_index.bounds().min_corner());
  _bounds_max.x = boost::geometry::get<0>(_triangles_index.bounds().max_corner());
  _bounds_max.y = boost::geometry::get<1>(_triangles_index.bounds().max_corner());
}

std::vector<OccupancyMap::rt_value_t> OccupancyMap::QueryIntersect(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max) const {
  std::vector<rt_value_t> index_entries;
  _triangles_index.query(
      boost::geometry::index::intersects(rt_box_t(
          rt_point_t(bounds_min.x, bounds_min.y), rt_point_t(bounds_max.x, bounds_max.y))),
      std::back_inserter(index_entries));
  return index_entries;
}
  
OccupancyGrid OccupancyMap::CreateOccupancyGrid(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, float resolution) const {

  cv::Mat mat = cv::Mat::zeros(
      static_cast<int>(std::ceil((bounds_max.x - bounds_min.x) / resolution)), // Rows
      static_cast<int>(std::ceil((bounds_max.y - bounds_min.y) / resolution)), // Columns
      CV_8UC1);

  std::vector<std::vector<cv::Point>> triangle_mat;
  triangle_mat.emplace_back();
  triangle_mat.back().emplace_back();
  triangle_mat.back().emplace_back();
  triangle_mat.back().emplace_back();

  for (const rt_value_t& index_entry : QueryIntersect(bounds_min, bounds_max)) {
    const geom::Triangle2D& triangle = _triangles[index_entry.second];

    // TODO: Any better way other than casting?
    triangle_mat[0][0].x = static_cast<int>(std::floor((triangle.v0.y - bounds_min.y) / resolution));
    triangle_mat[0][0].y = static_cast<int>(std::floor((bounds_max.x - triangle.v0.x) / resolution));
    triangle_mat[0][1].x = static_cast<int>(std::floor((triangle.v1.y - bounds_min.y) / resolution));
    triangle_mat[0][1].y = static_cast<int>(std::floor((bounds_max.x - triangle.v1.x) / resolution));
    triangle_mat[0][2].x = static_cast<int>(std::floor((triangle.v2.y - bounds_min.y) / resolution));
    triangle_mat[0][2].y = static_cast<int>(std::floor((bounds_max.x - triangle.v2.x) / resolution));

    cv::fillPoly(mat, triangle_mat, cv::Scalar(255), cv::LINE_8);
  }

  return OccupancyGrid(mat);
}

PolygonTable OccupancyMap::CreatePolygonTable(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, float cell_size, float resolution) const {
  int rows = static_cast<int>(std::ceil((bounds_max.x - bounds_min.x) / cell_size)); 
  int columns = static_cast<int>(std::ceil((bounds_max.y - bounds_min.y) / cell_size)); 

  PolygonTable table(static_cast<size_t>(rows), static_cast<size_t>(columns));
  for (int row = 0; row < rows; row++) {
    for (int column = 0; column < columns; column++) {
      geom::Vector2D cell_bounds_min(bounds_max.x - (row + 1) * cell_size, bounds_min.y + column * cell_size); 
      geom::Vector2D cell_bounds_max(bounds_max.x - row * cell_size, bounds_min.y + (column + 1) * cell_size); 
      OccupancyGrid cell_occupancy_grid = CreateOccupancyGrid(cell_bounds_min, cell_bounds_max, resolution);
      cv::bitwise_not(cell_occupancy_grid.Mat(), cell_occupancy_grid.Mat());
      
      std::vector<std::vector<cv::Point>> contours;
      findContours(cell_occupancy_grid.Mat(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      for (const std::vector<cv::Point>& contour : contours) {
        std::vector<geom::Vector2D> polygon(contour.size());
        for (size_t i = 0; i < contour.size(); i++) {
          polygon[i].x = cell_bounds_max.x - (contour[i].y + 0.5f) * resolution;
          polygon[i].y = cell_bounds_min.y + (contour[i].x + 0.5f) * resolution;
        }
        table.Insert(static_cast<size_t>(row), static_cast<size_t>(column), polygon);
      }
    }
  }

  return table;
}

}
}
