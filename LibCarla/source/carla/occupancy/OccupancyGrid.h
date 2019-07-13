#pragma once

#include <cstdint>
#include <opencv2/core/mat.hpp>

namespace carla {
namespace occupancy {

class OccupancyGrid {
public:

  OccupancyGrid() = default;

  OccupancyGrid(uint32_t rows, uint32_t columns) 
    : _mat(static_cast<int>(rows), static_cast<int>(columns), CV_8UC1) { }

  OccupancyGrid(const cv::Mat& mat) : _mat(mat) { }

  uint8_t* Data() { return _mat.data; }

  const uint8_t* Data() const { return _mat.data; }

  uint32_t Rows() const { return static_cast<uint32_t>(_mat.rows); }

  uint32_t Columns() const { return static_cast<uint32_t>(_mat.cols); }

  uint8_t& At(uint32_t row, uint32_t column) { 
    return _mat.at<uint8_t>(
        static_cast<int32_t>(row), 
        static_cast<int32_t>(column)); 
  }

  const uint8_t& At(uint32_t row, uint32_t column) const { 
    return _mat.at<uint8_t>(
        static_cast<int32_t>(row), 
        static_cast<int32_t>(column)); 
  }

private:

  cv::Mat _mat;

};

}
}
