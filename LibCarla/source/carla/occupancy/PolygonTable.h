#pragma once

#include <unordered_map>
#include <vector>
#include <boost/functional/hash.hpp>
#include <utility>
#include "carla/geom/Vector2D.h"

namespace carla {
namespace occupancy {

class PolygonTable {
private:

  typedef std::pair<size_t, size_t> index_t;
  typedef std::vector<geom::Vector2D> polygon_t;
  typedef std::vector<std::vector<std::vector<polygon_t>>> table_t;

public:

  PolygonTable(size_t rows, size_t columns) 
    : _rows(rows),
    _columns(columns),
    _table(rows, std::vector<std::vector<polygon_t>>(columns)) { }

  size_t Rows() { return _rows; }
  size_t Columns() { return _columns; }

  void Insert(size_t row, size_t column, const polygon_t& polygon) { _table[row][column].emplace_back(polygon); }
  const std::vector<polygon_t>& Get(size_t row, size_t column) const { return _table[row][column]; }

private:

  size_t _rows;
  size_t _columns;
  table_t _table;
};

}
}
