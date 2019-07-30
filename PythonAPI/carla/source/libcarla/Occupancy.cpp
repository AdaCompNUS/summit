#include <carla/geom/Vector2D.h>
#include <carla/occupancy/OccupancyGrid.h>
#include <carla/occupancy/OccupancyMap.h>
#include <carla/occupancy/PolygonTable.h>
#include <boost/python/register_ptr_to_python.hpp>
#include <boost/python/numpy.hpp>
#include <cstdint>

void export_occupancy() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::occupancy;

  class_<OccupancyGrid>("OccupancyGrid", no_init)
    .def("__init__", 
        make_constructor(+[](uint32_t rows, uint32_t columns) {
          return MakeShared<OccupancyGrid>(rows, columns);
        }))
    .add_property("rows", &OccupancyGrid::Rows)
    .add_property("columns", &OccupancyGrid::Columns)
    .add_property("data", +[](OccupancyGrid& self) {
          return numpy::from_data(
              self.Data(), // Pointer
              numpy::dtype::get_builtin<uint8_t>(), // Type
              make_tuple(self.Rows(), self.Columns()), // Shape
              make_tuple(self.Columns() * sizeof(uint8_t), sizeof(uint8_t)), // Strides
              object());
        })
    .def("get", +[](const OccupancyGrid& self, uint32_t row, uint32_t column) {
          return self.At(row, column);
        })
    .def("set", +[](OccupancyGrid& self, uint32_t row, uint32_t column, uint8_t value) {
          self.At(row, column) = value;
        })
  ;
  
  class_<PolygonTable>("PolygonTable", no_init)
    .def("__init__", 
        make_constructor(+[](size_t rows, size_t columns) {
          return MakeShared<PolygonTable>(rows, columns);
        }))
    .add_property("rows", &PolygonTable::Rows)
    .add_property("columns", &PolygonTable::Columns)
    .def("get", &PolygonTable::Get,
        return_internal_reference<>())
  ;

  class_<OccupancyMap>("OccupancyMap", no_init)
    .def("__init__", 
        make_constructor(+[](const std::vector<geom::Triangle2D>& triangles) {
          return MakeShared<OccupancyMap>(triangles);
        }))
    .add_property("triangles", 
        make_function(&OccupancyMap::Triangles, return_internal_reference<>()))
    .add_property("bounds_min", 
        make_function(&OccupancyMap::BoundsMin))
    .add_property("bounds_max", 
        make_function(&OccupancyMap::BoundsMax))
    .def("create_occupancy_grid", &OccupancyMap::CreateOccupancyGrid)
    .def("create_polygon_table", &OccupancyMap::CreatePolygonTable)
  ;
}
