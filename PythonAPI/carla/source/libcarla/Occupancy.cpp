#include <carla/geom/Vector2D.h>
#include <carla/occupancy/OccupancyGrid.h>
#include <carla/occupancy/OccupancyMap.h>
#include <boost/python/numpy.hpp>
#include <cstdint>

void export_occupancy() {
  using namespace boost::python;
  using namespace carla::occupancy;

  class_<OccupancyGrid>("OccupancyGrid",
      init<uint32_t, uint32_t>((arg("rows"), arg("columns"))))
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

  class_<OccupancyMap>("OccupancyMap", no_init)
    .def("triangles", &OccupancyMap::Triangles,
        return_internal_reference<>())
    .def("create_occupancy_grid", &OccupancyMap::CreateOccupancyGrid)
  ;
}