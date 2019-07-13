#include <carla/geom/Vector2D.h>
#include <carla/occupancy/OccupancyGrid.h>
#include <carla/occupancy/OccupancyMap.h>
#include <cstdint>

void export_occupancy() {
  using namespace boost::python;

  class_<carla::occupancy::OccupancyGrid>("OccupancyGrid",
      init<uint32_t, uint32_t>((arg("rows"), arg("columns"))))
    .def(init<const carla::occupancy::OccupancyGrid &>((arg("rhs"))))
    .def("data", +[](const carla::occupancy::OccupancyGrid& self) -> uintptr_t {
          const uint8_t* data = self.Data();
          return reinterpret_cast<uintptr_t>(data);
        })
    .def("rows", &carla::occupancy::OccupancyGrid::Rows)
    .def("columns", &carla::occupancy::OccupancyGrid::Columns)
    .def("get", +[](const carla::occupancy::OccupancyGrid& self, uint32_t row, uint32_t column) -> uint8_t {
          return self.At(row, column);
        })
    .def("set", +[](carla::occupancy::OccupancyGrid& self, uint32_t row, uint32_t column, uint8_t value) {
          self.At(row, column) = value;
        })
  ;
}
