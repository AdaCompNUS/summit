#include <carla/geom/Vector2D.h>
#include <carla/occupancy/OccupancyGrid.h>
#include <carla/occupancy/OccupancyMap.h>
#include <boost/python/numpy.hpp>
#include <cstdint>

void export_occupancy() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::occupancy;

  class_<OccupancyGrid>("OccupancyGrid", no_init)
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
    .def("from_line", &OccupancyMap::FromLine)
    .staticmethod("from_line")
    .def("from_polygon", &OccupancyMap::FromPolygon)
    .staticmethod("from_polygon")
    .def("from_bounds", &OccupancyMap::FromBounds)
    .staticmethod("from_bounds")
    .def("union", &OccupancyMap::Union)
    .def("difference", &OccupancyMap::Difference)
    .def("buffer", &OccupancyMap::Buffer)
    .def("create_sidewalk", &OccupancyMap::CreateSidewalk)
    .def("get_mesh_triangles", &OccupancyMap::GetMeshTriangles)
    .def("get_triangles", &OccupancyMap::GetTriangles)
  ;
}
