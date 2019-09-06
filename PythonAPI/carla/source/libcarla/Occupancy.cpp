#include <carla/geom/Vector2D.h>
#include <carla/occupancy/OccupancyGrid.h>
#include <carla/occupancy/OccupancyMap.h>
#include <boost/python/numpy.hpp>
#include <cstdint>

namespace carla {
namespace occupancy {

  std::ostream &operator<<(std::ostream &out, const OccupancyMap &occupancy_map) {
    out << "OccupancyMap()";
    // TODO
    return out;
  }

}
}

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
    .def(init<>())
    .def(init<const std::vector<geom::Vector2D>&, float>())
    .def(init<const std::vector<geom::Vector2D>&>())
    .def(init<const geom::Vector2D&, const geom::Vector2D&>())
    .add_property("is_empty", make_function(&OccupancyMap::IsEmpty)) 
    .def("union", &OccupancyMap::Union)
    .def("difference", &OccupancyMap::Difference)
    .def("intersection", &OccupancyMap::Intersection)
    .def("buffer", &OccupancyMap::Buffer)
    .def("create_sidewalk", &OccupancyMap::CreateSidewalk)
    .def("get_triangles", &OccupancyMap::GetTriangles)
    .def("get_mesh_triangles", &OccupancyMap::GetMeshTriangles, (arg("height")=0.0f))
    .def("get_wall_mesh_triangles", &OccupancyMap::GetWallMeshTriangles)
  ;
  
  class_<std::vector<OccupancyMap>>("vector_of_occupancy_map")
      .def(boost::python::vector_indexing_suite<std::vector<OccupancyMap>>())
      .def(self_ns::str(self_ns::self))
  ;
}
