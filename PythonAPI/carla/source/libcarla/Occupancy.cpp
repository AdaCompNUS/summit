#include <carla/geom/Vector2D.h>
#include <carla/occupancy/OccupancyMap.h>
#include <boost/python/numpy.hpp>
#include <cstdint>

void export_occupancy() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::occupancy;

  class_<OccupancyMap>("OccupancyMap", no_init)
    .def(init<>())
    .def(init<const std::vector<geom::Vector2D>&, float>())
    .def("__init__", make_constructor(
          +[](const list& line_py, float width) {
            std::vector<carla::geom::Vector2D> line{
              stl_input_iterator<carla::geom::Vector2D>(line_py),
              stl_input_iterator<carla::geom::Vector2D>()};
            return boost::shared_ptr<OccupancyMap>(new OccupancyMap(line, width));
          }))
    .def(init<const std::vector<geom::Vector2D>&>())
    .def("__init__", make_constructor(
          +[](const list& polygon_py) {
            std::vector<carla::geom::Vector2D> polygon{
              stl_input_iterator<carla::geom::Vector2D>(polygon_py),
              stl_input_iterator<carla::geom::Vector2D>()};
            return boost::shared_ptr<OccupancyMap>(new OccupancyMap(polygon));
          }))
    .def(init<const geom::Vector2D&, const geom::Vector2D&>())
    .def("load", &OccupancyMap::Load)
    .staticmethod("load")
    .def("save", &OccupancyMap::Save)
    .add_property("is_empty", make_function(&OccupancyMap::IsEmpty)) 
    .def("union", &OccupancyMap::Union)
    .def("difference", &OccupancyMap::Difference)
    .def("intersection", &OccupancyMap::Intersection)
    .def("buffer", &OccupancyMap::Buffer)
    .def("contains", &OccupancyMap::Contains)
    .def("create_sidewalk", &OccupancyMap::CreateSidewalk)
    .def("get_polygons", &OccupancyMap::GetPolygons)
    .def("get_triangles", &OccupancyMap::GetTriangles)
    .def("get_mesh_triangles", &OccupancyMap::GetMeshTriangles, (arg("height")=0.0f))
    .def("get_wall_mesh_triangles", &OccupancyMap::GetWallMeshTriangles)
  ;
  
  class_<std::vector<OccupancyMap>>("vector_of_occupancy_map")
      .def(boost::python::vector_indexing_suite<std::vector<OccupancyMap>>())
  ;
}
