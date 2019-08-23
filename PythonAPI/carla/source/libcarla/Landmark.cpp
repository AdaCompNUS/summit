#include <carla/landmark/LandmarkMap.h>
#include <cstdint>

void export_landmark() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::landmark;

  class_<LandmarkMap>("LandmarkMap", no_init)
    .def("load", 
        +[](const std::string& file) { 
          return LandmarkMap::Load(file); 
        })
    .def("load", 
        +[](const std::string& file, const geom::Vector2D& offset) { 
          return LandmarkMap::Load(file, offset); 
        })
    .staticmethod("load")
    .def("get_mesh_triangles", &LandmarkMap::GetMeshTriangles)
  ;
}
