#include <carla/landmark/Landmark.h>
#include <cstdint>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace carla {
namespace landmark {

inline bool operator==(const Landmark& lhs, const Landmark& rhs) {
  return lhs.Outline() == rhs.Outline();
}

}
}

void export_landmark() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::landmark;
  
  class_<Landmark>("Landmark", no_init)
    .def("load", 
        +[](const std::string& file) { 
          return Landmark::Load(file); 
        })
    .def("load", 
        +[](const std::string& file, const geom::Vector2D& offset) { 
          return Landmark::Load(file, offset); 
        })
    .staticmethod("load")
    .add_property("outline", 
        make_function(&Landmark::Outline, return_internal_reference<>()))
    .def("get_wall_mesh_triangles", &Landmark::GetWallMeshTriangles)
    .def("get_outline_mesh_triangles", &Landmark::GetOutlineMeshTriangles)
  ;
  class_<std::vector<Landmark>>("vector_of_landmark")
    .def(vector_indexing_suite<std::vector<Landmark>>())
  ;
}
