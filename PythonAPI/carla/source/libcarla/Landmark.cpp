#include <carla/landmark/Landmark.h>
#include <cstdint>

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
  ;
}
