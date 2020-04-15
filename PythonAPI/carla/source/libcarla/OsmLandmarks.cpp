#include <carla/osmlandmarks/OsmLandmarks.h>
#include <cstdint>

void export_osm_landmark() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::osmlandmarks;
  
  class_<OsmLandmarks>("OsmLandmarks", no_init)
    .def("load", 
        +[](const std::string& file) { 
          return OsmLandmarks::Load(file); 
        })
    .def("load", 
        +[](const std::string& file, const geom::Vector2D& offset) { 
          return OsmLandmarks::Load(file, offset); 
        })
    .staticmethod("load")
  ;
}
