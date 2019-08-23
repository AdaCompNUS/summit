#include <carla/landmark/LandmarkMap.h>
#include <cstdint>

void export_landmark() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::landmark;

  class_<LandmarkMap>("LandmarkMap", no_init)
    .def("load", &LandmarkMap::Load)
    .staticmethod("load")
  ;
}
