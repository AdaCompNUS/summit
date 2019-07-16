#include <carla/crowd/CrowdController.h>

void export_crowd() {
  using namespace boost::python;
  using namespace carla::crowd;

  class_<CrowdController>("CrowdController", no_init)
    .def("start", &CrowdController::Start)
    .def("stop", &CrowdController::Stop)
  ;
}
