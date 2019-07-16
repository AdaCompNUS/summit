#include <carla/geom/Vector2D.h>
#include <carla/client/World.h>
#include <carla/crowd/CrowdController.h>
#include <carla/lanenetwork/RouteMap.h>

void export_crowd() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::crowd;

  class_<CrowdController>("CrowdController", no_init)
    .def("__init__", 
        make_constructor(+[](SharedPtr<client::World> world, SharedPtr<lanenetwork::RouteMap> route_map, const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max) {
          return MakeShared<CrowdController>(std::move(world), std::move(route_map), bounds_min, bounds_max);
        }))
    .def("start", &CrowdController::Start)
    .def("stop", &CrowdController::Stop)
  ;
}
