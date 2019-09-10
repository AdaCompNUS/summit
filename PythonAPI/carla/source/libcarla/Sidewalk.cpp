#include <carla/geom/Vector2D.h>
#include <carla/occupancy/OccupancyMap.h>
#include <carla/sidewalk/Sidewalk.h>
#include <boost/python/register_ptr_to_python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace carla {
namespace sidewalk {

  std::ostream &operator<<(std::ostream &out, const SidewalkRoutePoint &route_point) {
    out << "SidewalkRoutePoint(polygon_id=" << route_point.polygon_id
        << ", segment_id=" << route_point.segment_id
        << ", offset=" << route_point.offset << ')';
    return out;
  }
  
  inline bool operator==(const SidewalkRoutePoint &lhs, const SidewalkRoutePoint &rhs) {
    return lhs.polygon_id == rhs.polygon_id &&
      lhs.segment_id == rhs.segment_id &&
      lhs.offset == rhs.offset;
  }

  inline bool operator!=(const SidewalkRoutePoint &lhs, const SidewalkRoutePoint &rhs) {
    return !(lhs == rhs);
  }

}
}

void export_sidewalk() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::sidewalk;
  
  class_<SidewalkRoutePoint>("SidewalkRoutePoint", init<>())
    .def_readwrite("polygon_id", &SidewalkRoutePoint::polygon_id)
    .def_readwrite("segment_id", &SidewalkRoutePoint::segment_id)
    .def_readwrite("offset", &SidewalkRoutePoint::offset)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<std::vector<SidewalkRoutePoint>>("vector_of_sidewalk_route_point")
    .def(vector_indexing_suite<std::vector<SidewalkRoutePoint>>())
    .def(self_ns::str(self_ns::self))
  ;

  class_<Sidewalk>("Sidewalk", no_init)
    .add_property("polygons", 
        make_function(&Sidewalk::Polygons, return_internal_reference<>()))
    .def("create_occupancy_map",
        &Sidewalk::CreateOccupancyMap)
    .def("get_route_point_position",
        &Sidewalk::GetRoutePointPosition)
    .def("get_nearest_route_point",
        &Sidewalk::GetNearestRoutePoint)
    .def("get_next_route_point",
        &Sidewalk::GetNextRoutePoint)
    .def("get_previous_route_point",
        &Sidewalk::GetPreviousRoutePoint)
    .def("get_adjacent_route_points",
        &Sidewalk::GetAdjacentRoutePoints)
    .def("intersects",
        &Sidewalk::Intersects)
  ;
}
