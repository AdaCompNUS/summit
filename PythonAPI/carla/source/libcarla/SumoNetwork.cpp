#include <carla/sumonetwork/SumoNetwork.h>
#include <carla/geom/Vector2D.h>
#include <cstdint>
#include <unordered_map>
#include "unordered_map_indexing_suite.hpp"
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/register_ptr_to_python.hpp>

namespace carla {
namespace sumonetwork {

inline bool operator==(const Lane& lhs, const Lane& rhs) {
  return lhs.id == rhs.id;
}

inline bool operator!=(const Lane& lhs, const Lane& rhs) {
  return !(lhs.id == rhs.id);
}

inline bool operator==(const Connection& lhs, const Connection& rhs) {
  return lhs.from == rhs.from && 
    lhs.to == rhs.to &&
    lhs.from_lane == rhs.from_lane &&
    lhs.to_lane == rhs.to_lane;
}

inline bool operator!=(const Connection& lhs, const Connection& rhs) {
  return !(lhs == rhs);
}

inline bool operator==(const RoutePoint& lhs, const RoutePoint& rhs) {
  return lhs.edge == rhs.edge &&
    lhs.lane == rhs.lane &&
    lhs.segment == rhs.segment &&
    lhs.offset == rhs.offset;
}

inline bool operator!=(const RoutePoint& lhs, const RoutePoint& rhs) {
  return !(lhs == rhs);
}

std::ostream &operator<<(std::ostream &out, const Function& function) {
  switch (function) {
  case Function::Normal:
    out << "Normal";
    break;
  case Function::Internal:
    out << "Internal";
    break;
  case Function::Connector:
    out << "Connector";
    break;
  case Function::Crossing:
    out << "Crossing";
    break;
  case Function::WalkingArea:
    out << "WalkingArea";
    break;
  }
  return out;
}
  
std::ostream &operator<<(std::ostream &out, const Edge& edge) {
  out << "Edge(id=" << edge.id
      << ", from=" << edge.from
      << ", to=" << edge.to
      << ", priority=" << edge.priority
      << ", function=" << edge.function << ')';
  return out;
}

std::ostream &operator<<(std::ostream &out, const Lane& lane) {
  out << "Lane(id=" << lane.id
      << ", index=" << lane.index
      << ", speed=" << lane.speed
      << ", length=" << lane.length << ')';
  return out;
}

std::ostream &operator<<(std::ostream &out, const Junction& junction) {
  out << "Junction(id=" << junction.id
      << ", x=" << junction.x
      << ", y=" << junction.y << ')';
  return out;
}

std::ostream &operator<<(std::ostream &out, const Connection& connection) {
  out << "Connection(from=" << connection.from
      << ", to=" << connection.to
      << ", from_lane=" << connection.from_lane
      << ", to_lane=" << connection.to_lane
      << ", via=" << connection.via << ')';
  return out;
}

std::ostream &operator<<(std::ostream &out, const RoutePoint& route_point) {
  out << "RoutePoint(edge=" << route_point.edge
      << ", lane=" << route_point.lane
      << ", segment=" << route_point.segment
      << ", offset=" << route_point.offset << ')';
  return out;
}

}
}

void export_sumo_network() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::sumonetwork;

  enum_<Function>("Function")
    .value("Normal", Function::Normal)
    .value("Internal", Function::Internal)
    .value("Connector", Function::Connector)
    .value("Crossing", Function::Crossing)
    .value("WalkingArea", Function::WalkingArea)
  ;

  class_<Edge>("Edge", no_init)
    .def_readonly("id", &Edge::id)
    .def_readonly("from", &Edge::from)
    .def_readonly("to", &Edge::to)
    .def_readonly("priority", &Edge::priority)
    .def_readonly("function", &Edge::function)
    .def_readonly("lanes", &Edge::lanes)
    .def(self_ns::str(self_ns::self))
  ;
  class_<std::unordered_map<std::string, Edge>>("unordered_map_of_string_edge")
    .def(unordered_map_indexing_suite<std::unordered_map<std::string, Edge>>())
  ;
  
  class_<Lane>("Lane", no_init)
    .def_readonly("id", &Lane::id)
    .def_readonly("index", &Lane::index)
    .def_readonly("speed", &Lane::speed)
    .def_readonly("length", &Lane::length)
    .def_readonly("shape", &Lane::shape)
    .def(self_ns::str(self_ns::self))
  ;
  class_<std::vector<Lane>>("vector_of_lane")
    .def(vector_indexing_suite<std::vector<Lane>>())
  ;

  class_<Junction>("Junction", no_init)
    .def_readonly("id", &Junction::id)
    .def_readonly("x", &Junction::x)
    .def_readonly("y", &Junction::y)
    .def_readonly("inc_lanes", &Junction::inc_lanes)
    .def_readonly("int_lanes", &Junction::int_lanes)
    .def_readonly("shape", &Junction::shape)
    .def(self_ns::str(self_ns::self))
  ;
  class_<std::unordered_map<std::string, Junction>>("unordered_map_of_string_junction")
    .def(unordered_map_indexing_suite<std::unordered_map<std::string, Junction>>())
  ;

  class_<Connection>("Connection", no_init)
    .def_readonly("from", &Connection::from)
    .def_readonly("to", &Connection::to)
    .def_readonly("from_lane", &Connection::from_lane)
    .def_readonly("to_lane", &Connection::to_lane)
    .def_readonly("via", &Connection::via)
    .def(self_ns::str(self_ns::self))
  ;
  class_<std::vector<Connection>>("vector_of_connection")
    .def(vector_indexing_suite<std::vector<Connection>>())
  ;
  
  class_<RoutePoint>("RoutePoint", no_init)
    .def_readonly("edge", &RoutePoint::edge)
    .def_readonly("lane", &RoutePoint::lane)
    .def_readonly("segment", &RoutePoint::segment)
    .def_readwrite("offset", &RoutePoint::offset)
    .def(self_ns::str(self_ns::self))
  ;
  class_<std::vector<RoutePoint>>("vector_of_routepoint")
    .def(vector_indexing_suite<std::vector<RoutePoint>>())
  ;

  class_<SumoNetwork>("SumoNetwork", no_init)
    .def("load", 
        +[](const std::string& data) { 
          return MakeShared<SumoNetwork>(SumoNetwork::Load(data)); 
        })
    .staticmethod("load")
    .add_property("edges", 
        make_function(&SumoNetwork::Edges, return_internal_reference<>()))
    .add_property("junctions", 
        make_function(&SumoNetwork::Junctions, return_internal_reference<>()))
    .add_property("connections", 
        make_function(&SumoNetwork::Connections, return_internal_reference<>()))
    .def("get_route_point_position", &SumoNetwork::GetRoutePointPosition)
    .def("get_nearest_route_point", &SumoNetwork::GetNearestRoutePoint)
    .def("get_next_route_points", &SumoNetwork::GetNextRoutePoints)
    .def("create_occupancy_map", &SumoNetwork::CreateOccupancyMap)
  ;
  register_ptr_to_python<SharedPtr<SumoNetwork>>();

}
