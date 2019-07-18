#include <carla/lanenetwork/LaneNetwork.h>
#include <carla/lanenetwork/RouteMap.h>
#include <carla/geom/Vector2D.h>
#include <cstdint>
#include <unordered_map>
#include "unordered_map_indexing_suite.hpp"
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/register_ptr_to_python.hpp>

namespace carla {
namespace lanenetwork {

  std::ostream &operator<<(std::ostream &out, const Node &node) {
    out << "Node(id=" << node.id
        << ", position=" << node.position << ')';
    return out;
  }
  
  std::ostream &operator<<(std::ostream &out, const Road &road) {
    out << "Road(id=" << road.id
        << ", source_node_id=" << road.source_node_id
        << ", destination_node_id=" << road.destination_node_id;
    
    out << ", forward_lane_ids=(";
    std::copy(
        road.forward_lane_ids.begin(), 
        road.forward_lane_ids.end(),
        std::ostream_iterator<int64_t>(out, ", "));
    out << ")";
    
    out << ", backward_lane_ids=(";
    std::copy(
        road.backward_lane_ids.begin(), 
        road.backward_lane_ids.end(),
        std::ostream_iterator<int64_t>(out, ", "));
    out << ")";
    
    out << ')';
    return out;
  }

  std::ostream &operator<<(std::ostream &out, const Lane &lane) {
    out << "Lane(id=" << lane.id
        << ", road_id=" << lane.road_id
        << ", is_forward=" << lane.is_forward
        << ", index=" << lane.index << ')';
    return out;
  }
  
  std::ostream &operator<<(std::ostream &out, const LaneConnection &lane_connection) {
    out << "LaneConnection(id=" << lane_connection.id
        << ", source_lane_id=" << lane_connection.source_lane_id
        << ", destination_lane_id=" << lane_connection.destination_lane_id
        << ", source_offset=" << lane_connection.source_offset
        << ", destination_offset=" << lane_connection.destination_offset << ')';
    return out;
  }
  
  std::ostream &operator<<(std::ostream &out, const RoutePoint &route_point) {
    out << "RoutePoint(id=" << route_point.segment_id
        << ", offset=" << route_point.offset << ')';
    return out;
  }
  
}
}

void export_lane_network() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::lanenetwork;

  class_<Node>("Node", 
      init<int64_t, const carla::geom::Vector2D&>(
        (arg("id"), arg("position"))))
    .def(init<const Node &>((arg("rhs"))))
    .def_readwrite("id", &Node::id)
    .def_readwrite("position", &Node::position)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<std::unordered_map<int64_t, Node>>("unordered_map_of_node")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, Node>>())
  ;

  class_<Road>("Road", 
      init<int64_t, int64_t, int64_t>(
        (arg("id"), arg("source_node_id"), arg("destination_node_id"))))
    .def(init<const Road &>((arg("rhs"))))
    .def_readwrite("id", &Road::id)
    .def_readwrite("source_node_id", &Road::id)
    .def_readwrite("destination_node_id", &Road::id)
    .def_readwrite("forward_lane_ids", &Road::forward_lane_ids)
    .def_readwrite("backward_lane_ids", &Road::backward_lane_ids)
    .def(self_ns::str(self_ns::self))
  ;

  class_<std::unordered_map<int64_t, Road>>("unordered_map_of_road")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, Road>>())
  ;

  class_<Lane>("Lane", 
      init<int64_t, int64_t, bool, uint32_t>(
        (arg("id"), arg("road_id"), arg("is_forward"), arg("index"))))
    .def(init<const Lane &>((arg("rhs"))))
    .def_readwrite("id", &Lane::id)
    .def_readwrite("road_id", &Lane::road_id)
    .def_readwrite("is_forward", &Lane::is_forward)
    .def_readwrite("index", &Lane::index)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<std::unordered_map<int64_t, Lane>>("unordered_map_of_lane")
    .def(unordered_map_indexing_suite<std::unordered_map<int64_t, Lane>>())
  ;
  
  class_<LaneConnection>("LaneConnection", 
      init<int64_t, int64_t, int64_t, float, float>(
        (arg("id"), arg("source_lane_id"), arg("destination_lane_id"), arg("source_offset"), arg("destination_offset"))))
    .def(init<const LaneConnection &>((arg("rhs"))))
    .def_readwrite("id", &LaneConnection::id)
    .def_readwrite("source_lane_id", &LaneConnection::source_lane_id)
    .def_readwrite("destination_lane_id", &LaneConnection::destination_lane_id)
    .def_readwrite("source_offset", &LaneConnection::source_offset)
    .def_readwrite("destination_offset", &LaneConnection::destination_offset)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<std::unordered_map<int64_t, LaneConnection>>("unordered_map_of_lane_connection")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, LaneConnection>>())
  ;
 
  class_<LaneNetwork>("LaneNetwork", no_init)
    .def("load", 
        +[](const std::string& path) { 
          return MakeShared<LaneNetwork>(LaneNetwork::Load(path)); 
        }, 
        (arg("path")))
    .staticmethod("load")
    .add_property("lane_width", 
        &LaneNetwork::LaneWidth)
    .add_property("nodes", 
        &LaneNetwork::Nodes,
        return_internal_reference<>())
    .add_property("roads", 
        &LaneNetwork::Roads,
        return_internal_reference<>())
    .add_property("lanes", 
        &LaneNetwork::Lanes,
        return_internal_reference<>())
    .add_property("lane_connections", 
        &LaneNetwork::LaneConnections,
        return_internal_reference<>())
    .def("get_road_length", 
        &LaneNetwork::GetRoadLength, 
        (arg("road")))
    .def("get_road_direction", 
        &LaneNetwork::GetRoadDirection, 
        (arg("road")))
    .def("get_lane_direction", 
        &LaneNetwork::GetLaneDirection, 
        (arg("lane")))
    .def("get_lane_start", 
        &LaneNetwork::GetLaneStart, 
        (arg("lane"), arg("offset") = 0.0f))
    .def("get_lane_end", 
        &LaneNetwork::GetLaneEnd, 
        (arg("lane"), arg("offset") = 0.0f))
    .def("get_incoming_lane_connection_ids", 
        &LaneNetwork::GetIncomingLaneConnectionIds, 
        (arg("lane")),
        return_internal_reference<>())
    .def("get_outgoing_lane_connection_ids", 
        &LaneNetwork::GetOutgoingLaneConnectionIds, 
        (arg("lane")),
        return_internal_reference<>())
    .def("get_lane_start_min_offset", 
        &LaneNetwork::GetLaneStartMinOffset,
        (arg("lane")))
    .def("get_lane_end_min_offset", 
        &LaneNetwork::GetLaneEndMinOffset, 
        (arg("lane")))
    .def("create_occupancy_map",
        &LaneNetwork::CreateOccupancyMap)
  ;
  
  register_ptr_to_python<SharedPtr<LaneNetwork>>();
  
  class_<RoutePoint>("RoutePoint", 
      init<int64_t, float>(
        (arg("id"), arg("offset"))))
    .def(init<const RoutePoint &>((arg("rhs"))))
    .def("__eq__", &RoutePoint::operator==)
    .def("__ne__", &RoutePoint::operator!=)
    .def_readwrite("id", &RoutePoint::segment_id)
    .def_readwrite("offset", &RoutePoint::offset)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<std::vector<RoutePoint>>("vector_of_route_point")
       .def(vector_indexing_suite<std::vector<RoutePoint>>())
  ;
  
  class_<RouteMap>("RouteMap", no_init)
    .def("__init__", 
        make_constructor(+[](SharedPtr<LaneNetwork> lane_network) {
          return MakeShared<RouteMap>(std::move(lane_network));
        }))
    .def("get_position", &RouteMap::GetPosition)
    .def("rand_route_point", &RouteMap::RandRoutePoint)
    .def("get_nearest_route_point", &RouteMap::GetNearestRoutePoint)
    .def("get_next_route_points", &RouteMap::GetNextRoutePoints)
  ;
  
  register_ptr_to_python<SharedPtr<RouteMap>>();
  
}
