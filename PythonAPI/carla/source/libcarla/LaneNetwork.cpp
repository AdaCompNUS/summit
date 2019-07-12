#include <carla/lanenetwork/LaneNetwork.h>
#include <carla/lanenetwork/RouteMap.h>
#include <carla/geom/Vector2D.h>
#include <cstdint>
#include <unordered_map>
#include "unordered_map_indexing_suite.hpp"
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

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
  
  // TODO print more stuff maybe.
  std::ostream &operator<<(std::ostream &out, const LaneNetwork &lane_network) {
    out << "LaneNetwork(lane_width=" << lane_network.LaneWidth() << ')';
    return out;
  }
  
  std::ostream &operator<<(std::ostream &out, const RoutePoint &route_point) {
    out << "RoutePoint(id=" << route_point.segment_id
        << ", offset=" << route_point.offset << ')';
    return out;
  }
  
  // TODO print more stuff maybe.
  std::ostream &operator<<(std::ostream &out, const RouteMap &route_map) {
    out << "RouteMap()";
    return out;
  }
}
}

void export_lane_network() {
  using namespace boost::python;

  class_<carla::lanenetwork::Node>("Node", 
      init<int64_t, const carla::geom::Vector2D&>(
        (arg("id"), arg("position"))))
    .def(init<const carla::lanenetwork::Node &>((arg("rhs"))))
    .def_readwrite("id", &carla::lanenetwork::Node::id)
    .def_readwrite("position", &carla::lanenetwork::Node::position)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<std::unordered_map<int64_t, carla::lanenetwork::Node>>("unordered_map_of_node")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, carla::lanenetwork::Node>>())
  ;

  class_<carla::lanenetwork::Road>("Road", 
      init<int64_t, int64_t, int64_t>(
        (arg("id"), arg("source_node_id"), arg("destination_node_id"))))
    .def(init<const carla::lanenetwork::Road &>((arg("rhs"))))
    .def_readwrite("id", &carla::lanenetwork::Road::id)
    .def_readwrite("source_node_id", &carla::lanenetwork::Road::id)
    .def_readwrite("destination_node_id", &carla::lanenetwork::Road::id)
    .def_readwrite("forward_lane_ids", &carla::lanenetwork::Road::forward_lane_ids)
    .def_readwrite("backward_lane_ids", &carla::lanenetwork::Road::backward_lane_ids)
    .def(self_ns::str(self_ns::self))
  ;

  class_<std::unordered_map<int64_t, carla::lanenetwork::Road>>("unordered_map_of_road")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, carla::lanenetwork::Road>>())
  ;

  class_<carla::lanenetwork::Lane>("Lane", 
      init<int64_t, int64_t, bool, uint32_t>(
        (arg("id"), arg("road_id"), arg("is_forward"), arg("index"))))
    .def(init<const carla::lanenetwork::Lane &>((arg("rhs"))))
    .def_readwrite("id", &carla::lanenetwork::Lane::id)
    .def_readwrite("road_id", &carla::lanenetwork::Lane::road_id)
    .def_readwrite("is_forward", &carla::lanenetwork::Lane::is_forward)
    .def_readwrite("index", &carla::lanenetwork::Lane::index)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<std::unordered_map<int64_t, carla::lanenetwork::Lane>>("unordered_map_of_lane")
    .def(unordered_map_indexing_suite<std::unordered_map<int64_t, carla::lanenetwork::Lane>>())
  ;
  
  class_<carla::lanenetwork::LaneConnection>("LaneConnection", 
      init<int64_t, int64_t, int64_t, float, float>(
        (arg("id"), arg("source_lane_id"), arg("destination_lane_id"), arg("source_offset"), arg("destination_offset"))))
    .def(init<const carla::lanenetwork::LaneConnection &>((arg("rhs"))))
    .def_readwrite("id", &carla::lanenetwork::LaneConnection::id)
    .def_readwrite("source_lane_id", &carla::lanenetwork::LaneConnection::source_lane_id)
    .def_readwrite("destination_lane_id", &carla::lanenetwork::LaneConnection::destination_lane_id)
    .def_readwrite("source_offset", &carla::lanenetwork::LaneConnection::source_offset)
    .def_readwrite("destination_offset", &carla::lanenetwork::LaneConnection::destination_offset)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<std::unordered_map<int64_t, carla::lanenetwork::LaneConnection>>("unordered_map_of_lane_connection")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, carla::lanenetwork::LaneConnection>>())
  ;
  
  class_<carla::lanenetwork::LaneNetwork>("LaneNetwork", 
      init<float>(
        (arg("lane_width") = 3.0f)))
    .def(init<const carla::lanenetwork::LaneNetwork &>((arg("rhs"))))
    .def("load", 
        &carla::lanenetwork::LaneNetwork::Load, 
        (arg("path")))
    .staticmethod("load")
    .def("lane_width", 
        &carla::lanenetwork::LaneNetwork::LaneWidth)
    .def("nodes", 
        &carla::lanenetwork::LaneNetwork::Nodes,
        return_internal_reference<>())
    .def("roads", 
        &carla::lanenetwork::LaneNetwork::Roads,
        return_internal_reference<>())
    .def("lanes", 
        &carla::lanenetwork::LaneNetwork::Lanes,
        return_internal_reference<>())
    .def("lane_connections", 
        &carla::lanenetwork::LaneNetwork::LaneConnections,
        return_internal_reference<>())
    .def("get_road_length", 
        &carla::lanenetwork::LaneNetwork::GetRoadLength, 
        (arg("road")))
    .def("get_road_direction", 
        &carla::lanenetwork::LaneNetwork::GetRoadDirection, 
        (arg("road")))
    .def("get_lane_direction", 
        &carla::lanenetwork::LaneNetwork::GetLaneDirection, 
        (arg("lane")))
    .def("get_lane_start", 
        &carla::lanenetwork::LaneNetwork::GetLaneStart, 
        (arg("lane"), arg("offset") = 0.0f))
    .def("get_lane_end", 
        &carla::lanenetwork::LaneNetwork::GetLaneEnd, 
        (arg("lane"), arg("offset") = 0.0f))
    .def("get_incoming_lane_connection_ids", 
        &carla::lanenetwork::LaneNetwork::GetIncomingLaneConnectionIds, 
        (arg("lane")),
        return_internal_reference<>())
    .def("get_outgoing_lane_connection_ids", 
        &carla::lanenetwork::LaneNetwork::GetOutgoingLaneConnectionIds, 
        (arg("lane")),
        return_internal_reference<>())
    .def("get_lane_start_min_offset", 
        &carla::lanenetwork::LaneNetwork::GetLaneStartMinOffset,
        (arg("lane")))
    .def("get_lane_end_min_offset", 
        &carla::lanenetwork::LaneNetwork::GetLaneEndMinOffset, 
        (arg("lane")))
    .def("create_route_map",
        &carla::lanenetwork::LaneNetwork::CreateRouteMap)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<carla::lanenetwork::RoutePoint>("RoutePoint", 
      init<int64_t, float>(
        (arg("id"), arg("offset"))))
    .def(init<const carla::lanenetwork::RoutePoint &>((arg("rhs"))))
    .def("__eq__", &carla::lanenetwork::RoutePoint::operator==)
    .def("__ne__", &carla::lanenetwork::RoutePoint::operator!=)
    .def_readwrite("id", &carla::lanenetwork::RoutePoint::segment_id)
    .def_readwrite("offset", &carla::lanenetwork::RoutePoint::offset)
    .def(self_ns::str(self_ns::self))
  ;
  
  class_<std::vector<carla::lanenetwork::RoutePoint>>("vector_of_route_point")
       .def(vector_indexing_suite<std::vector<carla::lanenetwork::RoutePoint>>())
  ;
  
  class_<carla::lanenetwork::RouteMap>("RouteMap")
    .def(init<const carla::lanenetwork::RouteMap &>((arg("rhs"))))
    .def("get_position", &carla::lanenetwork::RouteMap::GetPosition)
    .def("rand_route_point", &carla::lanenetwork::RouteMap::RandRoutePoint)
    .def("get_nearest_route_point", &carla::lanenetwork::RouteMap::GetNearestRoutePoint)
    .def("get_next_route_points", &carla::lanenetwork::RouteMap::GetNextRoutePoints)
    .def(self_ns::str(self_ns::self))
  ;
  
}
