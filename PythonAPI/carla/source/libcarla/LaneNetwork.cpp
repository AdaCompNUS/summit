#include <carla/lanenetwork/LaneNetwork.h>
#include <carla/geom/Vector2D.h>
#include <cstdint>
#include "unordered_map_indexing_suite.hpp"

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
  
  class_<std::unordered_map<int64_t, carla::lanenetwork::Node>>("map_of_node")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, carla::lanenetwork::Node>>());
  
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

  class_<std::unordered_map<int64_t, carla::lanenetwork::Road>>("map_of_road")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, carla::lanenetwork::Road>>());

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
  
  class_<std::unordered_map<int64_t, carla::lanenetwork::Lane>>("map_of_lane")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, carla::lanenetwork::Lane>>());
  
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
  
  class_<std::unordered_map<int64_t, carla::lanenetwork::LaneConnection>>("map_of_lane_connection")
       .def(unordered_map_indexing_suite<std::unordered_map<int64_t, carla::lanenetwork::LaneConnection>>());
  
  class_<carla::lanenetwork::LaneNetwork>("LaneNetwork", 
      init<float>(
        (arg("lane_width") = 3.0f)))
    .def(init<const carla::lanenetwork::LaneNetwork &>((arg("rhs"))))
    .def("load", STATIC_CALL_WITHOUT_GIL_1(carla::lanenetwork::LaneNetwork, Load, const std::string), (arg("path"))) 
    .staticmethod("load")
    .def("nodes", 
        CONST_CALL_WITHOUT_GIL(carla::lanenetwork::LaneNetwork, Nodes))
    .def("lanes", 
        CONST_CALL_WITHOUT_GIL(carla::lanenetwork::LaneNetwork, Roads))
    .def("roads", 
        CONST_CALL_WITHOUT_GIL(carla::lanenetwork::LaneNetwork, Lanes))
    .def("lane_connections", 
        CONST_CALL_WITHOUT_GIL(carla::lanenetwork::LaneNetwork, LaneConnections))
    .def("get_road_length", 
        CONST_CALL_WITHOUT_GIL_1(carla::lanenetwork::LaneNetwork, GetRoadLength, const carla::lanenetwork::Road &), 
        (arg("road")))
    .def("get_road_direction", 
        CONST_CALL_WITHOUT_GIL_1(carla::lanenetwork::LaneNetwork, GetRoadDirection, const carla::lanenetwork::Road &), 
        (arg("road")))
    .def("get_lane_direction", 
        CONST_CALL_WITHOUT_GIL_1(carla::lanenetwork::LaneNetwork, GetLaneDirection, const carla::lanenetwork::Lane &), 
        (arg("lane")))
    .def("get_lane_start", 
        CONST_CALL_WITHOUT_GIL_2(carla::lanenetwork::LaneNetwork, GetLaneStart, const carla::lanenetwork::Lane &, float), 
        (arg("lane"), arg("offset") = 0.0f))
    .def("get_lane_end", 
        CALL_WITHOUT_GIL_2(carla::lanenetwork::LaneNetwork, GetLaneEnd, const carla::lanenetwork::Lane &, float), 
        (arg("lane"), arg("offset") = 0.0f))
    .def("get_incoming_lane_connection_ids", 
        CALL_WITHOUT_GIL_1(carla::lanenetwork::LaneNetwork, GetIncomingLaneConnectionIds, const carla::lanenetwork::Lane &), 
        (arg("lane")))
    .def("get_outgoing_lane_connection_ids", 
        CALL_WITHOUT_GIL_1(carla::lanenetwork::LaneNetwork, GetOutgoingLaneConnectionIds, const carla::lanenetwork::Lane &), 
        (arg("lane")))
    .def("get_lane_start_min_offset", 
        CALL_WITHOUT_GIL_1(carla::lanenetwork::LaneNetwork, GetLaneStartMinOffset, const carla::lanenetwork::Lane &), 
        (arg("lane")))
    .def("get_lane_end_min_offset", 
        CALL_WITHOUT_GIL_1(carla::lanenetwork::LaneNetwork, GetLaneEndMinOffset, const carla::lanenetwork::Lane &), 
        (arg("lane")))
    .def(self_ns::str(self_ns::self))
  ;
}
