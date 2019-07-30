#include "SumoNetwork.h"
#include <boost/algorithm/string.hpp>
#include <pugixml/pugixml.hpp>
#include <string>

namespace carla {
namespace sumonetwork {

static std::vector<geom::Vector2D> parse_position_list(const std::string& s) {
  if (s.empty()) return {};

  std::vector<std::string> split_list;
  boost::split(
      split_list, 
      s,
      boost::is_any_of(", "),
      boost::token_compress_on);

  std::vector<geom::Vector2D> position_list;
  for (size_t i = 0; i < split_list.size(); i += 2) {
    position_list.emplace_back(
        std::stof(split_list[i]), 
        std::stof(split_list[i + 1]));
  }

  return position_list;
}

static std::vector<std::string> parse_string_list(const std::string& s) {
  if (s.empty()) return {};
  
  std::vector<std::string> split_list;
  boost::split(
      split_list, 
      s,
      boost::is_any_of(" "),
      boost::token_compress_on);
  return split_list;
}

SumoNetwork SumoNetwork::Load(const std::string& data) {
  pugi::xml_document xml;
  xml.load_string(data.c_str());

  SumoNetwork sumo_network;
  
  pugi::xml_node net_node = xml.child("net");
  
  for (pugi::xml_node edge_node : net_node.children("edge")) {
    Edge edge;
    edge.id = edge_node.attribute("id").value();
    edge.from = edge_node.attribute("from").value();
    edge.to = edge_node.attribute("to").value();
    edge.priority = edge_node.attribute("priority").as_int();
    std::string function = edge_node.attribute("function").value();
    if (function == "internal") edge.function = Function::Internal;
    else if (function == "connector") edge.function = Function::Connector;
    else if (function == "crossing") edge.function = Function::Crossing;
    else if (function == "walkingarea") edge.function = Function::WalkingArea;
    else edge.function = Function::Normal;
    for (pugi::xml_node lane_node : edge_node.children("lane")) {
      Lane lane;
      lane.id = lane_node.attribute("id").value();
      lane.index = lane_node.attribute("index").as_uint();
      lane.speed = lane_node.attribute("speed").as_double();
      lane.length = lane_node.attribute("length").as_double();
      lane.shape = parse_position_list(lane_node.attribute("shape").value());
      edge.lanes.emplace_back(std::move(lane));
    }
    sumo_network._edges.emplace(edge.id, std::move(edge));
  }

  for (pugi::xml_node junction_node : net_node.children("junction")) {
    Junction junction;
    junction.id = junction_node.attribute("id").value();
    junction.x = junction_node.attribute("x").as_double();
    junction.y = junction_node.attribute("y").as_double();
    junction.inc_lanes = parse_string_list(junction_node.attribute("incLanes").value());
    junction.int_lanes = parse_string_list(junction_node.attribute("intLanes").value());
    junction.shape = parse_position_list(junction_node.attribute("shape").value());
    sumo_network._junctions.emplace(junction.id, std::move(junction));
  }

  for (pugi::xml_node connection_node : net_node.children("connection")) {
    Connection connection;
    connection.from = connection_node.attribute("from").value();
    connection.to = connection_node.attribute("to").value();
    connection.from_lane = connection_node.attribute("from_lane").as_uint();
    connection.to_lane = connection_node.attribute("to_lane").as_uint();
    connection.via = connection_node.attribute("via").value();
    sumo_network._connections.emplace_back(std::move(connection));
  }

  return sumo_network;
}

}
}
