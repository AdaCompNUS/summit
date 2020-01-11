#include <carla/microsim/Agent.h>
#include <carla/microsim/PedestrianAgent.h>
#include <carla/microsim/VehicleAgent.h>
#include <carla/microsim/Simulator.h>
#include <carla/sidewalk/Sidewalk.h>
#include <carla/sumonetwork/SumoNetwork.h>
#include <cstdint>

namespace carla {
namespace microsim {

inline bool operator==(const Agent& lhs, const Agent& rhs) {
  return &lhs == &rhs;
}

inline bool operator!=(const Agent& lhs, const Agent& rhs) {
  return &lhs != &rhs;
}

}
}

void export_microsim() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::microsim;

  class_<Agent>("Agent", no_init);
  class_<std::vector<Agent>>("vector_of_agent")
      .def(boost::python::vector_indexing_suite<std::vector<Agent>>())
  ;
  
  class_<PedestrianAttributes>("PedestrianAttributes", init<>())
    .def_readwrite("radius", &PedestrianAttributes::radius)
    .def_readwrite("max_speed", &PedestrianAttributes::max_speed)
  ;

  class_<PedestrianAgent>("PedestrianAgent", no_init)
    .def("__init__", make_constructor(
          +[](const sidewalk::Sidewalk* sidewalk, const PedestrianAttributes& attributes, const geom::Vector2D& position) {
            return boost::shared_ptr<PedestrianAgent>(new PedestrianAgent(sidewalk, attributes, position));
          }))
    .def("__init__", make_constructor(
          +[](const sidewalk::Sidewalk* sidewalk, const PedestrianAttributes& attributes, const geom::Vector2D& position,
              const std::vector<sidewalk::SidewalkRoutePoint>& path) {
            return boost::shared_ptr<PedestrianAgent>(new PedestrianAgent(sidewalk, attributes, position, path));
          }))
    .def("__init__", make_constructor(
          +[](const sidewalk::Sidewalk* sidewalk, const PedestrianAttributes& attributes, const geom::Vector2D& position,
              const list& path_py) {
            std::vector<sidewalk::SidewalkRoutePoint> path{
              stl_input_iterator<sidewalk::SidewalkRoutePoint>(path_py),
              stl_input_iterator<sidewalk::SidewalkRoutePoint>()};
            return boost::shared_ptr<PedestrianAgent>(new PedestrianAgent(sidewalk, attributes, position, path));
          }))
    .def_readwrite("attributes", &PedestrianAgent::attributes)
    .def_readwrite("position", &PedestrianAgent::position)
    .def_readwrite("path", &PedestrianAgent::path)
  ;

  class_<VehicleAttributes>("VehicleAttributes", init<>())
    .def_readwrite("extent_min", &VehicleAttributes::extent_min)
    .def_readwrite("extent_max", &VehicleAttributes::extent_max)
    .def_readwrite("wheel_base", &VehicleAttributes::wheel_base)
    .def_readwrite("max_speed", &VehicleAttributes::max_speed)
  ;

  class_<VehicleAgent>("VehicleAgent", no_init)
    .def("__init__", make_constructor(
          +[](const sumonetwork::SumoNetwork* sumo_network, const VehicleAttributes& attributes,
              const geom::Vector2D& position, const geom::Vector2D& heading) {
            return boost::shared_ptr<VehicleAgent>(new VehicleAgent(sumo_network, attributes, position, heading));
          }))
    .def("__init__", make_constructor(
          +[](const sumonetwork::SumoNetwork* sumo_network, const VehicleAttributes& attributes, 
              const geom::Vector2D& position, const geom::Vector2D& heading, 
              const std::vector<sumonetwork::RoutePoint>& path) {
            return boost::shared_ptr<VehicleAgent>(new VehicleAgent(sumo_network, attributes, position, heading, path));
          }))
    .def("__init__", make_constructor(
          +[](const sumonetwork::SumoNetwork* sumo_network, const VehicleAttributes& attributes, 
              const geom::Vector2D& position, const geom::Vector2D& heading, 
              const list& path_py) {
            std::vector<sumonetwork::RoutePoint> path{
              stl_input_iterator<sumonetwork::RoutePoint>(path_py),
              stl_input_iterator<sumonetwork::RoutePoint>()};
            return boost::shared_ptr<VehicleAgent>(new VehicleAgent(sumo_network, attributes, position, heading, path));
          }))
    .def_readwrite("attributes", &VehicleAgent::attributes)
    .def_readwrite("position", &VehicleAgent::position)
    .def_readwrite("heading", &VehicleAgent::heading)
    .def_readwrite("path", &VehicleAgent::path)
  ;

  class_<Simulator>("Simulator", no_init)
    .def("__init__", make_constructor(
          +[](const sumonetwork::SumoNetwork* sumo_network, const sidewalk::Sidewalk* sidewalk, const VehicleAgent& ego_agent) {
            return boost::shared_ptr<Simulator>(new Simulator(sumo_network, sidewalk, ego_agent));
          }))
    .def("__init__", make_constructor(
          +[](const sumonetwork::SumoNetwork* sumo_network, const sidewalk::Sidewalk* sidewalk, const VehicleAgent& ego_agent, const std::vector<Agent>& exo_agents) {
            return boost::shared_ptr<Simulator>(new Simulator(sumo_network, sidewalk, ego_agent, exo_agents));
          }))
    .def("__init__", make_constructor(
          +[](const sumonetwork::SumoNetwork* sumo_network, const sidewalk::Sidewalk* sidewalk, const VehicleAgent& ego_agent, const list& exo_agents_py) {
            std::vector<Agent> exo_agents{
              stl_input_iterator<Agent>(exo_agents_py),
              stl_input_iterator<Agent>()};
            return boost::shared_ptr<Simulator>(new Simulator(sumo_network, sidewalk, ego_agent, exo_agents));
          }))
    .def_readwrite("sumo_network", &Simulator::sumo_network)
    .def_readwrite("sidewalk", &Simulator::sidewalk)
    .def_readwrite("ego_agent", &Simulator::ego_agent)
    .def_readwrite("exo_agents", &Simulator::exo_agents)
    .def("step", &Simulator::Step)
  ;

}
