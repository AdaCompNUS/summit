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

  class_<PedestrianAgent>("PedestrianAgent", no_init)
    .def(init<float, float, const geom::Vector2D&>())
    .add_property("radius", &PedestrianAgent::Radius)
    .add_property("max_speed", &PedestrianAgent::MaxSpeed)
    .add_property("position", 
        make_function(&PedestrianAgent::Position, return_internal_reference<>()))
  ;

  class_<VehicleAgent>("VehicleAgent", no_init)
    .def(init<const geom::Vector2D&, const geom::Vector2D&, float, float, const geom::Vector2D&, const geom::Vector2D&>())
    .add_property("extent_min", 
        make_function(&VehicleAgent::ExtentMin, return_internal_reference<>()))
    .add_property("extent_max", 
        make_function(&VehicleAgent::ExtentMax, return_internal_reference<>()))
    .add_property("wheel_base", &VehicleAgent::WheelBase)
    .add_property("max_speed", &VehicleAgent::MaxSpeed)
    .add_property("position",
        make_function(&VehicleAgent::Position, return_internal_reference<>()))
    .add_property("heading",
        make_function(&VehicleAgent::Heading, return_internal_reference<>()))
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
    .add_property("sumo_network", 
        make_function(&Simulator::SumoNetwork, return_internal_reference<>()))
    .add_property("sidewalk", 
        make_function(&Simulator::Sidewalk, return_internal_reference<>()))
    .add_property("ego_agent", 
        make_function(&Simulator::EgoAgent, return_internal_reference<>()))
    .add_property("exo_agents", 
        make_function(&Simulator::ExoAgents, return_internal_reference<>()))
    .def("step", &Simulator::Step)
  ;

}
