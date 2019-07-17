#include <carla/geom/Vector2D.h>
#include <carla/gamma/Vector2.h>
#include <carla/gamma/RVOSimulator.h>
#include <boost/python/register_ptr_to_python.hpp>

void export_gamma() {
  using namespace boost::python;
  using namespace RVO;
  using namespace carla;

  class_<RVOSimulator>("RVOSimulator", no_init)
    .def("__init__", 
        make_constructor(+[]() {
          return MakeShared<RVOSimulator>();
        }))
    .def("set_agent_defaults",
        +[](RVOSimulator& self, float neighbour_dist, size_t max_neighbours, float time_horizon,
            float time_horizon_obst, float radius, float max_speed) {
          self.setAgentDefaults(neighbour_dist, max_neighbours, time_horizon, time_horizon_obst,
              radius, max_speed);
        })
    .def("add_agent", 
        +[](RVOSimulator& self, const geom::Vector2D& position) {
          return static_cast<int>(self.addAgent(
                Vector2(position.x, position.y)));
        })
    .def("set_agent_position",
        +[](RVOSimulator& self, int agent_no, const geom::Vector2D& position) {
          self.setAgentPosition(
              static_cast<size_t>(agent_no),
              Vector2(position.x, position.y));
        })
    .def("set_agent_pref_velocity",
        +[](RVOSimulator& self, int agent_no, const geom::Vector2D& velocity) {
          self.setAgentPrefVelocity(
              static_cast<size_t>(agent_no),
              Vector2(velocity.x, velocity.y));
        })
    .def("do_step", &RVOSimulator::doStep)
    .def("get_velocity", 
        +[](RVOSimulator& self, int agent_no) {
          Vector2 velocity = self.getAgentVelocity(static_cast<size_t>(agent_no));
          return geom::Vector2D(velocity.x(), velocity.y());
        })
  ;
}
