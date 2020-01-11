#include <carla/geom/Vector2D.h>
#include <carla/gamma/Vector2.h>
#include <carla/gamma/RVOSimulator.h>
#include <boost/python/register_ptr_to_python.hpp>

void export_gamma() {
  using namespace boost::python;
  using namespace RVO;
  using namespace carla;

  static const auto GeomToGamma = [](const geom::Vector2D& v) { return Vector2(v.x, v.y); };
  static const auto GammaToGeom = [](const Vector2& v) { return geom::Vector2D(v.x(), v.y()); };
  
  enum_<AgentBehaviorType>("AgentBehaviorType")
    .value("Gamma", AgentBehaviorType::Gamma)
    .value("SimplifiedGamma", AgentBehaviorType::SimplifiedGamma)
  ;

  class_<AgentParams>("AgentParams", init<>())
    .def("get_default", 
        &AgentParams::getDefaultAgentParam, 
        return_value_policy<reference_existing_object>())
    .staticmethod("get_default")
    .add_property("position", 
        +[](AgentParams& self) {
          return GammaToGeom(self.position);
        },
        +[](AgentParams& self, const geom::Vector2D& position) {
          self.position = GeomToGamma(position);
        })
    .def_readwrite("neighbor_dist", &AgentParams::neighborDist)
    .def_readwrite("time_horizon", &AgentParams::timeHorizon)
    .def_readwrite("time_horizon_obst", &AgentParams::timeHorizonObst)
    .def_readwrite("radius", &AgentParams::radius)
    .def_readwrite("max_speed", &AgentParams::maxSpeed)
    .add_property("velocity",
        +[](AgentParams& self) {
          return GammaToGeom(self.velocity);
        },
        +[](AgentParams& self, const geom::Vector2D& velocity) {
          self.velocity = GeomToGamma(velocity);
        })
    .def_readwrite("tag", 
        &AgentParams::tag)
    .def_readwrite("max_tracking_angle", &AgentParams::max_tracking_angle)
    .def_readwrite("len_ref_to_front", &AgentParams::len_ref_to_front)
    .def_readwrite("len_ref_to_side", &AgentParams::len_ref_to_side)
    .def_readwrite("len_rear_axle_to_front_axle", &AgentParams::len_rear_axle_to_front_axle)
    .def_readwrite("error_bound", &AgentParams::error_bound)
    .def_readwrite("pref_speed", &AgentParams::pref_speed)
    .def_readwrite("r_front", &AgentParams::r_front)
    .def_readwrite("r_rear", &AgentParams::r_rear)
    .def_readwrite("res_dec_rate", &AgentParams::res_dec_rate)
  ;


  class_<RVOSimulator>("RVOSimulator", init<>())
    // RVO2
    .def("add_agent", 
        +[](RVOSimulator& self, const AgentParams& params, int agent_id) {
          return static_cast<int>(self.addAgent(params, agent_id));
        })
    .def("add_obstacle",
        +[](RVOSimulator& self, const list& vertices_py) {
          std::vector<geom::Vector2D> vertices{
            stl_input_iterator<geom::Vector2D>(vertices_py),
            stl_input_iterator<geom::Vector2D>()};

          std::vector<Vector2> vertices_gamma;
          for (const geom::Vector2D& vertex : vertices) {
            vertices_gamma.emplace_back(GeomToGamma(vertex));
          }
          self.addObstacle(vertices_gamma);
        })
    .def("process_obstacles", &RVOSimulator::processObstacles)
    .def("do_step", &RVOSimulator::doStep)
    .def("set_agent_position",
        +[](RVOSimulator& self, int agent_no, const geom::Vector2D& position) {
          self.setAgentPosition(
              static_cast<size_t>(agent_no),
              GeomToGamma(position));
        })
    .def("set_agent_pref_velocity",
        +[](RVOSimulator& self, int agent_no, const geom::Vector2D& velocity) {
          self.setAgentPrefVelocity(
              static_cast<size_t>(agent_no),
              GeomToGamma(velocity));
        })
    .def("get_agent_velocity", 
        +[](RVOSimulator& self, int agent_no) {
          return GammaToGeom(self.getAgentVelocity(static_cast<size_t>(agent_no)));
        })
    
    // GAMMA
    .def("set_agent_id", &RVOSimulator::setAgentID)
    .def("get_agent_id", &RVOSimulator::getAgentID)
    .def("get_agent_tag", &RVOSimulator::getAgentTag)
    .def("get_agent_heading",
        +[](RVOSimulator& self, int agent_no) {
          return GammaToGeom(self.getAgentHeading(agent_no));
        })
    .def("set_agent_bounding_box_corners",
        +[](RVOSimulator& self, int agent_no, const list& corners_py) {
          std::vector<geom::Vector2D> corners{
            stl_input_iterator<geom::Vector2D>(corners_py),
            stl_input_iterator<geom::Vector2D>()};

          std::vector<Vector2> corners_gamma;
          for (const geom::Vector2D& corner : corners) {
            corners_gamma.emplace_back(GeomToGamma(corner));
          }
          self.setAgentBoundingBoxCorners(agent_no, corners_gamma);
        })
    .def("set_agent_heading",
        +[](RVOSimulator& self, int agent_no, const geom::Vector2D& heading) {
          self.setAgentHeading(
              agent_no,
              GeomToGamma(heading));
        })
    .def("set_agent_max_tracking_angle", &RVOSimulator::setAgentMaxTrackingAngle)
    .def("set_agent_attention_radius", &RVOSimulator::setAgentAttentionRadius)
    .def("set_agent_res_dec_rate", &RVOSimulator::setAgentResDecRate)
    .def("set_agent", &RVOSimulator::setAgent)
    .def("set_agent_velocity",
        +[](RVOSimulator& self, int agent_no, const geom::Vector2D& velocity) {
          self.setAgentVelocity(
              static_cast<size_t>(agent_no),
              GeomToGamma(velocity));
        })
    .def("set_agent_velocity_convex",
        +[](RVOSimulator& self, int agent_no, const list& velocity_convex_py) {
          std::vector<geom::Vector2D> velocity_convex{
            stl_input_iterator<geom::Vector2D>(velocity_convex_py),
            stl_input_iterator<geom::Vector2D>()};

          std::vector<Vector2> velocity_convex_gamma;
          for (const geom::Vector2D& v : velocity_convex) {
            velocity_convex_gamma.emplace_back(GeomToGamma(v));
          }
          
          self.setAgentVelocityConvex(agent_no, velocity_convex_gamma);
        })
    .def("set_agent_lane_constraints",
        +[](RVOSimulator& self, size_t agent_no, bool left_lane_constrained, bool right_lane_constrained) {
          self.setAgentLaneConstraints(agent_no, left_lane_constrained, right_lane_constrained);
        })
    .def("set_agent_path_forward",
        +[](RVOSimulator& self, size_t agent_no, const geom::Vector2D& path_forward) {
          self.setAgentPathForward(
              agent_no,
              GeomToGamma(path_forward));
        })
    .def("set_agent_behavior_type",
        +[](RVOSimulator& self, int agent_no, AgentBehaviorType behavior_type) {
          self.setAgentBehaviorType(
              agent_no,
              behavior_type);
        })
  ;
}
