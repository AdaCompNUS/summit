#include "carla/microsim/Simulator.h"

namespace carla {
namespace microsim {

void Simulator::Step(float delta, float ego_control_speed, float ego_control_steer) {
  this->ego_agent.Step(delta, ego_control_speed, ego_control_steer);
}

/*
Simulator Simulator::RefreshExoAgents(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, int min_path_points,
    int max_num_pedestrian, int max_num_bike, int max_num_car,
    float pedestrian_clearance, float bike_clearance, float car_clearance, float ego_clearance) const {

}
*/

}
}
