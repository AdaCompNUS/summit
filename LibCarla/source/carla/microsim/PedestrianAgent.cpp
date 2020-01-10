#include "PedestrianAgent.h"

namespace carla {
namespace microsim {

PedestrianAgent PedestrianAgent::Step(const geom::Vector2D& control_velocity, float delta) const {
  return PedestrianAgent(_radius, _max_speed, _position + delta * control_velocity);
}

}
}
