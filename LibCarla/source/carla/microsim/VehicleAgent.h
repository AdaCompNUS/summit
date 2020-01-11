#pragma once

#include "carla/geom/Vector2D.h"
#include "carla/microsim/Agent.h"
#include "carla/sumonetwork/SumoNetwork.h"

namespace carla {
namespace microsim {

struct VehicleAttributes {
  geom::Vector2D extent_min;
  geom::Vector2D extent_max;
  float wheel_base;
  float max_speed;
};

class VehicleAgent : public Agent {

public:

  const sumonetwork::SumoNetwork* sumo_network;
  VehicleAttributes attributes;
  geom::Vector2D position;
  geom::Vector2D heading;
  std::vector<sumonetwork::RoutePoint> path;

  VehicleAgent(
      const sumonetwork::SumoNetwork* sumo_network,
      const VehicleAttributes& attributes,
      const geom::Vector2D& position, const geom::Vector2D& heading,
      const std::vector<sumonetwork::RoutePoint>& path = {})
    : sumo_network(sumo_network), attributes(attributes),
    position(position), heading(heading), path(path) { }

  void Step(float delta, float control_speed, float control_steer);

};

}
}
