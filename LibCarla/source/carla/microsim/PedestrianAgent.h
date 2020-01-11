#pragma once

#include "carla/geom/Vector2D.h"
#include "carla/microsim/Agent.h"
#include "carla/sidewalk/Sidewalk.h"

namespace carla {
namespace microsim {

struct PedestrianAttributes {
  float radius;
  float max_speed;
};

class PedestrianAgent : public Agent {

public:

  const sidewalk::Sidewalk* sidewalk;
  PedestrianAttributes attributes;
  geom::Vector2D position;
  std::vector<sidewalk::SidewalkRoutePoint> path;

  PedestrianAgent(const sidewalk::Sidewalk* sidewalk,
      const PedestrianAttributes& attributes,
      const geom::Vector2D& position, 
      const std::vector<sidewalk::SidewalkRoutePoint>& path = {})
    : sidewalk(sidewalk), attributes(attributes), 
    position(position), path(path) { }

  void Step(float delta, const geom::Vector2D& control_velocity);

};

}
}
