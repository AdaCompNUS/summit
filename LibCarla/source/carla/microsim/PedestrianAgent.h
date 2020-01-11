#pragma once

#include "carla/geom/Vector2D.h"
#include "carla/microsim/Agent.h"
#include "carla/sidewalk/Sidewalk.h"

namespace carla {
namespace microsim {

class PedestrianAgent : public Agent {

public:

  PedestrianAgent(const sidewalk::Sidewalk* sidewalk,
      float radius, float max_speed, 
      const geom::Vector2D& position, 
      const std::vector<sidewalk::SidewalkRoutePoint>& path = {})
    : _sidewalk(sidewalk),
    _radius(radius), _max_speed(max_speed), 
    _position(position), _path(path) { }

  float Radius() const { return _radius; }
  float MaxSpeed() const { return _max_speed; }
  const geom::Vector2D& Position() const { return _position; }
  const std::vector<sidewalk::SidewalkRoutePoint>& Path() const { return _path; }

  PedestrianAgent Step(const geom::Vector2D& control_velocity, float delta) const;

private:
  const sidewalk::Sidewalk* const _sidewalk;
  const float _radius;
  const float _max_speed;
  const geom::Vector2D _position;
  const std::vector<sidewalk::SidewalkRoutePoint> _path;

};

}
}
