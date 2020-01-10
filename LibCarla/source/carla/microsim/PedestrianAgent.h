#pragma once

#include "carla/geom/Vector2D.h"
#include "carla/microsim/Agent.h"

namespace carla {
namespace microsim {

class PedestrianAgent : public Agent {

public:

  PedestrianAgent(float radius, float max_speed, const geom::Vector2D& position)
    : _radius(radius), _max_speed(max_speed), _position(position) { }

  float Radius() const { return _radius; }
  float MaxSpeed() const { return _max_speed; }
  const geom::Vector2D& Position() const { return _position; }

  PedestrianAgent Step(const geom::Vector2D& control_velocity, float delta) const;

private:
  const float _radius;
  const float _max_speed;
  const geom::Vector2D _position;

};

}
}
