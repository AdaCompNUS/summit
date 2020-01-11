#pragma once

#include "carla/geom/Vector2D.h"
#include "carla/microsim/Agent.h"
#include "carla/sumonetwork/SumoNetwork.h"

namespace carla {
namespace microsim {

class VehicleAgent : public Agent {

public:

  VehicleAgent(
      const sumonetwork::SumoNetwork* sumo_network,
      const geom::Vector2D& extent_min, const geom::Vector2D& extent_max, 
      float wheel_base, float max_speed, 
      const geom::Vector2D& position, const geom::Vector2D& heading,
      const std::vector<sumonetwork::RoutePoint>& path = {})
    : _sumo_network(sumo_network),
    _extent_min(extent_min), _extent_max(extent_max),
    _wheel_base(wheel_base), _max_speed(max_speed),
    _position(position), _heading(heading),
    _path(path) { }

  const geom::Vector2D& ExtentMin() const { return _extent_min; }
  const geom::Vector2D& ExtentMax() const { return _extent_max; }
  float WheelBase() const { return _wheel_base; }
  float MaxSpeed() const { return _max_speed; }
  const geom::Vector2D& Position() const { return _position; }
  const geom::Vector2D& Heading() const { return _heading; }
  const std::vector<sumonetwork::RoutePoint>& Path() const { return _path; }

  VehicleAgent Step(float delta, float control_speed, float control_steer) const;

private:

  const sumonetwork::SumoNetwork* const _sumo_network;
  const geom::Vector2D _extent_min;
  const geom::Vector2D _extent_max;
  const float _wheel_base;
  const float _max_speed;
  const geom::Vector2D _position;
  const geom::Vector2D _heading;
  const std::vector<sumonetwork::RoutePoint> _path;

};

}
}
