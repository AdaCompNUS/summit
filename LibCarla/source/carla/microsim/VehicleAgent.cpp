#include "VehicleAgent.h"
#include "carla/geom/Math.h"

namespace carla {
namespace microsim {

VehicleAgent VehicleAgent::Step(float delta, float control_speed, float control_steer) const {
  if (control_steer < 0.00001) {
    return VehicleAgent(_extent_min, _extent_max, _wheel_base, _max_speed,
        _position + _heading * control_speed * delta, _heading);
  } else {
    float radius = _wheel_base / std::tan(carla::geom::Math::ToRadians(control_steer));
    geom::Vector2D center = _position + radius * _heading.Rotate(carla::geom::Math::ToRadians(90.0f));
    float angle = control_speed * delta / radius;

    return VehicleAgent(_extent_min, _extent_max, _wheel_base, _max_speed,
        center + (_position - center).Rotate(angle), _heading.Rotate(angle).MakeUnitVector());
  }
}

}
}
