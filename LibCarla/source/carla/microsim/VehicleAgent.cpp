#include "VehicleAgent.h"
#include "carla/geom/Math.h"

namespace carla {
namespace microsim {

VehicleAgent VehicleAgent::Step(float delta, float control_speed, float control_steer) const {
  
  geom::Vector2D new_position;
  geom::Vector2D new_heading;
  std::vector<sumonetwork::RoutePoint> new_path;

  if (std::abs(control_steer) < 0.00001) {
    new_position = _position + _heading * control_speed * delta;
    new_heading = _heading;
  } else {
    float radius = _wheel_base / std::tan(carla::geom::Math::ToRadians(control_steer));
    geom::Vector2D center = _position + radius * _heading.Rotate(carla::geom::Math::ToRadians(90.0f));
    float angle = control_speed * delta / radius;
    new_position = center + (_position - center).Rotate(angle);
    new_heading = _heading.Rotate(angle).MakeUnitVector();
  }

  if (_path.size() > 0) {
    size_t cut_index = 0;
    float min_offset = -1;
    size_t min_offset_index = 0;
    
    for (size_t i = 0; i < (_path.size() + 1) / 2; i++) {
      float offset = (new_position - _sumo_network->GetRoutePointPosition(_path[i])).Length();
      if (min_offset < 0 || offset < min_offset) {
        min_offset = offset;
        min_offset_index = i;
      }
      if (offset <= 1.0) {
        cut_index = i + 1;
      }
    }

    if (min_offset > 1.0) {
      cut_index = min_offset_index;
    }
    
    new_path = std::vector<sumonetwork::RoutePoint>(_path.begin() + static_cast<long>(cut_index), _path.end());
  }

  return VehicleAgent(_sumo_network, _extent_min, _extent_max, _wheel_base, _max_speed, 
      std::move(new_position), std::move(new_heading), std::move(new_path));

}

}
}
