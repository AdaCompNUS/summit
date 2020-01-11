#include "VehicleAgent.h"
#include "carla/geom/Math.h"

namespace carla {
namespace microsim {

void VehicleAgent::Step(float delta, float control_speed, float control_steer) {
  
  if (std::abs(control_steer) < 0.00001) {
    this->position = this->position + this->heading * control_speed * delta;
  } else {
    float radius = this->attributes.wheel_base / std::tan(carla::geom::Math::ToRadians(control_steer));
    geom::Vector2D center = this->position + radius * this->heading.Rotate(carla::geom::Math::ToRadians(90.0f));
    float angle = control_speed * delta / radius;
    this->position = center + (this->position - center).Rotate(angle);
    this->heading = this->heading.Rotate(angle).MakeUnitVector();
  }

  if (this->path.size() > 0) {
    size_t cut_index = 0;
    float min_offset = -1;
    size_t min_offset_index = 0;
    
    for (size_t i = 0; i < (this->path.size() + 1) / 2; i++) {
      float offset = (this->position - this->sumo_network->GetRoutePointPosition(this->path[i])).Length();
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
    
    this->path = std::vector<sumonetwork::RoutePoint>(this->path.begin() + static_cast<long>(cut_index), this->path.end());
  }

}

}
}
