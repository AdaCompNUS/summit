#include "PedestrianAgent.h"

namespace carla {
namespace microsim {

void PedestrianAgent::Step(float delta, const geom::Vector2D& control_velocity) {

  this->position = this->position + delta * control_velocity;

  if (this->path.size() > 0) {
    size_t cut_index = 0;
    float min_offset = -1;
    size_t min_offset_index = 0;
    
    for (size_t i = 0; i < (this->path.size() + 1) / 2; i++) {
      float offset = (this->position - this->sidewalk->GetRoutePointPosition(this->path[i])).Length();
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
    
    this->path = std::vector<sidewalk::SidewalkRoutePoint>(this->path.begin() + static_cast<long>(cut_index), this->path.end());
  }
}

}
}
