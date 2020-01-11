#include "PedestrianAgent.h"

namespace carla {
namespace microsim {

PedestrianAgent PedestrianAgent::Step(const geom::Vector2D& control_velocity, float delta) const {

  geom::Vector2D new_position = _position + delta * control_velocity;
  std::vector<sidewalk::SidewalkRoutePoint> new_path;

  if (_path.size() > 0) {
    size_t cut_index = 0;
    float min_offset = -1;
    size_t min_offset_index = 0;
    
    for (size_t i = 0; i < (_path.size() + 1) / 2; i++) {
      float offset = (new_position - _sidewalk->GetRoutePointPosition(_path[i])).Length();
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
    
    new_path = std::vector<sidewalk::SidewalkRoutePoint>(_path.begin() + static_cast<long>(cut_index), _path.end());
  }
  
  return PedestrianAgent(_sidewalk, _radius, _max_speed, std::move(new_position), std::move(new_path));
}

}
}
