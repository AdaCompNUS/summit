#include "CrowdController.h"

namespace carla {
namespace crowd {

  CrowdController::CrowdController(SharedPtr<client::World> world, SharedPtr<lanenetwork::LaneNetwork> lane_network, 
    const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max)
  : _world(std::move(world)),
  _lane_network(lane_network),
  _bounds_min(bounds_min),
  _bounds_max(bounds_max) {

  _world->OnTick(std::bind(&CrowdController::OnWorldTick, this, std::placeholders::_1));
}

void CrowdController::OnWorldTick(const client::WorldSnapshot& world_snapshot) {
  
}

}
}
