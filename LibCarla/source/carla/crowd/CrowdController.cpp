#include "CrowdController.h"

namespace carla {
namespace crowd {

CrowdController::CrowdController(SharedPtr<client::World> world, SharedPtr<const lanenetwork::RouteMap> route_map, 
    const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max)
  : _world(std::move(world)),
  _route_map(std::move(route_map)),
  _bounds_min(bounds_min),
  _bounds_max(bounds_max) {
}

void CrowdController::OnWorldTick(const client::WorldSnapshot& world_snapshot) {
  std::cout << world_snapshot.GetTimestamp().platform_timestamp << std::endl;
}

void CrowdController::Start() {
  _world->OnTick(std::bind(&CrowdController::OnWorldTick, this, std::placeholders::_1));
}

void CrowdController::Stop() {

}

}
}
