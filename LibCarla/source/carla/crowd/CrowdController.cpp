#include "CrowdController.h"

namespace carla {
namespace crowd {

CrowdController::CrowdController(SharedPtr<client::World> world, SharedPtr<lanenetwork::LaneNetwork> lane_network)
  : _world(std::move(world)),
  _lane_network(lane_network) {

  _world->OnTick(std::bind(&CrowdController::OnWorldTick, this, std::placeholders::_1));
}

void CrowdController::OnWorldTick(const client::WorldSnapshot& world_snapshot) {
  
}

}
}
