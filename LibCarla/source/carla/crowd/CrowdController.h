#pragma once

#include "carla/client/World.h"
#include "carla/Memory.h"
#include "carla/lanenetwork/LaneNetwork.h"


namespace carla {
namespace crowd {

class CrowdController {

public:

  CrowdController(SharedPtr<client::World> world, SharedPtr<lanenetwork::LaneNetwork>);

private:

  SharedPtr<client::World> _world;
  SharedPtr<lanenetwork::LaneNetwork> _lane_network;

  void OnWorldTick(const client::WorldSnapshot& world_snapshot); 

};

}
}
