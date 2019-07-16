#pragma once

#include "carla/client/World.h"
#include "carla/Memory.h"
#include "carla/lanenetwork/LaneNetwork.h"
#include "carla/geom/Vector2D.h"

namespace carla {
namespace crowd {

class CrowdController {

public:

  CrowdController(SharedPtr<client::World> world, SharedPtr<lanenetwork::LaneNetwork> lane_network, 
      const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max);
  
  void Start();

  void Stop();

private:

  SharedPtr<client::World> _world;
  SharedPtr<lanenetwork::LaneNetwork> _lane_network;
  geom::Vector2D _bounds_min;
  geom::Vector2D _bounds_max;

  void OnWorldTick(const client::WorldSnapshot& world_snapshot); 

};

}
}
