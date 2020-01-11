#pragma once

#include <vector>
#include "carla/sidewalk/Sidewalk.h"
#include "carla/sumonetwork/SumoNetwork.h"
#include "carla/microsim/Agent.h"
#include "carla/microsim/VehicleAgent.h"
#include "carla/microsim/PedestrianAgent.h"

namespace carla {
namespace microsim {

class Simulator {

public:

  const sumonetwork::SumoNetwork* sumo_network;
  const sidewalk::Sidewalk* sidewalk;
  VehicleAgent ego_agent;
  std::vector<Agent> exo_agents;

  Simulator(const sumonetwork::SumoNetwork* sumo_network, const sidewalk::Sidewalk* sidewalk, 
      const VehicleAgent& ego_agent, const std::vector<Agent>& exo_agents = {})
    : sumo_network(sumo_network), sidewalk(sidewalk),
    ego_agent(ego_agent), exo_agents(exo_agents) { }

  void Step(float delta, float ego_control_speed, float ego_control_steer);
  void RefreshExoAgents(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, int min_path_points,
      int max_num_pedestrian, int max_num_bike, int max_num_car,
      float pedestrian_clearance, float bike_clearance, float car_clearance, float ego_clearance);

};

}
}
