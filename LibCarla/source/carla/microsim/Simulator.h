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

  Simulator(const sumonetwork::SumoNetwork* sumo_network, const sidewalk::Sidewalk* sidewalk, 
      const VehicleAgent& ego_agent, const std::vector<Agent>& exo_agents = {})
    : _sumo_network(sumo_network),
    _sidewalk(sidewalk),
    _ego_agent(ego_agent),
    _exo_agents(exo_agents) { }

  const sumonetwork::SumoNetwork* SumoNetwork() const { return _sumo_network; }
  const sidewalk::Sidewalk* Sidewalk() const { return _sidewalk; }
  const VehicleAgent& EgoAgent() const { return _ego_agent; }
  const std::vector<Agent>& ExoAgents() const { return _exo_agents; }

  Simulator Step(float delta, float ego_control_speed, float ego_control_steer) const;
  Simulator RefreshExoAgents(const geom::Vector2D& bounds_min, const geom::Vector2D& bounds_max, int min_path_points,
      int max_num_pedestrian, int max_num_bike, int max_num_car,
      float pedestrian_clearance, float bike_clearance, float car_clearance, float ego_clearance) const;

private:

  const sumonetwork::SumoNetwork* const _sumo_network;
  const sidewalk::Sidewalk* const _sidewalk;
  const VehicleAgent _ego_agent;
  const std::vector<Agent> _exo_agents;

};

}
}
