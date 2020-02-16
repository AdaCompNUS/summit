// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "carla/client/Actor.h"
#include "carla/client/Vehicle.h"
#include "carla/geom/Location.h"
#include "carla/geom/Math.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Vector3D.h"
#include "carla/Memory.h"
#include "carla/rpc/ActorId.h"

#include "carla/trafficmanager/AtomicActorSet.h"
#include "carla/trafficmanager/InMemoryMap.h"
#include "carla/trafficmanager/LocalizationUtils.h"
#include "carla/trafficmanager/MessengerAndDataTypes.h"
#include "carla/trafficmanager/Parameters.h"
#include "carla/trafficmanager/PipelineStage.h"
#include "carla/trafficmanager/SimpleWaypoint.h"
#include "carla/trafficmanager/PerformanceDiagnostics.h"

namespace carla {
namespace traffic_manager {

  using namespace std::chrono;
  namespace cc = carla::client;
  using Actor = carla::SharedPtr<cc::Actor>;
  using Vehicle = carla::SharedPtr<cc::Vehicle>;
  using ActorId = carla::ActorId;
  using ActorIdSet = std::unordered_set<ActorId>;
  using TLS = carla::rpc::TrafficLightState;

  /// This class is responsible for maintaining a horizon of waypoints ahead
  /// of the vehicle for it to follow.
  /// The class is also responsible for managing lane change decisions and
  /// modify the waypoint trajectory appropriately.
  class LocalizationStage : public PipelineStage {

  private:

    /// Section keys to switch between the output data frames.
    bool planner_frame_selector;
    bool collision_frame_selector;
    bool traffic_light_frame_selector;
    /// Output data frames to be shared with the motion planner stage.
    std::shared_ptr<LocalizationToPlannerFrame> planner_frame_a;
    std::shared_ptr<LocalizationToPlannerFrame> planner_frame_b;
    /// Output data frames to be shared with the collision stage.
    std::shared_ptr<LocalizationToCollisionFrame> collision_frame_a;
    std::shared_ptr<LocalizationToCollisionFrame> collision_frame_b;
    /// Output data frames to be shared with the traffic light stage
    std::shared_ptr<LocalizationToTrafficLightFrame> traffic_light_frame_a;
    std::shared_ptr<LocalizationToTrafficLightFrame> traffic_light_frame_b;
    /// Pointer to messenger to motion planner stage.
    std::shared_ptr<LocalizationToPlannerMessenger> planner_messenger;
    /// Pointer to messenger to collision stage.
    std::shared_ptr<LocalizationToCollisionMessenger> collision_messenger;
    /// Pointer to messenger to traffic light stage.
    std::shared_ptr<LocalizationToTrafficLightMessenger> traffic_light_messenger;
    /// Reference to set of all actors registered with the traffic manager.
    AtomicActorSet &registered_actors;
    /// List of actors registered with the traffic manager in
    /// current update cycle.
    std::vector<Actor> actor_list;
    /// State counter to track changes in registered actors.
    int registered_actors_state;
    /// Reference to local map-cache object.
    InMemoryMap &local_map;
    /// Runtime parameterization object.
    Parameters &parameters;
    /// Reference to Carla's debug helper object.
    cc::DebugHelper &debug_helper;
    /// Carla world object;
    cc::World& world;
    /// Structures to hold waypoint buffers for all vehicles.
    /// These are shared with the collisions stage.
    std::shared_ptr<BufferList> buffer_list;
    /// Map connecting actor ids to indices of data arrays.
    std::unordered_map<ActorId, uint64_t> vehicle_id_to_index;
    /// Number of vehicles currently registered with the traffic manager.
    uint64_t number_of_vehicles;
    /// Used to only calculate the extended buffer once at junctions
    std::map<carla::ActorId, bool> approached;
    /// Point used to know if the junction has free space after its end, mapped to their respective actor id
    std::map<carla::ActorId, SimpleWaypointPtr> final_safe_points;
    /// Object for tracking paths of the traffic vehicles.
    TrackTraffic track_traffic;
    /// Map of all vehicles' idle time.
    std::unordered_map<ActorId, double> idle_time;
    /// Structure to hold the actor with the maximum idle time at each iteration.
    std::pair<Actor, double> maximum_idle_time;
    /// Variable to hold current timestamp from the world snapshot.
    cc::Timestamp current_timestamp;
    /// Simulated seconds since the beginning of the current episode when the last actor was destroyed.
    double elapsed_last_actor_destruction = 0.0;
    /// Counter to track unregistered actors' scan interval.
    uint64_t unregistered_scan_duration = 0;
    /// A structure used to keep track of actors spawned outside of traffic
    /// manager.
    std::unordered_map<ActorId, Actor> unregistered_actors;
    /// Code snippet execution time profiler.
    SnippetProfiler snippet_profiler;

    /// A simple method used to draw waypoint buffer ahead of a vehicle.
    void DrawBuffer(Buffer &buffer);

    /// Method to determine lane change and obtain target lane waypoint.
    SimpleWaypointPtr AssignLaneChange(Actor vehicle, bool force, bool direction);

    // When near an intersection, extends the buffer throughout all the
    // intersection to see if there is space after it
    SimpleWaypointPtr GetSafeLocationAfterJunction(const Vehicle &vehicle, Buffer &waypoint_buffer);

    /// Methods to modify waypoint buffer and track traffic.
    void PushWaypoint(Buffer& buffer, ActorId actor_id, SimpleWaypointPtr& waypoint);
    void PopWaypoint(Buffer& buffer, ActorId actor_id);
    /// Method to scan for unregistered actors and update their grid positioning.
    void ScanUnregisteredVehicles();

    /// Methods for idle vehicle elimination.
    void UpdateIdleTime(const Actor& actor);
    bool IsVehicleStuck(const Actor& actor);
    void CleanActor(const ActorId actor_id);
    bool TryDestroyVehicle(const Actor& actor);

  public:

    LocalizationStage(
        std::string stage_name,
        std::shared_ptr<LocalizationToPlannerMessenger> planner_messenger,
        std::shared_ptr<LocalizationToCollisionMessenger> collision_messenger,
        std::shared_ptr<LocalizationToTrafficLightMessenger> traffic_light_messenger,
        AtomicActorSet &registered_actors,
        InMemoryMap &local_map,
        Parameters &parameters,
        cc::DebugHelper &debug_helper,
        cc::World& world);

    ~LocalizationStage();

    void DataReceiver() override;

    void Action() override;

    void DataSender() override;

  };

} // namespace traffic_manager
} // namespace carla
