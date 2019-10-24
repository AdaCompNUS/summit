// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/client/World.h"

#include "carla/Logging.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/ActorList.h"
#include "carla/client/detail/Simulator.h"

#include <exception>

namespace carla {
namespace client {

  SharedPtr<Map> World::GetMap() const {
    return _episode.Lock()->GetCurrentMap();
  }

  SharedPtr<BlueprintLibrary> World::GetBlueprintLibrary() const {
    return _episode.Lock()->GetBlueprintLibrary();
  }

  boost::optional<geom::Location> World::GetRandomLocationFromNavigation() const {
    return _episode.Lock()->GetRandomLocationFromNavigation();
  }

  SharedPtr<Actor> World::GetSpectator() const {
    return _episode.Lock()->GetSpectator();
  }

  rpc::EpisodeSettings World::GetSettings() const {
    return _episode.Lock()->GetEpisodeSettings();
  }

  uint64_t World::ApplySettings(const rpc::EpisodeSettings &settings) {
    return _episode.Lock()->SetEpisodeSettings(settings);
  }
    
  uint32_t World::SpawnDynamicMesh(const std::vector<geom::Vector3D> &triangles, std::string material) {
    return _episode.Lock()->SpawnDynamicMesh(triangles, material);
  }
  
  uint32_t World::SpawnDynamicTileMesh(const geom::Vector3D& bounds_min, const geom::Vector3D& bounds_max, const std::vector<uint8_t>& data) {
    return _episode.Lock()->SpawnDynamicTileMesh(bounds_min, bounds_max, data);
  }
    
  bool World::DestroyDynamicMesh(uint32_t id) {
    return _episode.Lock()->DestroyDynamicMesh(id);
  }

  rpc::WeatherParameters World::GetWeather() const {
    return _episode.Lock()->GetWeatherParameters();
  }

  void World::SetWeather(const rpc::WeatherParameters &weather) {
    _episode.Lock()->SetWeatherParameters(weather);
  }

  WorldSnapshot World::GetSnapshot() const {
    return _episode.Lock()->GetWorldSnapshot();
  }

  SharedPtr<Actor> World::GetActor(ActorId id) const {
    auto simulator = _episode.Lock();
    auto description = simulator->GetActorById(id);
    return description.has_value() ?
        simulator->MakeActor(std::move(*description)) :
        nullptr;
  }

  SharedPtr<ActorList> World::GetActors() const {
    return SharedPtr<ActorList>{new ActorList{
                                  _episode,
                                  _episode.Lock()->GetAllTheActorsInTheEpisode()}};
  }

  SharedPtr<ActorList> World::GetActors(const std::vector<ActorId> &actor_ids) const {
    return SharedPtr<ActorList>{new ActorList{
                                  _episode,
                                  _episode.Lock()->GetActorsById(actor_ids)}};
  }

  SharedPtr<Actor> World::SpawnActor(
      const ActorBlueprint &blueprint,
      const geom::Transform &transform,
      Actor *parent_actor,
      rpc::AttachmentType attachment_type) {
    return _episode.Lock()->SpawnActor(blueprint, transform, parent_actor, attachment_type);
  }

  SharedPtr<Actor> World::TrySpawnActor(
      const ActorBlueprint &blueprint,
      const geom::Transform &transform,
      Actor *parent_actor,
      rpc::AttachmentType attachment_type) noexcept {
    try {
      return SpawnActor(blueprint, transform, parent_actor, attachment_type);
    } catch (const std::exception &e) {
      return nullptr;
    }
  }

  WorldSnapshot World::WaitForTick(time_duration timeout) const {
    return _episode.Lock()->WaitForTick(timeout);
  }

  size_t World::OnTick(std::function<void(WorldSnapshot)> callback) {
    return _episode.Lock()->RegisterOnTickEvent(std::move(callback));
  }

  void World::RemoveOnTick(size_t callback_id) {
    _episode.Lock()->RemoveOnTickEvent(callback_id);
  }

  uint64_t World::Tick() {
    return _episode.Lock()->Tick();
  }

} // namespace client
} // namespace carla
