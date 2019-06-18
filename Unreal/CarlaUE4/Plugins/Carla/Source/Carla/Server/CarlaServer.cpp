// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Server/CarlaServer.h"

#include "Carla/OpenDrive/OpenDrive.h"
#include "Carla/Util/DebugShapeDrawer.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Walker/WalkerController.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/Functional.h>
#include <carla/Version.h>
#include <carla/rpc/Actor.h>
#include <carla/rpc/ActorDefinition.h>
#include <carla/rpc/ActorDescription.h>
#include <carla/rpc/Command.h>
#include <carla/rpc/CommandResponse.h>
#include <carla/rpc/DebugShape.h>
#include <carla/rpc/EpisodeInfo.h>
#include <carla/rpc/EpisodeSettings.h>
#include <carla/rpc/MapInfo.h>
#include <carla/rpc/Response.h>
#include <carla/rpc/Server.h>
#include <carla/rpc/String.h>
#include <carla/rpc/Transform.h>
#include <carla/rpc/Vector2D.h>
#include <carla/rpc/Vector3D.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/rpc/VehiclePhysicsControl.h>
#include <carla/rpc/WalkerControl.h>
#include <carla/rpc/WeatherParameters.h>
#include <carla/streaming/Server.h>
#include <compiler/enable-ue4-macros.h>

#include <vector>

template <typename T>
using R = carla::rpc::Response<T>;

// =============================================================================
// -- Static local functions ---------------------------------------------------
// =============================================================================

template <typename T, typename Other>
static std::vector<T> MakeVectorFromTArray(const TArray<Other> &Array)
{
  return {Array.GetData(), Array.GetData() + Array.Num()};
}

// =============================================================================
// -- FCarlaServer::FPimpl -----------------------------------------------
// =============================================================================

class FCarlaServer::FPimpl
{
public:

  FPimpl(uint16_t RPCPort, uint16_t StreamingPort)
    : Server(RPCPort),
      StreamingServer(StreamingPort),
      BroadcastStream(StreamingServer.MakeMultiStream())
  {
    BindActions();
  }

  carla::rpc::Server Server;

  carla::streaming::Server StreamingServer;

  carla::streaming::MultiStream BroadcastStream;

  UCarlaEpisode *Episode = nullptr;

  size_t TickCuesReceived = 0u;

private:

  void BindActions();

};

// =============================================================================
// -- Define helper macros -----------------------------------------------------
// =============================================================================

#if WITH_EDITOR
#  define CARLA_ENSURE_GAME_THREAD() check(IsInGameThread());
#else
#  define CARLA_ENSURE_GAME_THREAD()
#endif // WITH_EDITOR

#define RESPOND_ERROR(str) {                                              \
    UE_LOG(LogCarlaServer, Log, TEXT("Responding error: %s"), TEXT(str)); \
    return carla::rpc::ResponseError(str); }

#define RESPOND_ERROR_FSTRING(fstr) {                                 \
    UE_LOG(LogCarlaServer, Log, TEXT("Responding error: %s"), *fstr); \
    return carla::rpc::ResponseError(carla::rpc::FromFString(fstr)); }

#define REQUIRE_CARLA_EPISODE() \
  CARLA_ENSURE_GAME_THREAD();   \
  if (Episode == nullptr) { RESPOND_ERROR("episode not ready"); }

class ServerBinder {
public:

  constexpr ServerBinder(const char *name, carla::rpc::Server &srv, bool sync)
    : _name(name),
      _server(srv),
      _sync(sync) {}

  template <typename FuncT>
  auto operator<<(FuncT func) {
    if (_sync) {
      _server.BindSync(_name, func);
    } else {
      _server.BindAsync(_name, func);
    }
    return func;
  }

private:

  const char *_name;

  carla::rpc::Server &_server;

  bool _sync;
};

#define BIND_SYNC(name) auto name = ServerBinder(#name, Server, true)
#define BIND_ASYNC(name) auto name = ServerBinder(#name, Server, false)

// =============================================================================
// -- Bind Actions -------------------------------------------------------------
// =============================================================================

void FCarlaServer::FPimpl::BindActions()
{
  namespace cr = carla::rpc;
  namespace cg = carla::geom;

  BIND_ASYNC(version) << []() -> R<std::string>
  {
    return carla::version();
  };

  // ~~ Tick ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_SYNC(tick_cue) << [this]() -> R<void>
  {
    ++TickCuesReceived;
    return R<void>::Success();
  };

  // ~~ Load new episode ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_ASYNC(get_available_maps) << [this]() -> R<std::vector<std::string>>
  {
    const auto MapNames = UCarlaStatics::GetAllMapNames();
    std::vector<std::string> result;
    result.reserve(MapNames.Num());
    for (const auto &MapName : MapNames)
    {
      result.emplace_back(cr::FromFString(MapName));
    }
    return result;
  };

  BIND_SYNC(load_new_episode) << [this](const std::string &map_name) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    if (!Episode->LoadNewEpisode(cr::ToFString(map_name)))
    {
      RESPOND_ERROR("map not found");
    }
    return R<void>::Success();
  };

  // ~~ Episode settings and info ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_SYNC(get_episode_info) << [this]() -> R<cr::EpisodeInfo>
  {
    REQUIRE_CARLA_EPISODE();
    return cr::EpisodeInfo{
      Episode->GetId(),
      BroadcastStream.token()};
  };

  BIND_SYNC(get_map_info) << [this]() -> R<cr::MapInfo>
  {
    REQUIRE_CARLA_EPISODE();
    auto FileContents = UOpenDrive::LoadXODR(Episode->GetMapName());
    const auto &SpawnPoints = Episode->GetRecommendedSpawnPoints();
    return cr::MapInfo{
      cr::FromFString(Episode->GetMapName()),
      cr::FromFString(FileContents),
      MakeVectorFromTArray<cg::Transform>(SpawnPoints)};
  };

  BIND_SYNC(get_episode_settings) << [this]() -> R<cr::EpisodeSettings>
  {
    REQUIRE_CARLA_EPISODE();
    return cr::EpisodeSettings{Episode->GetSettings()};
  };

  BIND_SYNC(set_episode_settings) << [this](const cr::EpisodeSettings &settings) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    Episode->ApplySettings(settings);
    return R<void>::Success();
  };

  BIND_SYNC(get_actor_definitions) << [this]() -> R<std::vector<cr::ActorDefinition>>
  {
    REQUIRE_CARLA_EPISODE();
    return MakeVectorFromTArray<cr::ActorDefinition>(Episode->GetActorDefinitions());
  };

  BIND_SYNC(get_spectator) << [this]() -> R<cr::Actor>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(Episode->GetSpectatorPawn());
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("internal error: unable to find spectator");
    }
    return Episode->SerializeActor(ActorView);
  };

  // ~~ Weather ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_SYNC(get_weather_parameters) << [this]() -> R<cr::WeatherParameters>
  {
    REQUIRE_CARLA_EPISODE();
    auto *Weather = Episode->GetWeather();
    if (Weather == nullptr)
    {
      RESPOND_ERROR("internal error: unable to find weather");
    }
    return Weather->GetCurrentWeather();
  };

  BIND_SYNC(set_weather_parameters) << [this](
      const cr::WeatherParameters &weather) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto *Weather = Episode->GetWeather();
    if (Weather == nullptr)
    {
      RESPOND_ERROR("internal error: unable to find weather");
    }
    Weather->ApplyWeather(weather);
    return R<void>::Success();
  };

  // ~~ Actor operations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_SYNC(get_actors_by_id) << [this](
      const std::vector<FActorView::IdType> &ids) -> R<std::vector<cr::Actor>>
  {
    REQUIRE_CARLA_EPISODE();
    std::vector<cr::Actor> Result;
    Result.reserve(ids.size());
    for (auto &&Id : ids)
    {
      auto View = Episode->FindActor(Id);
      if (View.IsValid())
      {
        Result.emplace_back(Episode->SerializeActor(View));
      }
    }
    return Result;
  };

  BIND_SYNC(spawn_actor) << [this](
      cr::ActorDescription Description,
      const cr::Transform &Transform) -> R<cr::Actor>
  {
    REQUIRE_CARLA_EPISODE();
    auto Result = Episode->SpawnActorWithInfo(Transform, std::move(Description));
    if (Result.Key != EActorSpawnResultStatus::Success)
    {
      RESPOND_ERROR_FSTRING(FActorSpawnResult::StatusToString(Result.Key));
    }
    if (!Result.Value.IsValid())
    {
      RESPOND_ERROR("internal error: actor could not be spawned");
    }
    return Episode->SerializeActor(Result.Value);
  };

  BIND_SYNC(spawn_actor_with_parent) << [this](
      cr::ActorDescription Description,
      const cr::Transform &Transform,
      cr::ActorId ParentId,
      cr::AttachmentType InAttachmentType) -> R<cr::Actor>
  {
    REQUIRE_CARLA_EPISODE();
    auto Result = Episode->SpawnActorWithInfo(Transform, std::move(Description));
    if (Result.Key != EActorSpawnResultStatus::Success)
    {
      RESPOND_ERROR_FSTRING(FActorSpawnResult::StatusToString(Result.Key));
    }
    if (!Result.Value.IsValid())
    {
      RESPOND_ERROR("internal error: actor could not be spawned");
    }
    auto ParentActorView = Episode->FindActor(ParentId);
    if (!ParentActorView.IsValid())
    {
      RESPOND_ERROR("unable to attach actor: parent actor not found");
    }
    Episode->AttachActors(
        Result.Value.GetActor(),
        ParentActorView.GetActor(),
        static_cast<EAttachmentType>(InAttachmentType));
    return Episode->SerializeActor(Result.Value);
  };

  BIND_SYNC(destroy_actor) << [this](cr::ActorId ActorId) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to destroy actor: not found");
    }
    if (!Episode->DestroyActor(ActorView.GetActor()))
    {
      RESPOND_ERROR("internal error: unable to destroy actor");
    }
    return R<void>::Success();
  };

  // ~~ Actor physics ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_SYNC(set_actor_location) << [this](
      cr::ActorId ActorId,
      cr::Location Location) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to set actor location: actor not found");
    }
    ActorView.GetActor()->SetActorRelativeLocation(
    Location,
    false,
    nullptr,
    ETeleportType::TeleportPhysics);
    return R<void>::Success();
  };

  BIND_SYNC(set_actor_transform) << [this](
      cr::ActorId ActorId,
      cr::Transform Transform) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to set actor transform: actor not found");
    }
    ActorView.GetActor()->SetActorRelativeTransform(
    Transform,
    false,
    nullptr,
    ETeleportType::TeleportPhysics);
    return R<void>::Success();
  };

  BIND_SYNC(set_actor_velocity) << [this](
      cr::ActorId ActorId,
      cr::Vector3D vector) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to set actor velocity: actor not found");
    }
    auto RootComponent = Cast<UPrimitiveComponent>(ActorView.GetActor()->GetRootComponent());
    if (RootComponent == nullptr)
    {
      RESPOND_ERROR("unable to set actor velocity: not supported by actor");
    }
    RootComponent->SetPhysicsLinearVelocity(
        vector.ToCentimeters().ToFVector(),
        false,
        "None");
    return R<void>::Success();
  };

  BIND_SYNC(set_actor_angular_velocity) << [this](
      cr::ActorId ActorId,
      cr::Vector3D vector) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to set actor angular velocity: actor not found");
    }
    auto RootComponent = Cast<UPrimitiveComponent>(ActorView.GetActor()->GetRootComponent());
    if (RootComponent == nullptr)
    {
      RESPOND_ERROR("unable to set actor angular velocity: not supported by actor");
    }
    RootComponent->SetPhysicsAngularVelocityInDegrees(
        vector.ToFVector(),
        false,
        "None");
    return R<void>::Success();
  };

  BIND_SYNC(add_actor_impulse) << [this](
      cr::ActorId ActorId,
      cr::Vector3D vector) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to add actor impulse: actor not found");
    }
    auto RootComponent = Cast<UPrimitiveComponent>(ActorView.GetActor()->GetRootComponent());
    if (RootComponent == nullptr)
    {
      RESPOND_ERROR("unable to add actor impulse: not supported by actor");
    }
    RootComponent->AddImpulse(
        vector.ToCentimeters().ToFVector(),
        "None",
        false);
    return R<void>::Success();
  };

  BIND_SYNC(get_physics_control) << [this](
      cr::ActorId ActorId) -> R<cr::VehiclePhysicsControl>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to get actor physics control: actor not found");
    }
    auto Vehicle = Cast<ACarlaWheeledVehicle>(ActorView.GetActor());
    if (Vehicle == nullptr)
    {
      RESPOND_ERROR("unable to get actor physics control: actor is not a vehicle");
    }

    return cr::VehiclePhysicsControl(Vehicle->GetVehiclePhysicsControl());
  };

  BIND_SYNC(apply_physics_control) << [this](
      cr::ActorId ActorId,
      cr::VehiclePhysicsControl PhysicsControl) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to apply actor physics control: actor not found");
    }
    auto Vehicle = Cast<ACarlaWheeledVehicle>(ActorView.GetActor());
    if (Vehicle == nullptr)
    {
      RESPOND_ERROR("unable to apply actor physics control: actor is not a vehicle");
    }

    Vehicle->ApplyVehiclePhysicsControl(FVehiclePhysicsControl(PhysicsControl));

    return R<void>::Success();
  };

  BIND_SYNC(set_actor_simulate_physics) << [this](
      cr::ActorId ActorId,
      bool bEnabled) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to set actor simulate physics: actor not found");
    }
    auto RootComponent = Cast<UPrimitiveComponent>(ActorView.GetActor()->GetRootComponent());
    if (RootComponent == nullptr)
    {
      RESPOND_ERROR("unable to set actor simulate physics: not supported by actor");
    }
    RootComponent->SetSimulatePhysics(bEnabled);
    return R<void>::Success();
  };

  // ~~ Apply control ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_SYNC(apply_control_to_vehicle) << [this](
      cr::ActorId ActorId,
      cr::VehicleControl Control) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to apply control: actor not found");
    }
    auto Vehicle = Cast<ACarlaWheeledVehicle>(ActorView.GetActor());
    if (Vehicle == nullptr)
    {
      RESPOND_ERROR("unable to apply control: actor is not a vehicle");
    }
    Vehicle->ApplyVehicleControl(Control, EVehicleInputPriority::Client);
    return R<void>::Success();
  };

  BIND_SYNC(apply_control_to_walker) << [this](
      cr::ActorId ActorId,
      cr::WalkerControl Control) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to apply control: actor not found");
    }
    auto Pawn = Cast<APawn>(ActorView.GetActor());
    if (Pawn == nullptr)
    {
      RESPOND_ERROR("unable to apply control: actor is not a walker");
    }
    auto Controller = Cast<AWalkerController>(Pawn->GetController());
    if (Controller == nullptr)
    {
      RESPOND_ERROR("unable to apply control: walker has an incompatible controller");
    }
    Controller->ApplyWalkerControl(Control);
    return R<void>::Success();
  };

  BIND_SYNC(set_actor_autopilot) << [this](
      cr::ActorId ActorId,
      bool bEnabled) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->FindActor(ActorId);
    if (!ActorView.IsValid())
    {
      RESPOND_ERROR("unable to set autopilot: actor not found");
    }
    auto Vehicle = Cast<ACarlaWheeledVehicle>(ActorView.GetActor());
    if (Vehicle == nullptr)
    {
      RESPOND_ERROR("unable to set autopilot: actor does not support autopilot");
    }
    auto Controller = Cast<AWheeledVehicleAIController>(Vehicle->GetController());
    if (Controller == nullptr)
    {
      RESPOND_ERROR("unable to set autopilot: vehicle controller does not support autopilot");
    }
    Controller->SetAutopilot(bEnabled);
    return R<void>::Success();
  };

  // ~~ Traffic lights ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_SYNC(set_traffic_light_state) << [this](
      cr::ActorId ActorId,
      cr::TrafficLightState trafficLightState) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->GetActorRegistry().Find(ActorId);
    if (!ActorView.IsValid() || ActorView.GetActor()->IsPendingKill())
    {
      RESPOND_ERROR("unable to set state: actor not found");
    }
    auto TrafficLight = Cast<ATrafficLightBase>(ActorView.GetActor());
    if (TrafficLight == nullptr)
    {
      RESPOND_ERROR("unable to set state: actor is not a traffic light");
    }
    TrafficLight->SetTrafficLightState(static_cast<ETrafficLightState>(trafficLightState));
    return R<void>::Success();
  };

  BIND_SYNC(set_traffic_light_green_time) << [this](
      cr::ActorId ActorId,
      float GreenTime) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->GetActorRegistry().Find(ActorId);
    if (!ActorView.IsValid() || ActorView.GetActor()->IsPendingKill())
    {
      RESPOND_ERROR("unable to set green time: actor not found");
    }
    auto TrafficLight = Cast<ATrafficLightBase>(ActorView.GetActor());
    if (TrafficLight == nullptr)
    {
      RESPOND_ERROR("unable to set green time: actor is not a traffic light");
    }
    TrafficLight->SetGreenTime(GreenTime);
    return R<void>::Success();
  };

  BIND_SYNC(set_traffic_light_yellow_time) << [this](
      cr::ActorId ActorId,
      float YellowTime) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->GetActorRegistry().Find(ActorId);
    if (!ActorView.IsValid() || ActorView.GetActor()->IsPendingKill())
    {
      RESPOND_ERROR("unable to set yellow time: actor not found");
    }
    auto TrafficLight = Cast<ATrafficLightBase>(ActorView.GetActor());
    if (TrafficLight == nullptr)
    {
      RESPOND_ERROR("unable to set yellow time: actor is not a traffic light");
    }
    TrafficLight->SetYellowTime(YellowTime);
    return R<void>::Success();
  };

  BIND_SYNC(set_traffic_light_red_time) << [this](
      cr::ActorId ActorId,
      float RedTime) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->GetActorRegistry().Find(ActorId);
    if (!ActorView.IsValid() || ActorView.GetActor()->IsPendingKill())
    {
      RESPOND_ERROR("unable to set red time: actor not found");
    }
    auto TrafficLight = Cast<ATrafficLightBase>(ActorView.GetActor());
    if (TrafficLight == nullptr)
    {
      RESPOND_ERROR("unable to set red time: actor is not a traffic light");
    }
    TrafficLight->SetRedTime(RedTime);
    return R<void>::Success();
  };

  BIND_SYNC(freeze_traffic_light) << [this](
      cr::ActorId ActorId,
      bool Freeze) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->GetActorRegistry().Find(ActorId);
    if (!ActorView.IsValid() || ActorView.GetActor()->IsPendingKill())
    {
      RESPOND_ERROR("unable to alter frozen state: actor not found");
    }
    auto TrafficLight = Cast<ATrafficLightBase>(ActorView.GetActor());
    if (TrafficLight == nullptr)
    {
      RESPOND_ERROR("unable to alter frozen state: actor is not a traffic light");
    }
    TrafficLight->SetTimeIsFrozen(Freeze);
    return R<void>::Success();
  };

  BIND_SYNC(get_group_traffic_lights) << [this](
      const cr::ActorId ActorId) -> R<std::vector<cr::ActorId>>
  {
    REQUIRE_CARLA_EPISODE();
    auto ActorView = Episode->GetActorRegistry().Find(ActorId);
    if (!ActorView.IsValid() || ActorView.GetActor()->IsPendingKill())
    {
      RESPOND_ERROR("unable to get group traffic lights: actor not found");
    }
    auto TrafficLight = Cast<ATrafficLightBase>(ActorView.GetActor());
    if (TrafficLight == nullptr)
    {
      RESPOND_ERROR("unable to get group traffic lights: actor is not a traffic light");
    }
    std::vector<cr::ActorId> Result;
    for (auto TLight : TrafficLight->GetGroupTrafficLights())
    {
      auto View = Episode->FindActor(TLight);
      if (View.IsValid())
      {
        Result.push_back(View.GetActorId());
      }
    }
    return Result;
  };

  // ~~ Logging and playback ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_SYNC(start_recorder) << [this](std::string name) -> R<std::string>
  {
    REQUIRE_CARLA_EPISODE();
    return R<std::string>(Episode->StartRecorder(name));
  };

  BIND_SYNC(stop_recorder) << [this]() -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    Episode->GetRecorder()->Stop();
    return R<void>::Success();
  };

  BIND_SYNC(show_recorder_file_info) << [this](std::string name, bool show_all) -> R<std::string>
  {
    REQUIRE_CARLA_EPISODE();
    return R<std::string>(Episode->GetRecorder()->ShowFileInfo(
        name,
        show_all));
  };

  BIND_SYNC(show_recorder_collisions) << [this](std::string name, char type1, char type2) -> R<std::string>
  {
    REQUIRE_CARLA_EPISODE();
    return R<std::string>(Episode->GetRecorder()->ShowFileCollisions(
        name,
        type1,
        type2));
  };

  BIND_SYNC(show_recorder_actors_blocked) << [this](std::string name, double min_time, double min_distance) -> R<std::string>
  {
    REQUIRE_CARLA_EPISODE();
    return R<std::string>(Episode->GetRecorder()->ShowFileActorsBlocked(
        name,
        min_time,
        min_distance));
  };

  BIND_SYNC(replay_file) << [this](std::string name, double start, double duration, uint32_t follow_id) -> R<std::string>
  {
    REQUIRE_CARLA_EPISODE();
    return R<std::string>(Episode->GetRecorder()->ReplayFile(
        name,
        start,
        duration,
        follow_id));
  };

  BIND_SYNC(set_replayer_time_factor) << [this](double time_factor) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    Episode->GetRecorder()->SetReplayerTimeFactor(time_factor);
    return R<void>::Success();
  };

  // ~~ Draw debug shapes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  BIND_SYNC(draw_debug_shape) << [this](const cr::DebugShape &shape) -> R<void>
  {
    REQUIRE_CARLA_EPISODE();
    auto *World = Episode->GetWorld();
    check(World != nullptr);
    FDebugShapeDrawer Drawer(*World);
    Drawer.Draw(shape);
    return R<void>::Success();
  };

  // ~~ Apply commands in batch ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  using C = cr::Command;
  using CR = cr::CommandResponse;
  using ActorId = carla::ActorId;

  auto parse_result = [](ActorId id, const auto &response) {
    return response.HasError() ? CR{response.GetError()} : CR{id};
  };

#define MAKE_RESULT(operation) return parse_result(c.actor, operation);

  auto command_visitor = carla::Functional::MakeRecursiveOverload(
      [=](auto self, const C::SpawnActor &c) -> CR {
        auto result = c.parent.has_value() ?
            spawn_actor_with_parent(
                c.description,
                c.transform,
                *c.parent,
                cr::AttachmentType::Rigid) :
            spawn_actor(c.description, c.transform);
        if (!result.HasError()) {
          ActorId id = result.Get().id;
          auto set_id = carla::Functional::MakeOverload(
              [](C::SpawnActor &) {},
              [id](auto &s) { s.actor = id; });
          for (auto command : c.do_after) {
            boost::apply_visitor(set_id, command.command);
            boost::apply_visitor(self, command.command);
          }
          return id;
        }
        return result.GetError();
      },
      [=](auto, const C::DestroyActor &c) {         MAKE_RESULT(destroy_actor(c.actor)); },
      [=](auto, const C::ApplyVehicleControl &c) {  MAKE_RESULT(apply_control_to_vehicle(c.actor, c.control)); },
      [=](auto, const C::ApplyWalkerControl &c) {   MAKE_RESULT(apply_control_to_walker(c.actor, c.control)); },
      [=](auto, const C::ApplyTransform &c) {       MAKE_RESULT(set_actor_transform(c.actor, c.transform)); },
      [=](auto, const C::ApplyVelocity &c) {        MAKE_RESULT(set_actor_velocity(c.actor, c.velocity)); },
      [=](auto, const C::ApplyAngularVelocity &c) { MAKE_RESULT(set_actor_angular_velocity(c.actor, c.angular_velocity)); },
      [=](auto, const C::ApplyImpulse &c) {         MAKE_RESULT(add_actor_impulse(c.actor, c.impulse)); },
      [=](auto, const C::SetSimulatePhysics &c) {   MAKE_RESULT(set_actor_simulate_physics(c.actor, c.enabled)); },
      [=](auto, const C::SetAutopilot &c) {         MAKE_RESULT(set_actor_autopilot(c.actor, c.enabled)); });

#undef MAKE_RESULT

  BIND_SYNC(apply_batch) << [=](const std::vector<cr::Command> &commands, bool do_tick_cue)
  {
    std::vector<CR> result;
    result.reserve(commands.size());
    for (const auto &command : commands)
    {
      result.emplace_back(boost::apply_visitor(command_visitor, command.command));
    }
    if (do_tick_cue)
    {
      tick_cue();
    }
    return result;
  };
}

// =============================================================================
// -- Undef helper macros ------------------------------------------------------
// =============================================================================

#undef BIND_ASYNC
#undef BIND_SYNC
#undef REQUIRE_CARLA_EPISODE
#undef RESPOND_ERROR_FSTRING
#undef RESPOND_ERROR
#undef CARLA_ENSURE_GAME_THREAD

// =============================================================================
// -- FCarlaServer -------------------------------------------------------
// =============================================================================

FCarlaServer::FCarlaServer() : Pimpl(nullptr) {}

FCarlaServer::~FCarlaServer() {}

FDataMultiStream FCarlaServer::Start(uint16_t RPCPort, uint16_t StreamingPort)
{
  Pimpl = MakeUnique<FPimpl>(RPCPort, StreamingPort);
  StreamingPort = Pimpl->StreamingServer.GetLocalEndpoint().port();
  UE_LOG(LogCarlaServer, Log, TEXT("Initialized CarlaServer: Ports(rpc=%d, streaming=%d)"), RPCPort, StreamingPort);
  return Pimpl->BroadcastStream;
}

void FCarlaServer::NotifyBeginEpisode(UCarlaEpisode &Episode)
{
  check(Pimpl != nullptr);
  UE_LOG(LogCarlaServer, Log, TEXT("New episode '%s' started"), *Episode.GetMapName());
  Pimpl->Episode = &Episode;
}

void FCarlaServer::NotifyEndEpisode()
{
  check(Pimpl != nullptr);
  Pimpl->Episode = nullptr;
}

void FCarlaServer::AsyncRun(uint32 NumberOfWorkerThreads)
{
  check(Pimpl != nullptr);
  /// @todo Define better the number of threads each server gets.
  auto RPCThreads = NumberOfWorkerThreads / 2u;
  auto StreamingThreads = NumberOfWorkerThreads - RPCThreads;
  Pimpl->Server.AsyncRun(std::max(2u, RPCThreads));
  Pimpl->StreamingServer.AsyncRun(std::max(2u, StreamingThreads));
}

void FCarlaServer::RunSome(uint32 Milliseconds)
{
  Pimpl->Server.SyncRunFor(carla::time_duration::milliseconds(Milliseconds));
}

bool FCarlaServer::TickCueReceived()
{
  if (Pimpl->TickCuesReceived > 0u)
  {
    --(Pimpl->TickCuesReceived);
    return true;
  }
  return false;
}

void FCarlaServer::Stop()
{
  check(Pimpl != nullptr);
  Pimpl->Server.Stop();
}

FDataStream FCarlaServer::OpenStream() const
{
  check(Pimpl != nullptr);
  return Pimpl->StreamingServer.MakeStream();
}
