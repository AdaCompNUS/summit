// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

// #include "GameFramework/Actor.h"
#include <fstream>

#include "Carla/Actor/ActorDescription.h"

#include "CarlaRecorderAnimVehicle.h"
#include "CarlaRecorderAnimWalker.h"
#include "CarlaRecorderCollision.h"
#include "CarlaRecorderEventAdd.h"
#include "CarlaRecorderEventDel.h"
#include "CarlaRecorderEventParent.h"
#include "CarlaRecorderFrames.h"
#include "CarlaRecorderInfo.h"
#include "CarlaRecorderPosition.h"
#include "CarlaRecorderQuery.h"
#include "CarlaRecorderState.h"
#include "CarlaReplayer.h"

#include "CarlaRecorder.generated.h"

class AActor;
class UCarlaEpisode;

enum class CarlaRecorderPacketId : uint8_t
{
  FrameStart = 0,
  FrameEnd,
  EventAdd,
  EventDel,
  EventParent,
  Collision,
  Position,
  State,
  AnimVehicle,
  AnimWalker
};

/// Recorder for the simulation
UCLASS()
class CARLA_API ACarlaRecorder : public AActor
{
  GENERATED_BODY()

public:

  ACarlaRecorder(void);
  ACarlaRecorder(const FObjectInitializer &InObjectInitializer);

  // enable / disable
  bool IsEnabled(void)
  {
    return Enabled;
  }
  void Enable(void);

  void Disable(void);

  // start / stop
  std::string Start(std::string Name, FString MapName);

  void Stop(void);

  void Clear(void);

  void Write(double DeltaSeconds);

  // events
  void AddEvent(const CarlaRecorderEventAdd &Event);

  void AddEvent(const CarlaRecorderEventDel &Event);

  void AddEvent(const CarlaRecorderEventParent &Event);

  void AddCollision(AActor *Actor1, AActor *Actor2);

  void AddPosition(const CarlaRecorderPosition &Position);

  void AddState(const CarlaRecorderStateTrafficLight &State);

  void AddAnimVehicle(const CarlaRecorderAnimVehicle &Vehicle);

  void AddAnimWalker(const CarlaRecorderAnimWalker &Walker);

  // set episode
  void SetEpisode(UCarlaEpisode *ThisEpisode)
  {
    Episode = ThisEpisode;
    Replayer.SetEpisode(ThisEpisode);
  }

  void CreateRecorderEventAdd(
      uint32_t DatabaseId,
      uint8_t Type,
      const FTransform &Transform,
      FActorDescription ActorDescription);

  // replayer
  CarlaReplayer *GetReplayer(void)
  {
    return &Replayer;
  }

  // queries
  std::string ShowFileInfo(std::string Name, bool bShowAll = false);
  std::string ShowFileCollisions(std::string Name, char Type1, char Type2);
  std::string ShowFileActorsBlocked(std::string Name, double MinTime = 30, double MinDistance = 10);

  // replayer
  std::string ReplayFile(std::string Name, double TimeStart, double Duration, uint32_t FollowId);
  void SetReplayerTimeFactor(double TimeFactor);
  void SetReplayerIgnoreHero(bool IgnoreHero);

  void Tick(float DeltaSeconds) final;

private:

  bool Enabled;   // enabled or not

  uint32_t NextCollisionId = 0;

  // files
  std::ofstream File;

  UCarlaEpisode *Episode = nullptr;

  // structures
  CarlaRecorderInfo Info;
  CarlaRecorderFrames Frames;
  CarlaRecorderEventsAdd EventsAdd;
  CarlaRecorderEventsDel EventsDel;
  CarlaRecorderEventsParent EventsParent;
  CarlaRecorderCollisions Collisions;
  CarlaRecorderPositions Positions;
  CarlaRecorderStates States;
  CarlaRecorderAnimVehicles Vehicles;
  CarlaRecorderAnimWalkers Walkers;

  // replayer
  CarlaReplayer Replayer;

  // query tools
  CarlaRecorderQuery Query;

  void AddExistingActors(void);
  void AddActorPosition(FActorView &View);
  void AddWalkerAnimation(FActorView &View);
  void AddVehicleAnimation(FActorView &View);
  void AddTrafficLightState(FActorView &View);
};
