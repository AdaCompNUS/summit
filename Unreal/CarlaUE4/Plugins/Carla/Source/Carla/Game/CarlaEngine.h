// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/WorldObserver.h"
#include "Carla/Server/CarlaServer.h"
#include "Carla/Util/NonCopyable.h"

#include "Misc/CoreDelegates.h"

class UCarlaSettings;
struct FEpisodeSettings;

class FCarlaEngine : private NonCopyable
{
public:

  ~FCarlaEngine();

  void NotifyInitGame(const UCarlaSettings &Settings);

  void NotifyBeginEpisode(UCarlaEpisode &Episode);

  void NotifyEndEpisode();

  const FCarlaServer &GetServer() const
  {
    return Server;
  }

  UCarlaEpisode *GetCurrentEpisode()
  {
    return CurrentEpisode;
  }

private:

  void OnPreTick(UWorld *world, ELevelTick TickType, float DeltaSeconds);

  void OnPostTick(UWorld *World, ELevelTick TickType, float DeltaSeconds);

  void OnEpisodeSettingsChanged(const FEpisodeSettings &Settings);

  bool bIsRunning = false;

  bool bSynchronousMode = false;

  FCarlaServer Server;

  FWorldObserver WorldObserver;

  UCarlaEpisode *CurrentEpisode = nullptr;

  FDelegateHandle OnPreTickHandle;

  FDelegateHandle OnPostTickHandle;

  FDelegateHandle OnEpisodeSettingsChangeHandle;
};
