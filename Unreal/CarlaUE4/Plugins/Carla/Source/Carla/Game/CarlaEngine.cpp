// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Game/CarlaEngine.h"

#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStaticDelegates.h"
#include "Carla/Settings/CarlaSettings.h"
#include "Carla/Settings/EpisodeSettings.h"

#include "Runtime/Core/Public/Misc/App.h"

#include <thread>

// =============================================================================
// -- Static local methods -----------------------------------------------------
// =============================================================================

static uint32 FCarlaEngine_GetNumberOfThreadsForRPCServer()
{
  return std::max(std::thread::hardware_concurrency(), 4u) - 2u;
}

static TOptional<double> FCarlaEngine_GetFixedDeltaSeconds()
{
  return FApp::IsBenchmarking() ? FApp::GetFixedDeltaTime() : TOptional<double>{};
}

static void FCarlaEngine_SetFixedDeltaSeconds(TOptional<double> FixedDeltaSeconds)
{
  FApp::SetBenchmarking(FixedDeltaSeconds.IsSet());
  FApp::SetFixedDeltaTime(FixedDeltaSeconds.Get(0.0));
}

// =============================================================================
// -- FCarlaEngine -------------------------------------------------------------
// =============================================================================

FCarlaEngine::~FCarlaEngine()
{
  if (bIsRunning)
  {
    FWorldDelegates::OnWorldTickStart.Remove(OnPreTickHandle);
    FWorldDelegates::OnWorldPostActorTick.Remove(OnPostTickHandle);
    FCarlaStaticDelegates::OnEpisodeSettingsChange.Remove(OnEpisodeSettingsChangeHandle);
  }
}

void FCarlaEngine::NotifyInitGame(const UCarlaSettings &Settings)
{
  if (!bIsRunning)
  {
    const auto StreamingPort = Settings.StreamingPort.Get(Settings.RPCPort + 1u);
    auto BroadcastStream = Server.Start(Settings.RPCPort, StreamingPort);
    Server.AsyncRun(FCarlaEngine_GetNumberOfThreadsForRPCServer());

    WorldObserver.SetStream(BroadcastStream);

    OnPreTickHandle = FWorldDelegates::OnWorldTickStart.AddRaw(
        this,
        &FCarlaEngine::OnPreTick);
    OnPostTickHandle = FWorldDelegates::OnWorldPostActorTick.AddRaw(
        this,
        &FCarlaEngine::OnPostTick);
    OnEpisodeSettingsChangeHandle = FCarlaStaticDelegates::OnEpisodeSettingsChange.AddRaw(
        this,
        &FCarlaEngine::OnEpisodeSettingsChanged);

    bIsRunning = true;
  }
}

void FCarlaEngine::NotifyBeginEpisode(UCarlaEpisode &Episode)
{
  Episode.EpisodeSettings.FixedDeltaSeconds = FCarlaEngine_GetFixedDeltaSeconds();
  CurrentEpisode = &Episode;
  Server.NotifyBeginEpisode(Episode);
}

void FCarlaEngine::NotifyEndEpisode()
{
  Server.NotifyEndEpisode();
  CurrentEpisode = nullptr;
}

void FCarlaEngine::OnPreTick(ELevelTick TickType, float DeltaSeconds)
{
  if ((TickType == ELevelTick::LEVELTICK_All) && (CurrentEpisode != nullptr))
  {
    CurrentEpisode->TickTimers(DeltaSeconds);
    WorldObserver.BroadcastTick(*CurrentEpisode, DeltaSeconds);
  }
}

void FCarlaEngine::OnPostTick(UWorld *, ELevelTick, float)
{
  do
  {
    Server.RunSome(10u);
  }
  while (bSynchronousMode && !Server.TickCueReceived());
}

void FCarlaEngine::OnEpisodeSettingsChanged(const FEpisodeSettings &Settings)
{
  bSynchronousMode = Settings.bSynchronousMode;

  if (GEngine && GEngine->GameViewport)
  {
    GEngine->GameViewport->bDisableWorldRendering = Settings.bNoRenderingMode;
  }

  FCarlaEngine_SetFixedDeltaSeconds(Settings.FixedDeltaSeconds);
}
