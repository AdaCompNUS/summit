// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/CarlaActorFactory.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameInstance.h"
#include "Carla/Recorder/CarlaRecorder.h"
#include "Carla/Game/TaggerDelegate.h"
#include "Carla/Settings/CarlaSettingsDelegate.h"
#include "Carla/Weather/Weather.h"
#include "Carla/Crowd/CrowdController.h"
#include "LaneNetwork/LaneNetworkActor.h"

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"

#include <carla/road/Map.h>

#include "CarlaGameModeBase.generated.h"

/// Base class for the CARLA Game Mode.
UCLASS(HideCategories=(ActorTick))
class CARLA_API ACarlaGameModeBase : public AGameModeBase
{
  GENERATED_BODY()

public:

  ACarlaGameModeBase(const FObjectInitializer& ObjectInitializer);

  const UCarlaEpisode &GetCarlaEpisode() const
  {
    check(Episode != nullptr);
    return *Episode;
  }
  
  UFUNCTION(Exec)
  void RenderRoadMap(const FString& FileName) const;

  UFUNCTION(Exec)
  void LoadLaneNetwork(const FString& LaneNetworkPath);

  UFUNCTION(Exec)
  void SpawnWalkers(int Num);

protected:

  void InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage) override;

  void RestartPlayer(AController *NewPlayer) override;

  void BeginPlay() override;

  void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

  void Tick(float DeltaSeconds) override;

private:

  FRoadMap RoadMap;

  boost::optional<carla::road::Map> WaypointMap;

  void SpawnActorFactories();

  void CreateRoadMap();

  void CreateWaypointMap(const FString& MapName);

  UPROPERTY()
  UCarlaGameInstance *GameInstance = nullptr;

  UPROPERTY()
  UTaggerDelegate *TaggerDelegate = nullptr;

  UPROPERTY()
  UCarlaSettingsDelegate *CarlaSettingsDelegate = nullptr;

  UPROPERTY()
  UCarlaEpisode *Episode = nullptr;

  UPROPERTY()
  ACarlaRecorder *Recorder = nullptr;

  UPROPERTY()
  ALaneNetworkActor *LaneNetworkActor = nullptr;

  UPROPERTY()
  ACrowdController *CrowdController = nullptr;

  /// The class of Weather to spawn.
  UPROPERTY(Category = "CARLA Game Mode", EditAnywhere)
  TSubclassOf<AWeather> WeatherClass;

  /// List of actor spawners that will be used to define and spawn the actors
  /// available in game.
  UPROPERTY(Category = "CARLA Game Mode", EditAnywhere)
  TSet<TSubclassOf<ACarlaActorFactory>> ActorFactories;

  UPROPERTY()
  TArray<ACarlaActorFactory *> ActorFactoryInstances;
};
