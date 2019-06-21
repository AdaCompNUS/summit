// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Game/CarlaGameModeBase.h"
#include "Map/RoadMap.h"
#include "Map/RoadTriangle.h"
#include "Carla/OpenDrive/OpenDrive.h"
#include <carla/opendrive/OpenDriveParser.h>

#include <compiler/disable-ue4-macros.h>
#include <carla/rpc/WeatherParameters.h>
#include <compiler/enable-ue4-macros.h>

ACarlaGameModeBase::ACarlaGameModeBase(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PrePhysics;
  bAllowTickBeforeBeginPlay = false;

  Episode = CreateDefaultSubobject<UCarlaEpisode>(TEXT("Episode"));

  Recorder = CreateDefaultSubobject<ACarlaRecorder>(TEXT("Recorder"));

  CrowdController = CreateDefaultSubobject<ACrowdController>(TEXT("CrowdController"));

  OSMActor = CreateDefaultSubobject<AOSMActor>(TEXT("OSMActor"));

  TaggerDelegate = CreateDefaultSubobject<UTaggerDelegate>(TEXT("TaggerDelegate"));
  CarlaSettingsDelegate = CreateDefaultSubobject<UCarlaSettingsDelegate>(TEXT("CarlaSettingsDelegate"));
}

void ACarlaGameModeBase::InitGame(
    const FString &MapName,
    const FString &Options,
    FString &ErrorMessage)
{
  Super::InitGame(MapName, Options, ErrorMessage);

  checkf(
      Episode != nullptr,
      TEXT("Missing episode, can't continue without an episode!"));

#if WITH_EDITOR
    {
      // When playing in editor the map name gets an extra prefix, here we
      // remove it.
      FString CorrectedMapName = MapName;
      constexpr auto PIEPrefix = TEXT("UEDPIE_0_");
      CorrectedMapName.RemoveFromStart(PIEPrefix);
      UE_LOG(LogCarla, Log, TEXT("Corrected map name from %s to %s"), *MapName, *CorrectedMapName);
      Episode->MapName = CorrectedMapName;
    }
#else
  Episode->MapName = MapName;
#endif // WITH_EDITOR

  auto World = GetWorld();
  check(World != nullptr);

  GameInstance = Cast<UCarlaGameInstance>(GetGameInstance());
  checkf(
      GameInstance != nullptr,
      TEXT("GameInstance is not a UCarlaGameInstance, did you forget to set "
           "it in the project settings?"));

  if (TaggerDelegate != nullptr) {
    TaggerDelegate->RegisterSpawnHandler(World);
  } else {
    UE_LOG(LogCarla, Error, TEXT("Missing TaggerDelegate!"));
  }

  if(CarlaSettingsDelegate != nullptr) {
    CarlaSettingsDelegate->ApplyQualityLevelPostRestart();
    CarlaSettingsDelegate->RegisterSpawnHandler(World);
  } else {
    UE_LOG(LogCarla, Error, TEXT("Missing CarlaSettingsDelegate!"));
  }

  if (WeatherClass != nullptr) {
    Episode->Weather = World->SpawnActor<AWeather>(WeatherClass);
  } else {
    UE_LOG(LogCarla, Error, TEXT("Missing weather class!"));
  }

  GameInstance->NotifyInitGame();

  SpawnActorFactories();

  //CreateRoadMap();

  //CreateWaypointMap(MapName);

  // Dependency injection.
  Recorder->SetEpisode(Episode);
  Episode->SetRecorder(Recorder);
  //CrowdController->SetEpisode(Episode);
  //CrowdController->SetRoadMap(&RoadMap);
  //CrowdController->SetWaypointMap(&(WaypointMap.get()));
  //Episode->SetCrowdController(CrowdController);
}

void ACarlaGameModeBase::RestartPlayer(AController *NewPlayer)
{
  if (CarlaSettingsDelegate != nullptr)
  {
    CarlaSettingsDelegate->ApplyQualityLevelPreRestart();
  }

  Super::RestartPlayer(NewPlayer);
}

void ACarlaGameModeBase::BeginPlay()
{
  Super::BeginPlay();

  if (true) { /// @todo If semantic segmentation enabled.
    check(GetWorld() != nullptr);
    ATagger::TagActorsInLevel(*GetWorld(), true);
    TaggerDelegate->SetSemanticSegmentationEnabled();
  }

  Episode->InitializeAtBeginPlay();
  //CrowdController->InitializeAtBeginPlay();
  GameInstance->NotifyBeginEpisode(*Episode);

  if (Episode->Weather != nullptr)
  {
    Episode->Weather->ApplyWeather(carla::rpc::WeatherParameters::Default);
  }

  /// @todo Recorder should not tick here, FCarlaEngine should do it.
  // check if replayer is waiting to autostart
  if (Recorder)
  {
    Recorder->GetReplayer()->CheckPlayAfterMapLoaded();
  }
}

void ACarlaGameModeBase::Tick(float DeltaSeconds)
{
  Super::Tick(DeltaSeconds);

  /// @todo Recorder should not tick here, FCarlaEngine should do it.
  if (Recorder) Recorder->Tick(DeltaSeconds);
  //if (CrowdController) CrowdController->Tick(DeltaSeconds);
}

void ACarlaGameModeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Episode->EndPlay();
  GameInstance->NotifyEndEpisode();

  Super::EndPlay(EndPlayReason);

  if ((CarlaSettingsDelegate != nullptr) && (EndPlayReason != EEndPlayReason::EndPlayInEditor))
  {
    CarlaSettingsDelegate->Reset();
  }
}

void ACarlaGameModeBase::SpawnActorFactories()
{
  auto *World = GetWorld();
  check(World != nullptr);

  for (auto &FactoryClass : ActorFactories)
  {
    if (FactoryClass != nullptr)
    {
      auto *Factory = World->SpawnActor<ACarlaActorFactory>(FactoryClass);
      if (Factory != nullptr)
      {
        Episode->RegisterActorFactory(*Factory);
        ActorFactoryInstances.Add(Factory);
      }
      else
      {
        UE_LOG(LogCarla, Error, TEXT("Failed to spawn actor spawner"));
      }
    }
  }
}

void ACarlaGameModeBase::CreateRoadMap() {

  // Construct RoadMap.
  TArray<FRoadTriangle> RoadTriangles;
  for (TActorIterator<AStaticMeshActor> ActorItr(GetWorld()); ActorItr; ++ActorItr) {
    if (ActorItr->ActorHasTag(TEXT("Road"))){

      // Written with reference to FStaticMeshSectionAreaWeightedTriangleSampler::GetWeights in
      // Runtime/Engine/Private/StaticMesh.cpp of UE 4.22.

      FStaticMeshLODResources* LODResources = &(ActorItr->GetStaticMeshComponent()->GetStaticMesh()->RenderData->LODResources[0]);
      FIndexArrayView Indices = LODResources->IndexBuffer.GetArrayView();
      const FPositionVertexBuffer& PositionVertexBuffer = LODResources->VertexBuffers.PositionVertexBuffer;

      for (int I = 0; I < Indices.Num(); I += 3) {
        FRoadTriangle RoadTriangle(
            ActorItr->GetActorLocation() + ActorItr->GetTransform().TransformVector(PositionVertexBuffer.VertexPosition(Indices[I])),
            ActorItr->GetActorLocation() + ActorItr->GetTransform().TransformVector(PositionVertexBuffer.VertexPosition(Indices[I + 1])),
            ActorItr->GetActorLocation() + ActorItr->GetTransform().TransformVector(PositionVertexBuffer.VertexPosition(Indices[I + 2])));

        // Check for precision errors.
        if (RoadTriangle.GetBounds().Min.X < -1000000) continue;
        if (RoadTriangle.GetBounds().Min.Y < -1000000) continue;
        if (RoadTriangle.GetBounds().Min.Z < -1000000) continue;
        if (RoadTriangle.GetBounds().Max.X > 1000000) continue;
        if (RoadTriangle.GetBounds().Max.Y > 1000000) continue;
        if (RoadTriangle.GetBounds().Max.Z > 1000000) continue;

        RoadTriangles.Emplace(RoadTriangle);
      }
    }
  }
  RoadMap = FRoadMap(RoadTriangles, 10, 100);
}
  
void ACarlaGameModeBase::CreateWaypointMap(const FString& MapName) {
  FString Content = UOpenDrive::LoadXODR(MapName);
  WaypointMap = carla::opendrive::OpenDriveParser::Load(carla::rpc::FromFString(Content));
  if (!WaypointMap.has_value())
  {
    UE_LOG(LogCarla, Error, TEXT("Failed to parse OpenDrive file."));
    return;
  }
}

void ACarlaGameModeBase::RenderRoadMap(const FString& FileName) const {
  RoadMap.RenderBitmap(FileName);  
}

void ACarlaGameModeBase::LoadOSM(const FString& OSMPath) {
  OSMActor->SetOSM(OSMPath);
}
