// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Game/CarlaEpisode.h"

#include "Carla/Sensor/Sensor.h"
#include "Carla/Util/BoundingBoxCalculator.h"
#include "Carla/Util/RandomEngine.h"
#include "Carla/Vehicle/VehicleSpawnPoint.h"

#include "EngineUtils.h"
#include "Engine/StaticMeshActor.h"
#include "GameFramework/SpectatorPawn.h"
#include "Kismet/GameplayStatics.h"

static FString UCarlaEpisode_GetTrafficSignId(ETrafficSignState State)
{
  using TSS = ETrafficSignState;
  switch (State)
  {
    case TSS::TrafficLightRed:
    case TSS::TrafficLightYellow:
    case TSS::TrafficLightGreen:  return TEXT("traffic.traffic_light");
    case TSS::SpeedLimit_30:      return TEXT("traffic.speed_limit.30");
    case TSS::SpeedLimit_40:      return TEXT("traffic.speed_limit.40");
    case TSS::SpeedLimit_50:      return TEXT("traffic.speed_limit.50");
    case TSS::SpeedLimit_60:      return TEXT("traffic.speed_limit.60");
    case TSS::SpeedLimit_90:      return TEXT("traffic.speed_limit.90");
    case TSS::SpeedLimit_100:     return TEXT("traffic.speed_limit.100");
    case TSS::SpeedLimit_120:     return TEXT("traffic.speed_limit.120");
    case TSS::SpeedLimit_130:     return TEXT("traffic.speed_limit.130");
    case TSS::StopSign:           return TEXT("traffic.stop");
    case TSS::YieldSign:          return TEXT("traffic.yield");
    default:                      return TEXT("traffic.unknown");
  }
}

UCarlaEpisode::UCarlaEpisode(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer),
    Id(URandomEngine::GenerateRandomId())
{
  ActorDispatcher = CreateDefaultSubobject<UActorDispatcher>(TEXT("ActorDispatcher"));
}

bool UCarlaEpisode::LoadNewEpisode(const FString &MapString)
{
  FString FinalPath = MapString.IsEmpty() ? GetMapName() : MapString;
  bool bIsFileFound = false;
  if (MapString.StartsWith("/Game"))
  {
    // Full path
    if (!MapString.EndsWith(".umap"))
    {
      FinalPath += ".umap";
    }
    // Some conversions...
    FinalPath = FinalPath.Replace(TEXT("/Game/"), *FPaths::ProjectContentDir());
    if (FPaths::FileExists(IFileManager::Get().ConvertToAbsolutePathForExternalAppForRead(*FinalPath)))
    {
      bIsFileFound = true;
      FinalPath = MapString;
    }
  }
  else
  {
    if (MapString.Contains("/"))
    {
      bIsFileFound = false;
    }
    else
    {
      // Find the full path under Carla
      TArray<FString> TempStrArray, PathList;
      if (!MapString.EndsWith(".umap"))
      {
        FinalPath += ".umap";
      }
      IFileManager::Get().FindFilesRecursive(PathList, *FPaths::ProjectContentDir(), *FinalPath, true, false, false);
      if (PathList.Num() > 0)
      {
        FinalPath = PathList[0];
        FinalPath.ParseIntoArray(TempStrArray, TEXT("Content/"), true);
        FinalPath = TempStrArray[1];
        FinalPath.ParseIntoArray(TempStrArray, TEXT("."), true);
        FinalPath = "/Game/" + TempStrArray[0];
        bIsFileFound = true;
      }
    }
  }
  if (bIsFileFound)
  {
    UE_LOG(LogCarla, Warning, TEXT("Loading a new episode: %s"), *FinalPath);
    UGameplayStatics::OpenLevel(GetWorld(), *FinalPath, true);
    ApplySettings(FEpisodeSettings{});
  }
  return bIsFileFound;
}

void UCarlaEpisode::ApplySettings(const FEpisodeSettings &Settings)
{
  FCarlaStaticDelegates::OnEpisodeSettingsChange.Broadcast(Settings);
  EpisodeSettings = Settings;
}

TArray<FTransform> UCarlaEpisode::GetRecommendedSpawnPoints() const
{
  TArray<FTransform> SpawnPoints;
  for (TActorIterator<AVehicleSpawnPoint> It(GetWorld()); It; ++It)
  {
    SpawnPoints.Add(It->GetActorTransform());
  }
  return SpawnPoints;
}

carla::rpc::Actor UCarlaEpisode::SerializeActor(FActorView ActorView) const
{
  carla::rpc::Actor Actor;
  if (ActorView.IsValid())
  {
    Actor = ActorView.GetActorInfo()->SerializedData;
    auto Parent = ActorView.GetActor()->GetOwner();
    if (Parent != nullptr)
    {
      Actor.parent_id = FindActor(Parent).GetActorId();
    }
  }
  else
  {
    UE_LOG(LogCarla, Warning, TEXT("Trying to serialize invalid actor"));
  }
  return Actor;
}

void UCarlaEpisode::AttachActors(
    AActor *Child,
    AActor *Parent,
    EAttachmentType InAttachmentType)
{
  UActorAttacher::AttachActors(Child, Parent, InAttachmentType);

  // recorder event
  if (Recorder->IsEnabled())
  {
    CarlaRecorderEventParent RecEvent
    {
      FindActor(Child).GetActorId(),
      FindActor(Parent).GetActorId()
    };
    Recorder->AddEvent(std::move(RecEvent));
  }
}

void UCarlaEpisode::InitializeAtBeginPlay()
{
  auto World = GetWorld();
  check(World != nullptr);
  auto PlayerController = UGameplayStatics::GetPlayerController(GetWorld(), 0);
  if (PlayerController == nullptr)
  {
    UE_LOG(LogCarla, Error, TEXT("Can't find player controller!"));
    return;
  }
  Spectator = PlayerController->GetPawn();
  if (Spectator != nullptr)
  {
    FActorDescription Description;
    Description.Id = TEXT("spectator");
    Description.Class = Spectator->GetClass();
    ActorDispatcher->RegisterActor(*Spectator, Description);
  }
  else
  {
    UE_LOG(LogCarla, Error, TEXT("Can't find spectator!"));
  }

  for (TActorIterator<ATrafficSignBase> It(World); It; ++It)
  {
    ATrafficSignBase *Actor = *It;
    check(Actor != nullptr);
    FActorDescription Description;
    Description.Id = UCarlaEpisode_GetTrafficSignId(Actor->GetTrafficSignState());
    Description.Class = Actor->GetClass();
    ActorDispatcher->RegisterActor(*Actor, Description);
  }

  for (TActorIterator<AStaticMeshActor> It(World); It; ++It)
  {
    auto Actor = *It;
    check(Actor != nullptr);
    auto MeshComponent = Actor->GetStaticMeshComponent();
    check(MeshComponent != nullptr);
    if (MeshComponent->Mobility == EComponentMobility::Movable)
    {
      FActorDescription Description;
      Description.Id = TEXT("static.prop");
      Description.Class = Actor->GetClass();
      ActorDispatcher->RegisterActor(*Actor, Description);
    }
  }
}

void UCarlaEpisode::EndPlay(void)
{
  // stop recorder and replayer
  if (Recorder)
  {
    Recorder->Stop();
    if (Recorder->GetReplayer()->IsEnabled())
    {
      Recorder->GetReplayer()->Stop();
    }
  }
}

std::string UCarlaEpisode::StartRecorder(std::string Name)
{
  std::string result;

  if (Recorder)
  {
    result = Recorder->Start(Name, MapName);
  }
  else
  {
    result = "Recorder is not ready";
  }

  return result;
}
