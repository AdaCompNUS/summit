#include "CrowdController.h"
#include "EngineUtils.h"
#include "Engine/StaticMeshActor.h"
#include <vector>

ACrowdController::ACrowdController(void) {
  PrimaryActorTick.bCanEverTick = true; 
  PrimaryActorTick.TickGroup = TG_PrePhysics;
  bAllowTickBeforeBeginPlay = false;
}

ACrowdController::ACrowdController(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer) {
  PrimaryActorTick.bCanEverTick = true; 
  PrimaryActorTick.TickGroup = TG_PrePhysics;
  PrimaryActorTick.TickInterval = 0.1f;
  bAllowTickBeforeBeginPlay = false;
}

void ACrowdController::Tick(float DeltaSeconds) {
}

const FActorDefinition& ACrowdController::RandWalkerActorDefinition() const {
  const TArray<FActorDefinition>& ActorDefinitions = Episode->GetActorDefinitions();
  TArray<const FActorDefinition*> WalkerActorDefinitions;
  
  for (const FActorDefinition& ActorDefinition : ActorDefinitions) {
    if (FRegexMatcher(FRegexPattern(TEXT("(|.*,)walker(|,.*)")), ActorDefinition.Tags).FindNext()) {
      WalkerActorDefinitions.Add(&ActorDefinition);
    }
  }

  return *WalkerActorDefinitions[FMath::RandRange(0, WalkerActorDefinitions.Num() - 1)];
}

void ACrowdController::StartCrowd(int NumWalkers) {
  // Create occupancy area from occupancy map.
  OccupancyArea = OccupancyMap->GetOccupancyArea(Bounds, 10, 10);

  for (int I = 0; I < NumWalkers; I++) {
    FVector2D Position = OccupancyArea.RandPoint();
    FRoutePoint RoutePoint = RouteMap->GetNearestRoutePoint(Position); // Project to lane center.
    Position = RoutePoint.GetPosition();

    const FActorDefinition& ActorDefinition = RandWalkerActorDefinition();
    FActorDescription ActorDescription;
    ActorDescription.UId = ActorDefinition.UId;
    ActorDescription.Id = ActorDefinition.Id;
    ActorDescription.Class = ActorDefinition.Class;

    TSubclassOf<AActor> Class = ActorDefinition.Class;
    AActor* ActorClass = Class.GetDefaultObject();

    FVector Origin, BoxExtent;
    ActorDefinition.Class.GetDefaultObject()->GetActorBounds(true, Origin, BoxExtent);

    FTransform Transform(FVector(Position.X, Position.Y, BoxExtent.Z + 10));

    AActor* Actor= Episode->SpawnActor(Transform, ActorDescription);
    if (Actor) {
      float MaxSpeed = FMath::FRandRange(1.0, 5.0f);
      Walkers.Emplace(WaypointMap, Actor, MaxSpeed);
    }
  }
}

void ACrowdController::StopCrowd() {

}
