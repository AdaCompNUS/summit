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
  PrimaryActorTick.TickInterval = 1.0f;
  bAllowTickBeforeBeginPlay = false;
}

void ACrowdController::Tick(float DeltaSeconds) {
  for (int I = 0; I < Walkers.Num(); I++) {
    FVector2D Position = Walkers[I].GetPosition2D();
    boost::optional<FVector2D> PreferredVelocity = Walkers[I].GetPreferredVelocity();
    if (!Bounds.IsInside(Position) || !PreferredVelocity) {
      Walkers[I].GetActor()->Destroy();
      Walkers.RemoveAt(I);      
    } else {
      Walkers[I].SetVelocity(*PreferredVelocity);
    }
  }

  while (Walkers.Num() < NumWalkers) {
    FVector2D Position = OccupancyArea.RandPoint();
    FRoutePoint RoutePoint = RouteMap->GetNearestRoutePoint(Position); // Project to lane center.
    Position = RouteMap->GetPosition(RoutePoint);

    const FActorDefinition& ActorDefinition = RandWalkerActorDefinition();
    FActorDescription ActorDescription;
    ActorDescription.UId = ActorDefinition.UId;
    ActorDescription.Id = ActorDefinition.Id;
    ActorDescription.Class = ActorDefinition.Class;

    TSubclassOf<AActor> Class = ActorDefinition.Class;
    AActor* ActorClass = Class.GetDefaultObject();

    FVector Origin, BoxExtent;
    ActorDefinition.Class.GetDefaultObject()->GetActorBounds(true, Origin, BoxExtent);

    FTransform Transform(FVector(Position.X, Position.Y, BoxExtent.Z + 200));

    AActor* Actor= Episode->SpawnActor(Transform, ActorDescription);
    if (Actor) {
      float MaxSpeed = FMath::FRandRange(1.0, 5.0f);
      Walkers.Emplace(RouteMap, Actor, MaxSpeed);
    }
  }
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

void ACrowdController::StartCrowd(int InNumWalkers) {
  // Create occupancy area from occupancy map.
  OccupancyArea = OccupancyMap->GetOccupancyArea(Bounds, 10, 10);
  NumWalkers = InNumWalkers;
}

void ACrowdController::StopCrowd() {

}
