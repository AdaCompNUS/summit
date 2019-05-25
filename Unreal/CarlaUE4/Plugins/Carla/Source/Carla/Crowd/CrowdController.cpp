#include "CrowdController.h"
#include "EngineUtils.h"
#include "Engine/StaticMeshActor.h"
#include "Regex.h"

ACrowdController::ACrowdController(void) {
  PrimaryActorTick.bCanEverTick = true; 
  PrimaryActorTick.TickGroup = TG_PrePhysics;
  bAllowTickBeforeBeginPlay = false;
}

ACrowdController::ACrowdController(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer) {
  PrimaryActorTick.bCanEverTick = true; 
  PrimaryActorTick.TickGroup = TG_PrePhysics;
  bAllowTickBeforeBeginPlay = false;
}

void ACrowdController::Tick(float DeltaSeconds) {
  //FVector Point = RoadMap.RandPoint();
  
  //const FActorDefinition& ActorDefinition = RandWalkerActorDefinition();
  //FActorDescription ActorDescription;
  //ActorDescription.UId = ActorDefinition.UId;
  //ActorDescription.Id = ActorDefinition.Id;
  //ActorDescription.Class = ActorDefinition.Class;

  //TSubclassOf<AActor> Class = ActorDefinition.Class;
  //AActor* ActorClass = Class.GetDefaultObject();

  //FVector Origin, BoxExtent;
  //ActorDefinition.Class.GetDefaultObject()->GetActorBounds(true, Origin, BoxExtent);

  //FTransform Transform(FVector(Point.X, Point.Y, Point.Z + BoxExtent.Z + 10));

  //Episode->SpawnActor(Transform, ActorDescription);
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

void ACrowdController::Initialize() {
}
