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

void ACrowdController::InitializeAtBeginPlay() {
  //RVOSim.setTimeStep(0.2f); // Unnecessary since we are using UE time.
  RVOSim.setAgentDefaults(
      0.0f, // neighborDist
      10.0f, // maxNeighbors
      0.2f, // timeHorizon
      0.2f, // timeHorizonObst
      0.5f, // radius
      2.0f); // maxSpeed

  for (const TArray<FVector2D>& OffroadPolygon : RoadMap->GetOffroadPolygons()) {
    std::vector<RVO::Vector2> Obstacle;
    for (const FVector2D& Vertex : OffroadPolygon) {
      Obstacle.emplace_back(Vertex.X / 100.0f, Vertex.Y / 100.0f);
    }
    RVOSim.addObstacle(Obstacle);
  }

  RVOSim.processObstacles();
}

void ACrowdController::Tick(float DeltaSeconds) {
  while (Walkers.Num() < 300) {
    FVector Point = RoadMap->RandPoint();
    
    const FActorDefinition& ActorDefinition = RandWalkerActorDefinition();
    FActorDescription ActorDescription;
    ActorDescription.UId = ActorDefinition.UId;
    ActorDescription.Id = ActorDefinition.Id;
    ActorDescription.Class = ActorDefinition.Class;

    TSubclassOf<AActor> Class = ActorDefinition.Class;
    AActor* ActorClass = Class.GetDefaultObject();

    FVector Origin, BoxExtent;
    ActorDefinition.Class.GetDefaultObject()->GetActorBounds(true, Origin, BoxExtent);

    FTransform Transform(FVector(Point.X, Point.Y, Point.Z + BoxExtent.Z + 10));

    AActor* Actor= Episode->SpawnActor(Transform, ActorDescription);
    if (Actor) {
      Walkers.Emplace(WaypointMap, Actor);
      RVOSim.addAgent(RVO::Vector2(Point.X / 100.0f, Point.Y / 100.0f));
    }
  }

  for (int I = 0; I < Walkers.Num(); I++) {
    carla::geom::Vector2D Location = Walkers[I].GetLocation();
    carla::geom::Vector2D PreferredVelocity = Walkers[I].GetPreferredVelocity();  
    RVOSim.setAgentPosition(I, RVO::Vector2(Location.x, Location.y));
    RVOSim.setAgentPrefVelocity(I, RVO::Vector2(PreferredVelocity.x, PreferredVelocity.y));
  }
    
  RVOSim.doStep();

  for (int I = 0; I < Walkers.Num(); I++) {
    RVO::Vector2 NewVelocity = RVOSim.getAgentVelocity(I);
    Walkers[I].SetVelocity(carla::geom::Vector2D(NewVelocity.x(), NewVelocity.y()));
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
