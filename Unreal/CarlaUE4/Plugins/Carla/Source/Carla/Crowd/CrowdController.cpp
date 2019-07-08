#include "CrowdController.h"
#include "EngineUtils.h"
#include "Engine/StaticMeshActor.h"
#include <vector>
#include "RVO/RVO.h"

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
      float MaxSpeed = FMath::FRandRange(100.0f, 300.0f);
      Walkers.Emplace(RouteMap, Actor, MaxSpeed);
    }
  }

  for (int I = 0; I < Walkers.Num(); I++) {	
    FVector2D Position = Walkers[I].GetPosition2D();
    boost::optional<FVector2D> PreferredVelocity = Walkers[I].GetPreferredVelocity();

    RVOSim.setAgentPosition(I, RVO::Vector2(Position.X, Position.Y));	
    if (PreferredVelocity) {	
      RVOSim.setAgentPrefVelocity(I, RVO::Vector2(PreferredVelocity->X, PreferredVelocity->Y));	
    } else {	
      // TODO ORCA could fail here as agent cannot partake in reciprocal avoidance.	
      RVOSim.setAgentPrefVelocity(I, RVO::Vector2(0, 0));	
    }	
  }	
    	
  RVOSim.doStep();

  for (int I = 0; I < Walkers.Num(); I++) {
    RVO::Vector2 NewVelocity = RVOSim.getAgentVelocity(I);
    Walkers[I].SetVelocity(FVector2D(NewVelocity.x(), NewVelocity.y()));
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

  RVOSim.setTimeStep(0.2f); // Unnecessary since we are using UE time.
  RVOSim.setAgentDefaults(
      1000.0f, // neighborDist
      20, // maxNeighbors
      2.0f, // timeHorizon
      0.3f, // timeHorizonObst
      40.0f, // radius
      300.0f); // maxSpeed

  for (const TArray<FVector2D>& OffroadPolygon : OccupancyArea.OffroadPolygons) {
    std::vector<RVO::Vector2> Obstacle;
    for (const FVector2D& Vertex : OffroadPolygon) {
      Obstacle.emplace_back(Vertex.X, Vertex.Y);
    }
    RVOSim.addObstacle(Obstacle);
  }

  RVOSim.processObstacles();

  for (int I = 0; I < NumWalkers; I++) {
    RVOSim.addAgent(RVO::Vector2(0, 0));
  }
}

void ACrowdController::StopCrowd() {

}
