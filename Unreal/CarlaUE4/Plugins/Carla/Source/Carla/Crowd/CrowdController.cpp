#include "CrowdController.h"
#include "EngineUtils.h"
#include "Engine/StaticMeshActor.h"
#include "Regex.h"
#include "Polygon.h"

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
  if (X >= RoadPolygons.Num()) {
    return;
  }

  FVector2D Point = RoadPolygons[X].RandPoint();
  float Z = RoadPolygons[X].GetMaxZ();
  
  const FActorDefinition& ActorDefinition = RandWalkerActorDefinition();
  FActorDescription ActorDescription;
  ActorDescription.UId = ActorDefinition.UId;
  ActorDescription.Id = ActorDefinition.Id;
  ActorDescription.Class = ActorDefinition.Class;

  TSubclassOf<AActor> Class = ActorDefinition.Class;
  AActor* ActorClass = Class.GetDefaultObject();

  FVector Origin, BoxExtent;
  ActorDefinition.Class.GetDefaultObject()->GetActorBounds(true, Origin, BoxExtent);
  Z += BoxExtent.Z + 10;
  
  FTransform Transform(FVector(Point.X, Point.Y, Z));

  Episode->SpawnActor(Transform, ActorDescription);
  X++;
}

FVector ACrowdController::RandRoadPoint() const {
  float V = FMath::FRandRange(0, TotalRoadArea);
  int I = 0;
  for (; V > 0 && I < RoadPolygons.Num(); I++) {
    V -= RoadPolygons[I].GetArea();
  }

  const FPolygon& Polygon = RoadPolygons[I];
  FVector2D Point2D = Polygon.RandPoint();
  FVector Point;
  Point.X = Point2D.X;
  Point.Y = Point2D.Y;
  Point.Z = Polygon.GetMaxZ();
  return Point;
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
  TotalRoadArea = 0;
  for (TActorIterator<AStaticMeshActor> ActorItr(GetWorld()); ActorItr; ++ActorItr) {
    FRegexPattern Pattern(TEXT("\\/Game\\/Carla\\/Static\\/Road\\/RoadsTown03\\/SM_RoadTown03_[0-9]+\\.SM_RoadTown03_[0-9]+"));
    if (FRegexMatcher(Pattern, ActorItr->GetDetailedInfo()).FindNext()){
      const FPositionVertexBuffer& VertexBuffer = ActorItr->GetStaticMeshComponent()->GetStaticMesh()->RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer;
      TArray<FVector> Vertices;
      int Count = VertexBuffer.GetNumVertices();
      for (int Index = 0; Index < Count; Index++) {
        Vertices.Emplace(ActorItr->GetTransform().TransformVector(VertexBuffer.VertexPosition(Index)));
      }
      TotalRoadArea += RoadPolygons[RoadPolygons.Emplace(Vertices)].GetArea();
      UE_LOG(LogCarla, Display, TEXT("Area = %f, Polygon = %s"), RoadPolygons.Top().GetArea(), *ActorItr->GetDetailedInfo());
    }
  }
}
