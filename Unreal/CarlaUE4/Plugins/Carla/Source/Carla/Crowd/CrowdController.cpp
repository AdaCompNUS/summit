#include "CrowdController.h"
#include "EngineUtils.h"
#include "Engine/StaticMeshActor.h"
#include "Carla.h"
#include "Regex.h"
#include "Polygon.h"

ACrowdController::ACrowdController(void) {
  PrimaryActorTick.bCanEverTick = false; 
  UE_LOG(LogTemp, Warning, TEXT("Crowd Controller Created"));
}

ACrowdController::ACrowdController(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer) {
  PrimaryActorTick.bCanEverTick = false;
}

void ACrowdController::PostInitializeComponents() {
  Super::PostInitializeComponents();

  TotalRoadArea = 0;
  for (TActorIterator<AStaticMeshActor> ActorItr(GetWorld()); ActorItr; ++ActorItr) {
    if (ActorItr->GetFolderPath().ToString() == TEXT("Roads")) {
      const FPositionVertexBuffer& VertexBuffer = ActorItr->GetStaticMeshComponent()->GetStaticMesh()->RenderData->LODResources[0].VertexBuffers.PositionVertexBuffer;
      TArray<FVector2D> Vertices;
      int Count = VertexBuffer.GetNumVertices();
      for (int Index = 0; Index < Count; Index++) {
        const FVector Vertex3 = GetActorLocation() + GetTransform().TransformVector(VertexBuffer.VertexPosition(Index));
        Vertices.Emplace(Vertex3.X, Vertex3.Y);
      }
      TotalRoadArea += RoadPolygons[RoadPolygons.Emplace(Vertices)].GetArea();
    }
  }

  for (int I = 0; I < 10; I++) {
    FVector2D P = RandomRoadPoint();
    UE_LOG(LogCarla, Display, TEXT("Point = %f, %f"), P.X, P.Y);
  }
}

FVector2D ACrowdController::RandomRoadPoint() const {
  float V = FMath::FRandRange(0, TotalRoadArea);
  
  for (const FPolygon& Polygon : RoadPolygons){
    V -= Polygon.GetArea();
    if (V <= 0) return Polygon.RandomPoint();
  }

  return RoadPolygons.Last().RandomPoint();
}
