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
  FVector Point = RoadMap.RandPoint();
  
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

  Episode->SpawnActor(Transform, ActorDescription);
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
  
  // Construct RoadMap.
  
  TArray<FRoadTriangle> RoadTriangles;
  for (TActorIterator<AStaticMeshActor> ActorItr(GetWorld()); ActorItr; ++ActorItr) {
    FRegexPattern Pattern(TEXT("\\/Game\\/Carla\\/Static\\/Road\\/RoadsTown03\\/SM_RoadTown03_[0-9]+\\.SM_RoadTown03_[0-9]+"));
    if (FRegexMatcher(Pattern, ActorItr->GetDetailedInfo()).FindNext()){

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
  RoadMap = FRoadMap(RoadTriangles); 
}
