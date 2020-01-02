#pragma once

#include "DynamicMeshActor.h"
#include "DynamicMeshDispatcher.generated.h"

UCLASS()
class CARLA_API ADynamicMeshDispatcher : public AActor
{
  GENERATED_BODY()

public:

  uint32 SpawnDynamicMesh(const TArray<FVector>& Triangles, const FString& Material, uint8_t SemanticSegmentationLabel);

  uint32 SpawnDynamicTileMesh(FVector BoundsMin, FVector BoundsMax, const TArray<uint8_t>& Data, uint8_t SemanticSegmentationLabel);

  bool DestroyDynamicMesh(uint32 Id);

private:

  UFUNCTION()
  void OnActorDestroyed(AActor* Actor);

  int32 SpawnId = 0;
  
  UPROPERTY()
  TMap<uint32, ADynamicMeshActor*> ActorMap;
};
