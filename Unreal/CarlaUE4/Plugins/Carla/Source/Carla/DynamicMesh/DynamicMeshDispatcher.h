#pragma once

#include "DynamicMeshActor.h"
#include "DynamicMeshDispatcher.generated.h"

UCLASS()
class CARLA_API ADynamicMeshDispatcher : public AActor
{
  GENERATED_BODY()

public:

  uint32 SpawnDynamicMesh(const TArray<FVector>& Triangles, const FString& Material);

  bool DestroyDynamicMesh(uint32 Id);

private:

  UFUNCTION()
  void OnActorDestroyed(AActor* Actor);

  int32 SpawnId = 0;
  
  UPROPERTY()
  TMap<uint32, ADynamicMeshActor*> ActorMap;
};
