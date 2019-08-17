#pragma once

#include "DynamicMeshActor.h"
#include "DynamicMeshDispatcher.generated.h"

UCLASS()
class CARLA_API UDynamicMeshDispatcher : public UObject
{
  GENERATED_BODY()

public:

  uint32_t SpawnDynamicMesh(const TArray<FVector>& Triangles, const FString& Material);

  bool DestroyDynamicMesh(uint32_t Id);

private:

  UFUNCTION()
  void OnActorDestroyed(AActor* Actor);

  uint32_t SpawnId = 0;

  TMap<uint32_t, ADynamicMeshActor*> ActorMap;
};
