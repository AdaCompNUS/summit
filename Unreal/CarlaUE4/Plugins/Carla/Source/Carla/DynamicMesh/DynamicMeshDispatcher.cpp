#include "DynamicMeshDispatcher.h"

uint32_t UDynamicMeshDispatcher::SpawnDynamicMesh(const TArray<FVector>& Triangles, const FString& Material) {
  ADynamicMeshActor* DynamicMeshActor = GetWorld()->SpawnActor<ADynamicMeshActor>();
  DynamicMeshActor->SetMaterial(Material);
  DynamicMeshActor->SetTriangles(Triangles);
  ActorMap.Add(SpawnId++, DynamicMeshActor);
  DynamicMeshActor->OnDestroyed.AddDynamic(this, &UDynamicMeshDispatcher::OnActorDestroyed);

  return SpawnId - 1;
}
  
bool UDynamicMeshDispatcher::DestroyDynamicMesh(uint32_t Id) {
  if (!ActorMap.Contains(Id)) {
    return false;
  }

  ADynamicMeshActor* Actor = ActorMap[Id];
  return Actor->Destroy();
}

void UDynamicMeshDispatcher::OnActorDestroyed(AActor* Actor) {
  const uint32_t* KeyPtr = ActorMap.FindKey(static_cast<ADynamicMeshActor*>(Actor));
  if (KeyPtr != nullptr) {
    ActorMap.Remove(*KeyPtr);
  }
}
