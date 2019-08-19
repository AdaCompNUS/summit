#include "DynamicMeshDispatcher.h"

uint32 ADynamicMeshDispatcher::SpawnDynamicMesh(const TArray<FVector>& Triangles, const FString& Material) {
  if (GetWorld() == nullptr) {
    UE_LOG(LogCarla, Display, TEXT("NULL WORLD"));
    return -1;
  }

  ADynamicMeshActor* DynamicMeshActor = GetWorld()->SpawnActor<ADynamicMeshActor>();
  DynamicMeshActor->SetMaterial(Material);
  DynamicMeshActor->SetTriangles(Triangles);
  ActorMap.Add(SpawnId++, DynamicMeshActor);
  DynamicMeshActor->OnDestroyed.AddDynamic(this, &ADynamicMeshDispatcher::OnActorDestroyed);

  return SpawnId - 1;
}
  
bool ADynamicMeshDispatcher::DestroyDynamicMesh(uint32 Id) {
  if (!ActorMap.Contains(Id)) {
    return false;
  }

  ADynamicMeshActor* Actor = ActorMap[Id];
  return Actor->Destroy();
}

void ADynamicMeshDispatcher::OnActorDestroyed(AActor* Actor) {
  const uint32* KeyPtr = ActorMap.FindKey(static_cast<ADynamicMeshActor*>(Actor));
  if (KeyPtr != nullptr) {
    ActorMap.Remove(*KeyPtr);
  }
}
