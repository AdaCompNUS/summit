#include "DynamicMeshActor.h"
#include "ConstructorHelpers.h"

ADynamicMeshActor::ADynamicMeshActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
  MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Mesh"));
  MeshComponent->bUseAsyncCooking = true;
	RootComponent = MeshComponent;
}

void ADynamicMeshActor::SetMaterial(const FString& Material) {
  MeshComponent->SetMaterial(0, Cast<UMaterial>(StaticLoadObject(UMaterial::StaticClass(), NULL, *Material)));
}

void ADynamicMeshActor::SetTriangles(const TArray<FVector>& Triangles) {
  TArray<int> VertexIndices;
  VertexIndices.SetNum(Triangles.Num(), true);
  for (int I = 0; I < Triangles.Num(); I++) {
    VertexIndices[I] = I;
  }

  MeshComponent->bUseComplexAsSimpleCollision = true;
  MeshComponent->CreateMeshSection_LinearColor(0, Triangles, VertexIndices, {}, {}, {}, {}, true);
  MeshComponent->ContainsPhysicsTriMeshData(true);
}
