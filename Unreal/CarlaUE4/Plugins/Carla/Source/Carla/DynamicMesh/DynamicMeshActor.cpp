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

  TArray<FVector> Normals;
  Normals.SetNum(Triangles.Num(), true);
  for (int I = 0; I < Triangles.Num(); I += 3) {
    Normals[I] = FVector::CrossProduct(Triangles[I + 2] - Triangles[I], Triangles[I + 1] - Triangles[I]).GetSafeNormal();
    Normals[I + 1] = FVector::CrossProduct(Triangles[I] - Triangles[I + 1], Triangles[I + 2] - Triangles[I + 1]).GetSafeNormal();
    Normals[I + 2] = FVector::CrossProduct(Triangles[I + 1] - Triangles[I + 2], Triangles[I] - Triangles[I + 2]).GetSafeNormal();
  }

  MeshComponent->bUseComplexAsSimpleCollision = true;
  MeshComponent->CreateMeshSection_LinearColor(0, Triangles, VertexIndices, Normals, {}, {}, {}, true);
  MeshComponent->ContainsPhysicsTriMeshData(true);
}
