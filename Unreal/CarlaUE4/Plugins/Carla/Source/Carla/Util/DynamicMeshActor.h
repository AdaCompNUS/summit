#pragma once

#include "ProceduralMeshComponent.h"
#include "DynamicMeshActor.generated.h"

UCLASS(hidecategories = (Physics))
class CARLA_API ADynamicMeshActor : public AActor
{
	GENERATED_BODY()

public: 

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA")
	UProceduralMeshComponent* MeshComponent;
  
  ADynamicMeshActor(const FObjectInitializer& ObjectInitializer);

  void SetMaterial(const FString& Material);

  void SetTriangles(const TArray<FVector>& Triangles);

private:

  UMaterial* MeshMaterial;
};