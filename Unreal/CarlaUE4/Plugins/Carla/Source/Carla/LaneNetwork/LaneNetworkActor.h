#pragma once

#include "ProceduralMeshComponent.h"
#include "Map/RoadTriangle.h"
#include "LaneNetworkActor.generated.h"

UCLASS(hidecategories = (Physics))
class CARLA_API ALaneNetworkActor : public AActor
{
	GENERATED_BODY()

public: 

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CARLA")
	UProceduralMeshComponent* MeshComponent;
  
  ALaneNetworkActor(const FObjectInitializer& ObjectInitializer);
		
  void SetLaneNetwork(const FString& LaneNetworkPath);

  const TArray<FRoadTriangle>& GetRoadTriangles() const { return RoadTriangles; }

private:

  UMaterial* MeshMaterial;
  TArray<FRoadTriangle> RoadTriangles;
};
