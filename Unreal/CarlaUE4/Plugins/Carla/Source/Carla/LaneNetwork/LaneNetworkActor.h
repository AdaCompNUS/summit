#pragma once

#include "LaneNetwork.h"
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

  FVector2D RandomVehicleSpawnPoint() const;

private:

  FLaneNetwork LaneNetwork;
  TArray<long long> LaneIDs;
  UMaterial* MeshMaterial;
  TArray<FRoadTriangle> RoadTriangles;

  static FVector2D ToUE2D(const FVector2D& Position) { 
    return 100 * FVector2D(Position.Y, Position.X);
  }
  
  static FVector ToUE(const FVector2D& Position) { 
    return 100 * FVector(Position.Y, Position.X, 0);
  }
};
