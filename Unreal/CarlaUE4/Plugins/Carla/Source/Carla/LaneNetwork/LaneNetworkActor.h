#pragma once

#include "LaneNetwork.h"
#include "ProceduralMeshComponent.h"
#include "OccupancyMap/OccupancyGrid.h"
#include "LaneNetworkRouteMap.h"

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

  const FOccupancyMap& GetOccupancyMap() const { return OccupancyMap; }

private:

  FLaneNetwork LaneNetwork;
  FOccupancyMap OccupancyMap;
  UMaterial* MeshMaterial;

  static FVector2D ToUE2D(const FVector2D& Position) { 
    return 100 * FVector2D(Position.Y, Position.X);
  }
  
  static FVector ToUE(const FVector2D& Position) { 
    return 100 * FVector(Position.Y, Position.X, 0);
  }
};
