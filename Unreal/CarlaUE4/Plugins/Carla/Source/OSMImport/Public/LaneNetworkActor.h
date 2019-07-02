#pragma once

#include "ProceduralMeshComponent.h"
#include "LaneNetworkActor.generated.h"

UCLASS(hidecategories = (Physics))
class OSMIMPORT_API ALaneNetworkActor : public AActor
{
	GENERATED_BODY()

public: 

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	UProceduralMeshComponent* MeshComponent;
  
  ALaneNetworkActor(const FObjectInitializer& ObjectInitializer);
		
  void SetLaneNetwork(const FString& LaneNetworkPath);

private:

  UMaterial* MeshMaterial;
};
