#pragma once

#include "ProceduralMeshComponent.h"
#include "OSMActor.generated.h"

UCLASS(hidecategories = (Physics))
class OSMIMPORT_API AOSMActor : public AActor
{
	GENERATED_BODY()

public: 
  
  AOSMActor(const FObjectInitializer& ObjectInitializer);
		
  void SetOSM(const FString& OSMPath);

private:

	UProceduralMeshComponent* MeshComponent;
};
