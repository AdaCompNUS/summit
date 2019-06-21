#pragma once

#include "Components/MeshComponent.h"
#include "Interfaces/Interface_CollisionDataProvider.h"
#include "OSMComponent.generated.h"

UCLASS( meta=(BlueprintSpawnableComponent) , hidecategories = (Physics))
class OSMIMPORT_API UOSMComponent : public UMeshComponent, public IInterface_CollisionDataProvider
{
	GENERATED_BODY()

public:
  
  UOSMComponent(const FObjectInitializer& ObjectInitializer);
		
  void SetOSM(const FString& OSMPath);
};
