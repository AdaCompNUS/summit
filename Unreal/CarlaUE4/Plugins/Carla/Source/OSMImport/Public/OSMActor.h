#pragma once

#include "OSMActor.generated.h"

UCLASS(hidecategories = (Physics))
class OSMIMPORT_API AOSMActor : public AActor
{
	GENERATED_BODY()

public: 
  
  AOSMActor(const FObjectInitializer& ObjectInitializer);

  FORCEINLINE class UOSMComponent* GetOSMComponent() { return OSMComponent; }

private:

	class UOSMComponent* OSMComponent;
};
