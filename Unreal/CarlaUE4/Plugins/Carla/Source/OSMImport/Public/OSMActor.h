#pragma once

#include "OSMActor.generated.h"

UCLASS(hidecategories = (Physics))
class OSMIMPORT_API AOSMActor : public AActor
{
	GENERATED_UCLASS_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	class UOSMComponent* OSMComponent;

public: 
	FORCEINLINE class UOSMComponent* GetOSMComponent() { return OSMComponent; }
};
