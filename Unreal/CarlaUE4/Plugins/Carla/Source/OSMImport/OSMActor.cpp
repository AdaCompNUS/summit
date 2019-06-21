#include "OSMActor.h"
#include "OSMComponent.h"

AOSMActor::AOSMActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	OSMComponent = CreateDefaultSubobject<UOSMComponent>(TEXT("OSMComp"));
	RootComponent = OSMComponent;
}
