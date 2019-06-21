#include "OSMComponent.h"
#include "Engine/CollisionProfile.h"
#include "OSMFile.h"

UOSMComponent::UOSMComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// We make sure our mesh collision profile name is set to NoCollisionProfileName at initialization. 
	// Because we don't have collision data yet!
	SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);

	// We don't currently need to be ticked.  This can be overridden in a derived class though.
	PrimaryComponentTick.bCanEverTick = false;
	this->bAutoActivate = false;	// NOTE: Components instantiated through C++ are not automatically active, so they'll only tick once and then go to sleep!

	// We don't currently need InitializeComponent() to be called on us.  This can be overridden in a
	// derived class though.
	bWantsInitializeComponent = false;

	// Turn on shadows.  It looks better.
	CastShadow = true;

	// Our mesh is too complicated to be a useful occluder.
	bUseAsOccluder = false;

	// Our mesh can influence navigation.
	bCanEverAffectNavigation = true;
}
  
void UOSMComponent::SetOSM(const FString& OSMPath) {
  UE_LOG(LogTemp, Display, TEXT("OSMPath = %s"), *OSMPath);
	
  FString OSMPathMutable = OSMPath;
  FOSMFile OSMFile;
  if (!OSMFile.LoadOpenStreetMapFile(OSMPathMutable, false, nullptr)) {
    UE_LOG(LogTemp, Error, TEXT("OSM load failed."));
  } else {
    UE_LOG(LogTemp, Display, TEXT("OSM loaded."));
  }
}
