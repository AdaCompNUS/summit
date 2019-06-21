#include "OSMActor.h"
#include "OSMFile.h"

AOSMActor::AOSMActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
  MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("GeneratedMesh"));
  MeshComponent->bUseAsyncCooking = true;
	RootComponent = MeshComponent;
}

void AOSMActor::SetOSM(const FString& OSMPath) {
  UE_LOG(LogTemp, Display, TEXT("OSMPath = %s"), *OSMPath);
	
  FString OSMPathMutable = OSMPath;
  FOSMFile OSMFile;
  if (!OSMFile.LoadOpenStreetMapFile(OSMPathMutable, false, nullptr)) {
    UE_LOG(LogTemp, Error, TEXT("OSM load failed."));
  } else {
    UE_LOG(LogTemp, Display, TEXT("OSM loaded."));
  }
}
