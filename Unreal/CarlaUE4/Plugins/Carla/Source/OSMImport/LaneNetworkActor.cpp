#include "LaneNetworkActor.h"
#include "ConstructorHelpers.h"
#include "LaneNetwork.h"

ALaneNetworkActor::ALaneNetworkActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
  MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Mesh"));
  MeshComponent->bUseAsyncCooking = true;
	RootComponent = MeshComponent;
  
  static ConstructorHelpers::FObjectFinder<UMaterial> MeshMaterial(TEXT("Material'/Game/Carla/Static/GenericMaterials/Asphalt/M_Asphalt'"));
  MeshComponent->SetMaterial(0, MeshMaterial.Object);
}

void ALaneNetworkActor::SetLaneNetwork(const FString& LaneNetworkPath) {
  UE_LOG(LogTemp, Display, TEXT("Loading lane network from: %s"), *LaneNetworkPath);
}
