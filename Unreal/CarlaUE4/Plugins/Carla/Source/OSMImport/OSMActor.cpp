#include "OSMActor.h"
#include "OSMFile.h"
#include "ConstructorHelpers.h"
#include "LaneNetwork.h"

AOSMActor::AOSMActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
  MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Mesh"));
  MeshComponent->bUseAsyncCooking = true;
	RootComponent = MeshComponent;
  
  static ConstructorHelpers::FObjectFinder<UMaterial> MeshMaterial(TEXT("Material'/Game/Carla/Static/GenericMaterials/Asphalt/M_Asphalt'"));
  MeshComponent->SetMaterial(0, MeshMaterial.Object);
}

void AOSMActor::SetOSM(const FString& OSMPath) {
  LaneNetwork::Load("/home/leeyiyuan/Projects/osm-convert/network.ln");
  return;

  UE_LOG(LogTemp, Display, TEXT("OSMPath = %s"), *OSMPath);
	
  FString OSMPathMutable = OSMPath;
  FOSMFile OSMFile;
  if (!OSMFile.LoadOpenStreetMapFile(OSMPathMutable, false, nullptr)) {
    UE_LOG(LogTemp, Error, TEXT("OSM load failed."));
  } else {
    UE_LOG(LogTemp, Display, TEXT("OSM loaded."));
  }
  
  TArray<FVector> Vertices;
	TArray<int32> Triangles;

  for (const FOSMFile::FOSMWayInfo* OSMWay : OSMFile.Ways) {
    if (OSMWay->WayType == FOSMFile::EOSMWayType::Building) continue;
	
    bool IsValidRoadType = false;
		switch (OSMWay->WayType) {
      // Highway
			case FOSMFile::EOSMWayType::Motorway:
			case FOSMFile::EOSMWayType::Motorway_Link:
			case FOSMFile::EOSMWayType::Trunk:
			case FOSMFile::EOSMWayType::Trunk_Link:
			case FOSMFile::EOSMWayType::Primary:
			case FOSMFile::EOSMWayType::Primary_Link:

      // Major Road
			case FOSMFile::EOSMWayType::Secondary:
			case FOSMFile::EOSMWayType::Secondary_Link:
			case FOSMFile::EOSMWayType::Tertiary:
			case FOSMFile::EOSMWayType::Tertiary_Link:

      // Street
			case FOSMFile::EOSMWayType::Residential:
			case FOSMFile::EOSMWayType::Service:
			case FOSMFile::EOSMWayType::Unclassified:
			case FOSMFile::EOSMWayType::Road:	// @todo: Consider excluding "Road" from our data set, as it could be a highway that wasn't properly tagged in OSM yet
        IsValidRoadType = true;
        break;
		}

    if (!IsValidRoadType) continue;

    if (OSMWay->Nodes.Num() < 2) continue;
    
    for (const FOSMFile::FOSMNodeInfo* OSMNode : OSMWay->Nodes) {
      const FVector2D NodePos = ConvertLatLongToMetersRelative(
          OSMNode->Latitude,
          OSMNode->Longitude,
          OSMFile.AverageLatitude,
          OSMFile.AverageLongitude) * OSMToCentimetersScaleFactor;

      UE_LOG(LogTemp, Display, TEXT("Pos = %f, %f"), NodePos.X, NodePos.Y);

      Vertices.Add(FVector(NodePos.X - 100, NodePos.Y - 100, 0));
      Vertices.Add(FVector(NodePos.X - 100, NodePos.Y + 100, 0));
      Vertices.Add(FVector(NodePos.X + 100, NodePos.Y - 100, 0));
      Vertices.Add(FVector(NodePos.X + 100, NodePos.Y + 100, 0));
      Triangles.Add(Vertices.Num() - 4);
      Triangles.Add(Vertices.Num() - 3);
      Triangles.Add(Vertices.Num() - 2);
      Triangles.Add(Vertices.Num() - 1);
      Triangles.Add(Vertices.Num() - 2);
      Triangles.Add(Vertices.Num() - 3);
    }
  }

	MeshComponent->CreateMeshSection_LinearColor(
      0, 
      Vertices, 
      Triangles, 
      {}, 
      {}, 
      {}, 
      {}, 
      true);
	MeshComponent->ContainsPhysicsTriMeshData(true);
}
