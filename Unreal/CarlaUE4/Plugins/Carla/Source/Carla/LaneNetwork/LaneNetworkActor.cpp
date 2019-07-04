#include "LaneNetworkActor.h"
#include "ConstructorHelpers.h"
#include <vector>

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

  LaneNetwork = FLaneNetwork::Load(LaneNetworkPath);
  LaneIDs.Reset(LaneNetwork.Lanes.Num());
  LaneNetwork.Lanes.GenerateKeyArray(LaneIDs);
  
  TArray<FVector> Vertices;
  TArray<int> TriangleVertices;

  auto AddLineSegment = [&Vertices, &TriangleVertices](FVector2D Start, FVector2D End, float Width) {
    FVector2D Direction = End - Start;
    Direction.Normalize();
    FVector2D Normal = Direction.GetRotated(90);
    
    // Add line rectangle.
    {
      FVector2D V1 = Start + Normal * Width / 2; 
      FVector2D V2 = Start - Normal * Width / 2; 
      FVector2D V3 = End + Normal * Width / 2; 
      FVector2D V4 = End - Normal * Width / 2; 

      int VI1 = Vertices.Add(ToUE(V1));
      int VI2 = Vertices.Add(ToUE(V2));
      int VI3 = Vertices.Add(ToUE(V3));
      int VI4 = Vertices.Add(ToUE(V4));

      TriangleVertices.Add(VI1);
      TriangleVertices.Add(VI2);
      TriangleVertices.Add(VI3);

      TriangleVertices.Add(VI4);
      TriangleVertices.Add(VI3);
      TriangleVertices.Add(VI2);
    }
 
    int N = 16;
    // Add semicircles to start.
    for (int I = 0; I < N; I++) {
      FVector2D V1 = Start;
      FVector2D V2 = Start + Normal.GetRotated((180.0f / N) * I) * Width / 2;
      FVector2D V3 = Start + Normal.GetRotated((180.0f / N) * (I + 1)) * Width / 2;

      int VI1 = Vertices.Add(ToUE(V1));
      int VI2 = Vertices.Add(ToUE(V2));
      int VI3 = Vertices.Add(ToUE(V3));
    
      TriangleVertices.Add(VI1);
      TriangleVertices.Add(VI2);
      TriangleVertices.Add(VI3);
    }
    // Add semicircles to end.
    for (int I = 0; I < N; I++) {
      FVector2D V1 = End;
      FVector2D V2 = End + Normal.GetRotated(-(180.0f / N) * (I + 1)) * Width / 2;
      FVector2D V3 = End + Normal.GetRotated(-(180.0f / N) * I) * Width / 2;

      int VI1 = Vertices.Add(ToUE(V1));
      int VI2 = Vertices.Add(ToUE(V2));
      int VI3 = Vertices.Add(ToUE(V3));
      
      TriangleVertices.Add(VI1);
      TriangleVertices.Add(VI2);
      TriangleVertices.Add(VI3);
    }
  };

  for (const auto& LaneEntry : LaneNetwork.Lanes) {
    const FLane& Lane = LaneEntry.Value;
    boost::optional<float> StartMinOffset = LaneNetwork.GetLaneStartMinOffset(Lane);
    boost::optional<float> EndMinOffset = LaneNetwork.GetLaneEndMinOffset(Lane);
    if (StartMinOffset && EndMinOffset) {
      FVector2D Start = LaneNetwork.GetLaneStart(Lane, *StartMinOffset);
      FVector2D End = LaneNetwork.GetLaneEnd(Lane, *EndMinOffset);
      AddLineSegment(Start, End, LaneNetwork.LaneWidth);
    }
  }

  for (const auto& LaneConnectionEntry : LaneNetwork.LaneConnections) {
    const FLaneConnection& LaneConnection = LaneConnectionEntry.Value;
    FVector2D Source = LaneNetwork.GetLaneStart(
        LaneNetwork.Lanes[LaneConnection.SourceLaneID], 
        LaneConnection.SourceOffset);
    FVector2D Destination = LaneNetwork.GetLaneStart(
        LaneNetwork.Lanes[LaneConnection.DestinationLaneID], 
        LaneConnection.DestinationOffset);
    AddLineSegment(Source, Destination, LaneNetwork.LaneWidth);
  }

  RoadTriangles.Reset(TriangleVertices.Num() / 3);
  RoadTrianglesTree = aabb::Tree(2, 0); // Dimension, inflation thickness.
  for (int I = 0; I < TriangleVertices.Num(); I += 3) {
    RoadTriangles.Emplace(
        Vertices[TriangleVertices[I]],
        Vertices[TriangleVertices[I + 1]],
        Vertices[TriangleVertices[I + 2]]);
    float MinX = FMath::Min3(
        Vertices[TriangleVertices[I]].X,
        Vertices[TriangleVertices[I + 1]].X,
        Vertices[TriangleVertices[I + 2]].X);
    float MaxX = FMath::Max3(
        Vertices[TriangleVertices[I]].X,
        Vertices[TriangleVertices[I + 1]].X,
        Vertices[TriangleVertices[I + 2]].X);
    float MinY = FMath::Min3(
        Vertices[TriangleVertices[I]].Y,
        Vertices[TriangleVertices[I + 1]].Y,
        Vertices[TriangleVertices[I + 2]].Y);
    float MaxY = FMath::Max3(
        Vertices[TriangleVertices[I]].Y,
        Vertices[TriangleVertices[I + 1]].Y,
        Vertices[TriangleVertices[I + 2]].Y);

    std::vector<double> LowerBound = { MinX, MinY };
    std::vector<double> UpperBound = { MaxX, MaxY };
    RoadTrianglesTree.insertParticle(I / 3, LowerBound, UpperBound);
  }

  MeshComponent->bUseComplexAsSimpleCollision = true;
  MeshComponent->CreateMeshSection_LinearColor(0, Vertices, TriangleVertices, {}, {}, {}, {}, true);
  MeshComponent->ContainsPhysicsTriMeshData(true);
  
  UE_LOG(
      LogTemp, 
      Display, 
      TEXT("Loaded lane network. Nodes = %d, Roads = %d, Lanes = %d, LaneConnections = %d, Triangles = %d"), 
      LaneNetwork.Nodes.Num(),
      LaneNetwork.Roads.Num(),
      LaneNetwork.Lanes.Num(),
      LaneNetwork.LaneConnections.Num(),
      RoadTriangles.Num());
}


FVector2D ALaneNetworkActor::RandomVehicleSpawnPoint() const {
  const FLane& Lane = LaneNetwork.Lanes[FMath::RandRange(0, LaneIDs.Num() - 1)];
  boost::optional<float> StartMinOffset = LaneNetwork.GetLaneStartMinOffset(Lane);
  boost::optional<float> EndMinOffset = LaneNetwork.GetLaneEndMinOffset(Lane);
  
  if (!StartMinOffset || !EndMinOffset) {
    return RandomVehicleSpawnPoint();
  }

  FVector2D Start = LaneNetwork.GetLaneStart(Lane, *StartMinOffset);
  FVector2D End = LaneNetwork.GetLaneEnd(Lane, *EndMinOffset);

  return ToUE2D(Start + FMath::RandRange(0.0f, 1.0f) * (End - Start));
}

FRoadMap ALaneNetworkActor::GetRoadMap(const FBox2D Area, float Resolution) const {
  const std::vector<double> LowerBound = { Area.Min.X, Area.Min.Y };
  const std::vector<double> UpperBound = { Area.Max.X, Area.Max.Y };
  
  TArray<FRoadTriangle> MapTriangles;
  for (int I : RoadTrianglesTree.query(aabb::AABB(LowerBound, UpperBound))) {
    MapTriangles.Add(RoadTriangles[I]);
  }

  return FRoadMap(MapTriangles, Resolution, 10);
}
