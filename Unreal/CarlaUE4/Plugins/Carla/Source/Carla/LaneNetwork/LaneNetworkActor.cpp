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

  FLaneNetwork LaneNetwork = FLaneNetwork::Load(LaneNetworkPath);
  UE_LOG(
      LogTemp, 
      Display, 
      TEXT("Loaded lane network. Nodes = %d, Roads = %d, Lanes = %d, LaneConnections = %d"), 
      LaneNetwork.Nodes.Num(),
      LaneNetwork.Roads.Num(),
      LaneNetwork.Lanes.Num(),
      LaneNetwork.LaneConnections.Num());
    
  TArray<FVector> Vertices;
  TArray<int> Triangles;

  auto AddLineSegment = [&Vertices, &Triangles](FVector2D Start, FVector2D End, float Width) {
    FVector2D Direction = End - Start;
    Direction.Normalize();
    FVector2D Normal = Direction.GetRotated(90);
    
    // Add line rectangle.
    {
      FVector2D V1 = Start + Normal * Width / 2; 
      FVector2D V2 = Start - Normal * Width / 2; 
      FVector2D V3 = End + Normal * Width / 2; 
      FVector2D V4 = End - Normal * Width / 2; 

      int VI1 = Vertices.Add(100 * FVector(V1.Y, V1.X, 0));
      int VI2 = Vertices.Add(100 * FVector(V2.Y, V2.X, 0));
      int VI3 = Vertices.Add(100 * FVector(V3.Y, V3.X, 0));
      int VI4 = Vertices.Add(100 * FVector(V4.Y, V4.X, 0));

      Triangles.Add(VI1);
      Triangles.Add(VI2);
      Triangles.Add(VI3);

      Triangles.Add(VI4);
      Triangles.Add(VI3);
      Triangles.Add(VI2);
    }
 
    int N = 16;
    // Add semicircles to start.
    for (int I = 0; I < N; I++) {
      FVector2D V1 = Start;
      FVector2D V2 = Start + Normal.GetRotated((180.0f / N) * I) * Width / 2;
      FVector2D V3 = Start + Normal.GetRotated((180.0f / N) * (I + 1)) * Width / 2;

      int VI1 = Vertices.Add(100 * FVector(V1.Y, V1.X, 0));
      int VI2 = Vertices.Add(100 * FVector(V2.Y, V2.X, 0));
      int VI3 = Vertices.Add(100 * FVector(V3.Y, V3.X, 0));
    
      Triangles.Add(VI1);
      Triangles.Add(VI2);
      Triangles.Add(VI3);
    }
    // Add semicircles to end.
    for (int I = 0; I < N; I++) {
      FVector2D V1 = End;
      FVector2D V2 = End + Normal.GetRotated(-(180.0f / N) * (I + 1)) * Width / 2;
      FVector2D V3 = End + Normal.GetRotated(-(180.0f / N) * I) * Width / 2;

      int VI1 = Vertices.Add(100 * FVector(V1.Y, V1.X, 0));
      int VI2 = Vertices.Add(100 * FVector(V2.Y, V2.X, 0));
      int VI3 = Vertices.Add(100 * FVector(V3.Y, V3.X, 0));
      
      Triangles.Add(VI1);
      Triangles.Add(VI2);
      Triangles.Add(VI3);
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

  MeshComponent->CreateMeshSection_LinearColor(0, Vertices, Triangles, {}, {}, {}, {}, true);
  MeshComponent->ContainsPhysicsTriMeshData(true);
}
