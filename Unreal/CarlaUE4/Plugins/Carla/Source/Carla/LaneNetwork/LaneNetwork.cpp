#include "LaneNetwork.h"
#include "PlatformFilemanager.h"

FLaneNetwork FLaneNetwork::Load(const FString& Path) {
  IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
  IFileHandle* FileHandle = PlatformFile.OpenRead(*Path);

  // TODO: Error opening file.
  // WARNING: union conversion assumes little endian platform (e.g. x86) and big endian data.
  union {
    uint8 b;
    int32 i;
    int64 l;
    double d;
    uint8 buffer[8];
  } U;

  auto ReadBool = [&U, FileHandle]() -> boost::optional<bool> {
    if (FileHandle->Read(U.buffer, 1)) {
      return boost::optional<bool>(U.b);
    } else {
      return boost::optional<bool>();
    }
  };
  auto ReadByte = [&U, FileHandle]() -> boost::optional<uint8> {
    if (FileHandle->Read(U.buffer, 1)) {
      return boost::optional<uint8>(U.b);
    } else {
      return boost::optional<uint8>();
    }
  };
  auto ReadInt = [&U, FileHandle]() -> boost::optional<int32> {
    if (FileHandle->Read(U.buffer, 4)) {
      Swap(U.buffer[0], U.buffer[3]);
      Swap(U.buffer[1], U.buffer[2]);
      return boost::optional<int32>(U.i);
    } else {
      return boost::optional<int32>();
    }
  };
  auto ReadLong = [&U, FileHandle]() -> boost::optional<int64> {
    if (FileHandle->Read(U.buffer, 8)) {
      Swap(U.buffer[0], U.buffer[7]);
      Swap(U.buffer[1], U.buffer[6]);
      Swap(U.buffer[2], U.buffer[5]);
      Swap(U.buffer[3], U.buffer[4]);
      return boost::optional<int64>(U.l);
    } else {
      return boost::optional<int64>();
    }
  };
  auto ReadDouble = [&U, FileHandle]() -> boost::optional<double> {
    if (FileHandle->Read(U.buffer, 8)) {
      Swap(U.buffer[0], U.buffer[7]);
      Swap(U.buffer[1], U.buffer[6]);
      Swap(U.buffer[2], U.buffer[5]);
      Swap(U.buffer[3], U.buffer[4]);
      return boost::optional<double>(U.d);
    } else {
      return boost::optional<double>();
    }
  };

  FLaneNetwork LaneNetwork(*ReadDouble());

  boost::optional<uint8> ElementType = ReadByte();
  while (ElementType) {
    switch(*ElementType) {
      case 1: { // Node
        int64 ID = *ReadLong();
        FVector2D Pos((float)(*ReadDouble()), (float)(*ReadDouble()));
        LaneNetwork.Nodes.Emplace(ID, FNode(ID, Pos));
        break;
      }
      case 2: { // Road
        int64 ID = *ReadLong();
        int64 SourceNodeID = *ReadLong();
        int64 DestinationNodeID = *ReadLong();
        int32 NumForwardLanes = *ReadInt();
        TArray<int64> ForwardLaneIDs; ForwardLaneIDs.Reserve(NumForwardLanes);
        for (int I = 0; I < NumForwardLanes; I++) {
          ForwardLaneIDs.Add(*ReadLong());
        }
        int32 NumBackwardLanes = *ReadInt();
        TArray<int64> BackwardLaneIDs; BackwardLaneIDs.Reserve(NumBackwardLanes);
        for (int I = 0; I < NumBackwardLanes; I++) {
          BackwardLaneIDs.Add(*ReadLong());
        }
        LaneNetwork.Roads.Emplace(ID, FRoad(ID, SourceNodeID, DestinationNodeID, ForwardLaneIDs, BackwardLaneIDs));
        break;
      }
      case 3: { // Lane
        int64 ID = *ReadLong();
        int64 RoadID = *ReadLong();
        bool IsForward = *ReadBool();
        int32 Index = *ReadInt();
        LaneNetwork.Lanes.Emplace(ID, FLane(ID, RoadID, IsForward, Index));
        LaneNetwork.IncomingLaneConnectionIDsMap.Emplace(ID);
        LaneNetwork.OutgoingLaneConnectionIDsMap.Emplace(ID);
        break;
      }
      case 4: { // Lane Connection
        int64 ID = *ReadLong();
        int64 SourceLaneID = *ReadLong();
        int64 DestinationLaneID = *ReadLong();
        float SourceOffset = (float)(*ReadDouble());
        float DestinationOffset = (float)(*ReadDouble());
        LaneNetwork.LaneConnections.Emplace(ID, FLaneConnection(ID, SourceLaneID, DestinationLaneID, SourceOffset, DestinationOffset));
        LaneNetwork.IncomingLaneConnectionIDsMap.FindOrAdd(DestinationLaneID).Add(ID);
        LaneNetwork.OutgoingLaneConnectionIDsMap.FindOrAdd(SourceLaneID).Add(ID);
        break;
      }
    }
    ElementType = ReadByte();
  }



  // Lookup optimizations.
  for (const TPair<long long, FLane>& LaneEntry : LaneNetwork.Lanes) {
    LaneNetwork.LaneStartMinOffsetMap.Emplace(
        LaneEntry.Key, 
        LaneNetwork.GetLaneStartMinOffset(LaneEntry.Value));
    LaneNetwork.LaneEndMinOffsetMap.Emplace(
        LaneEntry.Key, 
        LaneNetwork.GetLaneEndMinOffset(LaneEntry.Value));
  }

  return LaneNetwork;
}
  
float FLaneNetwork::GetRoadLength(const FRoad& Road) const {
  return (Nodes[Road.DestinationNodeID].Position - Nodes[Road.SourceNodeID].Position).Size();
}

FVector2D FLaneNetwork::GetRoadDirection(const FRoad& Road) const {
  FVector2D Direction = Nodes[Road.DestinationNodeID].Position - Nodes[Road.SourceNodeID].Position;
  Direction.Normalize();
  return Direction;
}

FVector2D FLaneNetwork::GetLaneDirection(const FLane& Lane) const {
  FVector2D Direction = GetRoadDirection(Roads[Lane.RoadID]);
  if (!Lane.IsForward)
    Direction = -Direction;
  return Direction;
}

FVector2D FLaneNetwork::GetLaneStart(const FLane& Lane, float Offset) const {
  const FRoad& Road = Roads[Lane.RoadID];
  FVector2D Direction = GetRoadDirection(Road);
  FVector2D Normal = Direction.GetRotated(90);
  
  FVector2D Center;
  int Index;

  if (Lane.IsForward) {
    Center = Nodes[Road.SourceNodeID].Position + Offset * Direction;
    Index = Lane.Index + Road.BackwardLaneIDs.Num();
  } else {
    Center = Nodes[Road.DestinationNodeID].Position - Offset * Direction;
    Index = Road.BackwardLaneIDs.Num() - 1 - Lane.Index;
  }

  int NumLanes = Road.ForwardLaneIDs.Num() + Road.BackwardLaneIDs.Num();

  return Center + LaneWidth * (Index - 0.5 * (NumLanes - 1)) * Normal;
}

FVector2D FLaneNetwork::GetLaneEnd(const FLane& Lane, float Offset) const {
  const FRoad& Road = Roads[Lane.RoadID];
  FVector2D Direction = GetRoadDirection(Road);
  FVector2D Normal = Direction.GetRotated(90);

  FVector2D Center;
  int Index;

  if (Lane.IsForward) {
    Center = Nodes[Road.DestinationNodeID].Position - Offset * Direction;
    Index = Lane.Index + Road.BackwardLaneIDs.Num();
  } else {
    Center = Nodes[Road.SourceNodeID].Position + Offset * Direction;
    Index = Road.BackwardLaneIDs.Num() - 1 - Lane.Index;
  }

  int NumLanes = Road.ForwardLaneIDs.Num() + Road.BackwardLaneIDs.Num();

  return Center + LaneWidth * (Index - 0.5 * (NumLanes - 1)) * Normal;
}

const TArray<long long>& FLaneNetwork::GetIncomingLaneConnectionIDs(const FLane& Lane) const {
  return IncomingLaneConnectionIDsMap[Lane.ID];
}
  
const TArray<long long>& FLaneNetwork::GetOutgoingLaneConnectionIDs(const FLane& Lane) const {
  return OutgoingLaneConnectionIDsMap[Lane.ID];
}

float FLaneNetwork::GetLaneStartMinOffset(const FLane& Lane) const {
  const float* MapValue = LaneStartMinOffsetMap.Find(Lane.ID);
  if (MapValue) return *MapValue;

  boost::optional<float> MinOffset;
  for (long long LaneConnectionID : GetIncomingLaneConnectionIDs(Lane)) {
    const FLaneConnection& LaneConnection = LaneConnections[LaneConnectionID];
    if (!MinOffset || *MinOffset > LaneConnection.DestinationOffset) {
      MinOffset = boost::optional<float>(LaneConnection.DestinationOffset);
    }
  }
  return MinOffset.get_value_or(0);
}

float FLaneNetwork::GetLaneEndMinOffset(const FLane& Lane) const {
  const float* MapValue = LaneEndMinOffsetMap.Find(Lane.ID);
  if (MapValue) return *MapValue;

  boost::optional<float> MinOffset;
  for (long long LaneConnectionID : GetOutgoingLaneConnectionIDs(Lane)) {
    const FLaneConnection& LaneConnection = LaneConnections[LaneConnectionID];
    if (!MinOffset || *MinOffset > LaneConnection.SourceOffset) {
      MinOffset = boost::optional<float>(LaneConnection.SourceOffset);
    }
  }
  return MinOffset.get_value_or(0);
}
