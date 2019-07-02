#pragma once

class FNode {
public:
  long long ID;
  FVector2D Position;

  FNode(long long ID, const FVector2D& Position) : ID(ID), Position(Position) { }
};

class FRoad {
public:
  long long ID;
  long long SourceNodeID;
  long long DestinationNodeID;
  TArray<long long> ForwardLaneIDs;
  TArray<long long> BackwardLaneIDs;

  FRoad(long long ID, long long SourceNodeID, long long DestinationNodeID, const TArray<long long>& ForwardLaneIDs = TArray<long long>(), const TArray<long long>& BackwardLaneIDs = TArray<long long>())
    : ID(ID),
    SourceNodeID(SourceNodeID),
    DestinationNodeID(DestinationNodeID),
    ForwardLaneIDs(ForwardLaneIDs),
    BackwardLaneIDs(BackwardLaneIDs) { }
};

class FLane {
public:
  long long ID;
  long long RoadID;
  bool IsForward;
  int Index;

  FLane(long long ID, long long RoadID, bool IsForward, int Index)
    : ID(ID),
    RoadID(RoadID),
    IsForward(IsForward),
    Index(Index) { }
};

class FLaneConnection {
public:
  long long ID;
  long long SourceLaneID;
  long long DestinationLaneID;
  double SourceOffset;
  double DestinationOffset;

  FLaneConnection(long long ID, long long SourceLaneID, long long DestinationLaneID, double SourceOffset = 0, double DestinationOffset = 0) 
    : ID(ID),
    SourceLaneID(SourceLaneID),
    DestinationLaneID(DestinationLaneID),
    SourceOffset(SourceOffset),
    DestinationOffset(DestinationOffset) { }
};

class FLaneNetwork {
public:  
  double LaneWidth;
  TMap<long long, FNode> Nodes;
  TMap<long long, FRoad> Roads;
  TMap<long long, FLane> Lanes;
  TMap<long long, FLaneConnection> LaneConnections;

  FLaneNetwork(double LaneWidth=3) : LaneWidth(LaneWidth) { }

  static FLaneNetwork Load(const FString& Path);
};
