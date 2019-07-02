#pragma once

class Node {
public:
  long long ID;
  FVector2D Position;

  Node(long long ID, const FVector2D& Position) : ID(ID), Position(Position) { }
};

class Road {
public:
  long long ID;
  long long SourceNodeID;
  long long DestinationNodeID;
  TArray<long long> ForwardLaneIDs;
  TArray<long long> BackwardLaneIDs;

  Road(long long ID, long long SourceNodeID, long long DestinationNodeID, const TArray<long long>& ForwardLaneIDs = TArray<long long>(), const TArray<long long>& BackwardLaneIDs = TArray<long long>())
    : ID(ID),
    SourceNodeID(SourceNodeID),
    DestinationNodeID(DestinationNodeID),
    ForwardLaneIDs(ForwardLaneIDs),
    BackwardLaneIDs(BackwardLaneIDs) { }
};

class Lane {
public:
  long long ID;
  long long RoadID;
  bool IsForward;
  int Index;

  Lane(long long ID, long long RoadID, bool IsForward, int Index)
    : ID(ID),
    RoadID(RoadID),
    IsForward(IsForward),
    Index(Index) { }
};

class LaneConnection {
public:
  long long ID;
  long long SourceLaneID;
  long long DestinationLaneID;
  double SourceOffset;
  double DestinationOffset;

  LaneConnection(long long ID, long long SourceLaneID, long long DestinationLaneID, double SourceOffset = 0, double DestinationOffset = 0) 
    : ID(ID),
    SourceLaneID(SourceLaneID),
    DestinationLaneID(DestinationLaneID),
    SourceOffset(SourceOffset),
    DestinationOffset(DestinationOffset) { }
};

class LaneNetwork {
public:  
  double LaneWidth;
  TMap<long long, Node> Nodes;
  TMap<long long, Road> Roads;
  TMap<long long, Lane> Lanes;
  TMap<long long, LaneConnection> LaneConnections;

  LaneNetwork(double LaneWidth=3) : LaneWidth(LaneWidth) { }

  static LaneNetwork Load(const FString& Path);
};
