#include "LaneNetworkRouteMap.h"
#include <vector>
  
FLaneNetworkRouteMap::FLaneNetworkRouteMap(const FLaneNetwork* LaneNetwork)
    : LaneNetwork(LaneNetwork) { 

  std::vector<rt_value> IndexEntries; 
  
  for (const TPair<long long, FLane>& LaneEntry : LaneNetwork->Lanes) {
    FVector2D Start = ToUE2D(LaneNetwork->GetLaneStart(
        LaneEntry.Value, 
        LaneNetwork->GetLaneStartMinOffset(LaneEntry.Value)));
    FVector2D End = ToUE2D(LaneNetwork->GetLaneEnd(
        LaneEntry.Value, 
        LaneNetwork->GetLaneEndMinOffset(LaneEntry.Value)));
   
    int SegmentID = Segments.Emplace(true, LaneEntry.Key);
    IndexEntries.emplace_back(
        rt_segment(rt_point(Start.X, Start.Y), rt_point(End.X, End.Y)),
        SegmentID);
    LaneIDToSegmentIDMap.Emplace(LaneEntry.Key, SegmentID);
  }

  for (const TPair<long long, FLaneConnection>& LaneConnectionEntry : LaneNetwork->LaneConnections) {
    FVector2D Source = ToUE2D(LaneNetwork->GetLaneStart(
        LaneNetwork->Lanes[LaneConnectionEntry.Value.SourceLaneID], 
        LaneConnectionEntry.Value.SourceOffset));
    FVector2D Destination = ToUE2D(LaneNetwork->GetLaneStart(
        LaneNetwork->Lanes[LaneConnectionEntry.Value.DestinationLaneID], 
        LaneConnectionEntry.Value.DestinationOffset));

    int SegmentID = Segments.Emplace(false, LaneConnectionEntry.Key);
    IndexEntries.emplace_back(
        rt_segment(rt_point(Source.X, Source.Y), rt_point(Destination.X, Destination.Y)),
        SegmentID);
    LaneConnectionIDToSegmentIDMap.Emplace(LaneConnectionEntry.Key, SegmentID);
  }

  SegmentsIndex = rt_tree(IndexEntries);
}

FRoutePoint FLaneNetworkRouteMap::GetNearestRoutePoint(const FVector2D& Position) const {
  std::vector<rt_value> results;
  SegmentsIndex.query(boost::geometry::index::nearest(rt_point(Position.X, Position.Y), 1), std::back_inserter(results));
  rt_value& result = results[0];
  
  FVector2D SegmentStart(
      boost::geometry::get<0, 0>(result.first),
      boost::geometry::get<0, 1>(result.first));
  FVector2D SegmentEnd(
      boost::geometry::get<1, 0>(result.first),
      boost::geometry::get<1, 1>(result.first));
  FVector2D Direction = (SegmentEnd - SegmentStart).GetSafeNormal();

  float T = FVector2D::DotProduct(
      Position - SegmentStart,
      Direction);
  T = FMath::Min(0.0f, FMath::Max((SegmentEnd - SegmentStart).Size(), T));

  return FRoutePoint(result.second, SegmentStart + T * Direction);
}

TArray<FRoutePoint> FLaneNetworkRouteMap::GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) const {
  TArray<FRoutePoint> NextRoutePoints;

  TQueue<TPair<FRoutePoint, float>> Queue;
  Queue.Enqueue(TPair<FRoutePoint, float>(RoutePoint, LookaheadDistance));

  TPair<FRoutePoint, float> QueueItem;
  while (Queue.Dequeue(QueueItem)) {
    const FRoutePoint& CurrentRoutePoint = QueueItem.Key;
    network_segment CurrentSegment = Segments[CurrentRoutePoint.GetID()];
    float CurrentDistance = QueueItem.Value;
    
    FVector2D NetworkPosition = ToNetwork(CurrentRoutePoint.GetPosition());
    float NetworkDistance = ToNetwork(CurrentDistance);

    if (CurrentSegment.first) { // Segment is lane.
      const FLane& Lane = LaneNetwork->Lanes[CurrentSegment.second];
      
      FVector2D Start = LaneNetwork->GetLaneStart(Lane, 0);
      FVector2D End = LaneNetwork->GetLaneEnd(Lane, 0);
      FVector2D Direction = (End - Start).GetSafeNormal();
      float EndMinOffset = LaneNetwork->GetLaneEndMinOffset(Lane);
      float ProjectionEndOffset = FVector2D::DotProduct(NetworkPosition - End, -Direction);
      FVector2D Projection = End + ProjectionEndOffset * (-Direction);

      if (ProjectionEndOffset - EndMinOffset <= NetworkDistance) {
        NextRoutePoints.Emplace(
            CurrentSegment.first,
            ToUE2D(Projection + NetworkDistance * Direction));
      }

      for (long long LaneConnectionID : LaneNetwork->GetOutgoingLaneConnectionIDs(Lane)) {
        const FLaneConnection& LaneConnection = LaneNetwork->LaneConnections[LaneConnectionID];
        if (LaneConnection.SourceOffset > ProjectionEndOffset) continue;
        if (ProjectionEndOffset - LaneConnection.SourceOffset <= NetworkDistance) continue;

        Queue.Enqueue(TPair<FRoutePoint, float>(
              FRoutePoint(
                  LaneConnectionIDToSegmentIDMap[LaneConnection.ID], 
                  ToUE2D(End + LaneConnection.SourceOffset * (-Direction))),
              ToUE(NetworkDistance - (ProjectionEndOffset - LaneConnection.SourceOffset))));
      }
    } else { // Segment is lane connection.
      const FLaneConnection& LaneConnection = LaneNetwork->LaneConnections[CurrentSegment.second];
    
      FVector2D Source = LaneNetwork->GetLaneStart(
          LaneNetwork->Lanes[LaneConnection.SourceLaneID], 
          LaneConnection.SourceOffset);
      FVector2D Destination = LaneNetwork->GetLaneStart(
          LaneNetwork->Lanes[LaneConnection.DestinationLaneID], 
          LaneConnection.DestinationOffset);
      FVector2D Direction = (Destination - Source).GetSafeNormal();
      FVector2D Projection = Source + FVector2D::DotProduct(NetworkPosition - Source, Direction) * Direction;

      if (NetworkDistance <= (Destination - Projection).Size()) { // Lookahead ends at lane connection.
        NextRoutePoints.Emplace(
            CurrentSegment.first,
            ToUE2D(Projection + NetworkDistance * (Destination - Projection).GetSafeNormal()));
      } else {
        Queue.Enqueue(TPair<FRoutePoint, float>(
              FRoutePoint(LaneIDToSegmentIDMap[LaneConnection.DestinationLaneID], ToUE2D(Destination)),
              ToUE(NetworkDistance - (Destination - Projection).Size())));
      }
    }
  }

  return TArray<FRoutePoint>();
}
