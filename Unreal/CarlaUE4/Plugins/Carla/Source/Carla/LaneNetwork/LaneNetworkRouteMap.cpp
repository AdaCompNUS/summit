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
    FVector2D Source = ToUE2D(LaneNetwork->GetLaneEnd(
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

FVector2D FLaneNetworkRouteMap::GetPosition(const FRoutePoint& RoutePoint) const {
  const network_segment& Segment = Segments[RoutePoint.GetID()];
  
  if (Segment.first) {
    const FLane& Lane = LaneNetwork->Lanes[Segment.second];

    FVector2D Start = ToUE2D(LaneNetwork->GetLaneStart(
        Lane, 
        LaneNetwork->GetLaneStartMinOffset(Lane)));
    FVector2D End = ToUE2D(LaneNetwork->GetLaneEnd(
        Lane, 
        LaneNetwork->GetLaneEndMinOffset(Lane)));
    FVector2D Direction = (End - Start).GetSafeNormal();

    return Start + RoutePoint.GetOffset() * Direction;
  } else {
    const FLaneConnection& LaneConnection = LaneNetwork->LaneConnections[Segment.second];

    FVector2D Source = ToUE2D(LaneNetwork->GetLaneEnd(
        LaneNetwork->Lanes[LaneConnection.SourceLaneID], 
        LaneConnection.SourceOffset));
    FVector2D Destination = ToUE2D(LaneNetwork->GetLaneStart(
        LaneNetwork->Lanes[LaneConnection.DestinationLaneID], 
        LaneConnection.DestinationOffset));
    FVector2D Direction = (Destination - Source).GetSafeNormal();

    return Source + RoutePoint.GetOffset() * Direction;
  }
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
  T = FMath::Max(0.0f, FMath::Min((SegmentEnd - SegmentStart).Size(), T));

  return FRoutePoint(result.second, T);
}

TArray<FRoutePoint> FLaneNetworkRouteMap::GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) const {
  //UE_LOG(LogCarla, Display, TEXT("START"));
  TArray<FRoutePoint> NextRoutePoints;

  TQueue<TPair<FRoutePoint, float>> Queue;
  Queue.Enqueue(TPair<FRoutePoint, float>(RoutePoint, LookaheadDistance));

  TPair<FRoutePoint, float> QueueItem;
  while (Queue.Dequeue(QueueItem)) {
    const FRoutePoint& CurrentRoutePoint = QueueItem.Key;
    network_segment CurrentSegment = Segments[CurrentRoutePoint.GetID()];
   
    float Offset = CurrentRoutePoint.GetOffset();
    float Distance = QueueItem.Value;

    if (CurrentSegment.first) { // Segment is lane.
      const FLane& Lane = LaneNetwork->Lanes[CurrentSegment.second];
      
      FVector2D Start = ToUE2D(LaneNetwork->GetLaneStart(
          Lane, 
          LaneNetwork->GetLaneStartMinOffset(Lane)));
      FVector2D End = ToUE2D(LaneNetwork->GetLaneEnd(
          Lane, 
          LaneNetwork->GetLaneEndMinOffset(Lane)));
      FVector2D Direction = (End - Start).GetSafeNormal();
      
      //UE_LOG(LogCarla, Display, TEXT("Lane %f / %f, %f"), Offset, (Start - End).Size(), Distance);
      //UE_LOG(LogCarla, Display, TEXT("Lane Min Start, End = %f %f"),
      //    LaneNetwork->GetLaneStartMinOffset(Lane),
      //    LaneNetwork->GetLaneEndMinOffset(Lane));
      if (Offset + Distance <= (End - Start).Size()) {
        //UE_LOG(LogCarla, Display, TEXT("Lane Point %f / %f"), Offset + Distance, (Start - End).Size());
        NextRoutePoints.Emplace(CurrentRoutePoint.GetID(), Offset + Distance);
      }

      for (long long OutgoingLaneConnectionID : LaneNetwork->GetOutgoingLaneConnectionIDs(Lane)) {
        const FLaneConnection& OutgoingLaneConnection = LaneNetwork->LaneConnections[OutgoingLaneConnectionID];
       
        float OutgoingOffset = (End - Start).Size() - ToUE(
            OutgoingLaneConnection.SourceOffset - LaneNetwork->GetLaneEndMinOffset(Lane));

        //UE_LOG(LogCarla, Display, TEXT("Lane Connection %f %f %f"), Offset, OutgoingOffset, Distance);
        //UE_LOG(LogCarla, Display, TEXT("Lane Connection End %f"), OutgoingLaneConnection.SourceOffset);

        if (OutgoingOffset >= Offset && OutgoingOffset - Offset < Distance) {
          //UE_LOG(LogCarla, Display, TEXT("Enqueue"));
          Queue.Enqueue(TPair<FRoutePoint, float>(
              FRoutePoint(LaneConnectionIDToSegmentIDMap[OutgoingLaneConnection.ID], 0.0f),
              Distance - (OutgoingOffset - Offset)));
        }
      }
    } else { // Segment is lane connection.
      const FLaneConnection& LaneConnection = LaneNetwork->LaneConnections[CurrentSegment.second];
      
      FVector2D Source = ToUE2D(LaneNetwork->GetLaneEnd(
          LaneNetwork->Lanes[LaneConnection.SourceLaneID],
          LaneConnection.SourceOffset));
      FVector2D Destination = ToUE2D(LaneNetwork->GetLaneStart(
          LaneNetwork->Lanes[LaneConnection.DestinationLaneID],
          LaneConnection.DestinationOffset));
      
      //UE_LOG(LogCarla, Display, TEXT("Lane Connection %f / %f, %f"), Offset, (Destination - Source).Size(), Distance);
      
      if (Offset + Distance <= (Destination - Source).Size()) {
        //UE_LOG(LogCarla, Display, TEXT("Lane Connection Point %f %f / %f"), Offset + Distance, (Destination - Source).Size());
        NextRoutePoints.Emplace(CurrentRoutePoint.GetID(), Offset + Distance);
      } else {
        //UE_LOG(LogCarla, Display, TEXT("Enqueue"));
        Queue.Enqueue(TPair<FRoutePoint, float>(
            FRoutePoint(LaneIDToSegmentIDMap[LaneConnection.DestinationLaneID], 0.0f),
            Distance - ((Destination - Source).Size() - Offset)));
      }
    }
  }

  //UE_LOG(LogCarla, Display, TEXT("END"));

  return NextRoutePoints;
}
