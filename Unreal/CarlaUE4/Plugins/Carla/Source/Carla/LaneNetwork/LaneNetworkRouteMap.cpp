#include "LaneNetworkRouteMap.h"
#include <vector>
  
FLaneNetworkRouteMap::FLaneNetworkRouteMap(const FLaneNetwork* LaneNetwork)
    : LaneNetwork(LaneNetwork) { 

  std::vector<rt_value> IndexEntries; 
  
  for (const TPair<long long, FLane>& LaneEntry : LaneNetwork->Lanes) {
    FVector2D Start = LaneNetwork->GetLaneStart(
        LaneEntry.Value, 
        LaneNetwork->GetLaneStartMinOffset(LaneEntry.Value));
    FVector2D End = LaneNetwork->GetLaneEnd(
        LaneEntry.Value, 
        LaneNetwork->GetLaneEndMinOffset(LaneEntry.Value));
    
    IndexEntries.emplace_back(
        rt_segment(rt_point(Start.X, Start.Y), rt_point(End.X, End.Y)),
        Segments.Emplace(true, LaneEntry.Key));
  }

  for (const TPair<long long, FLaneConnection>& LaneConnectionEntry : LaneNetwork->LaneConnections) {
    FVector2D Source = LaneNetwork->GetLaneStart(
        LaneNetwork->Lanes[LaneConnectionEntry.Value.SourceLaneID], 
        LaneConnectionEntry.Value.SourceOffset);
    FVector2D Destination = LaneNetwork->GetLaneStart(
        LaneNetwork->Lanes[LaneConnectionEntry.Value.DestinationLaneID], 
        LaneConnectionEntry.Value.DestinationOffset);

    IndexEntries.emplace_back(
        rt_segment(rt_point(Source.X, Source.Y), rt_point(Destination.X, Destination.Y)),
        Segments.Emplace(false, LaneConnectionEntry.Key));
  }

  LanesIndex = rt_tree(IndexEntries);
}

FRoutePoint FLaneNetworkRouteMap::GetNearestRoutePoint(const FVector2D& Position) {
  std::vector<rt_value> results;
  LanesIndex.query(boost::geometry::index::nearest(rt_point(Position.X, Position.Y), 1), std::back_inserter(results));
  
  FVector2D SegmentStart(
      boost::geometry::get<0, 0>(results[0].first),
      boost::geometry::get<0, 1>(results[0].first));
  FVector2D SegmentEnd(
      boost::geometry::get<1, 0>(results[0].first),
      boost::geometry::get<1, 1>(results[0].first));

  float T = FVector2D::DotProduct(
      Position - SegmentStart,
      (SegmentEnd - SegmentStart).GetSafeNormal());
  T = FMath::Min(0.0f, FMath::Max((SegmentEnd - SegmentStart).Size(), T));

  return FRoutePoint(0, FVector2D(0, 0));
}

TArray<FRoutePoint> FLaneNetworkRouteMap::GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) {
  return TArray<FRoutePoint>();
}
