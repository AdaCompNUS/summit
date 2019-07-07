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
    
    IndexEntries.emplace_back(
        rt_segment(rt_point(Start.X, Start.Y), rt_point(End.X, End.Y)),
        Segments.Emplace(true, LaneEntry.Key));
  }

  for (const TPair<long long, FLaneConnection>& LaneConnectionEntry : LaneNetwork->LaneConnections) {
    FVector2D Source = ToUE2D(LaneNetwork->GetLaneStart(
        LaneNetwork->Lanes[LaneConnectionEntry.Value.SourceLaneID], 
        LaneConnectionEntry.Value.SourceOffset));
    FVector2D Destination = ToUE2D(LaneNetwork->GetLaneStart(
        LaneNetwork->Lanes[LaneConnectionEntry.Value.DestinationLaneID], 
        LaneConnectionEntry.Value.DestinationOffset));

    IndexEntries.emplace_back(
        rt_segment(rt_point(Source.X, Source.Y), rt_point(Destination.X, Destination.Y)),
        Segments.Emplace(false, LaneConnectionEntry.Key));
  }

  LanesIndex = rt_tree(IndexEntries);
}

FRoutePoint FLaneNetworkRouteMap::GetNearestRoutePoint(const FVector2D& Position) const {
  std::vector<rt_value> results;
  LanesIndex.query(boost::geometry::index::nearest(rt_point(Position.X, Position.Y), 1), std::back_inserter(results));
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
  return TArray<FRoutePoint>();
}
