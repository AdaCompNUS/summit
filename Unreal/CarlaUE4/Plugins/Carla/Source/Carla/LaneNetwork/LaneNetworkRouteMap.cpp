#include "LaneNetworkRouteMap.h"

FRoutePoint FLaneNetworkRouteMap::GetNearestRoutePoint(const FVector2D& Position) {
  return FRoutePoint(0, FVector2D(0, 0));
}

TArray<FRoutePoint> FLaneNetworkRouteMap::GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) {
  return TArray<FRoutePoint>();
}
