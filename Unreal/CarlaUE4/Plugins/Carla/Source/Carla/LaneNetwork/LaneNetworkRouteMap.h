#pragma once

#include "RouteMap/RouteMap.h"

class FLaneNetworkRouteMap : public FRouteMap {
public:

  FRoutePoint GetNearestRoutePoint(const FVector2D& Position) override;

  TArray<FRoutePoint> GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) override;
};
