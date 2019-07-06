#pragma once

#include "RouteMap/RouteMap.h"

class FLaneNetworkRouteMap : public FRouteMap {
public:

  FLaneNetworkRouteMap() { }

  FLaneNetworkRouteMap(const FLaneNetwork* LaneNetwork);

  FRoutePoint GetNearestRoutePoint(const FVector2D& Position) override;

  TArray<FRoutePoint> GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) override;

private:

  const FLaneNetwork* LaneNetwork;
};
