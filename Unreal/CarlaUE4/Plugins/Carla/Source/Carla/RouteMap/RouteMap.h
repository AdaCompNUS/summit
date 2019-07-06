#pragma once

#include "RoutePoint.h"

class FRouteMap {
public:

  // Do not mark these as const since implementing functions may mutate object, e.g. lazy loading
  // from some server, caching, etc.
  
  virtual FRoutePoint GetNearestRoutePoint(const FVector2D& Position) = 0; 

  virtual TArray<FRoutePoint> GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) = 0;
};
