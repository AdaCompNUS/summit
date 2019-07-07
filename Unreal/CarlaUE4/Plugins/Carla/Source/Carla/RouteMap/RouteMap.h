#pragma once

#include "RoutePoint.h"

class FRouteMap {
public:
  
  virtual FRoutePoint GetNearestRoutePoint(const FVector2D& Position) const = 0; 

  virtual TArray<FRoutePoint> GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) const = 0;
};
