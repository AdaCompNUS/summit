#pragma once

#include "RoutePoint.h"

class FRouteMap {
public:
  
  virtual FVector2D GetPosition(const FRoutePoint& RoutePoint) const = 0;
  
  virtual FRoutePoint GetNearestRoutePoint(const FVector2D& Position) const = 0; 

  virtual TArray<FRoutePoint> GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) const = 0;
};
