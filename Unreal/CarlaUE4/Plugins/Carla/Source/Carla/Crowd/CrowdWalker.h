#pragma once

#include "RouteMap/RouteMap.h"

// TODO Change to actor.
class FCrowdWalker {

public:
  
  FCrowdWalker(const FRouteMap* InRouteMap, AActor* InActor, float InMaxSpeed)
      : RouteMap(InRouteMap), Actor(InActor), MaxSpeed(InMaxSpeed) { }
  
  FVector GetPosition() const;

  FVector2D GetPosition2D() const;

  boost::optional<FVector2D> GetPreferredVelocity();

  void SetVelocity(const FVector2D& Velocity);

private:
 
  const FRouteMap* RouteMap;
  AActor* Actor;
  float MaxSpeed;

  TArray<FRoutePoint> PathRoutePoints;

  void AddClosestRoutePointToPath();

  bool ExtendPath();
};
