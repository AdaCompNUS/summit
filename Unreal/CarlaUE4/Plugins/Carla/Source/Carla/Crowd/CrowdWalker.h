#pragma once

class CrowdWalker {

public:
  
  CrowdWalker(carla::road::Map* InWaypointMap, AActor* InActor)
      : WaypointMap(InWaypointMap), Actor(InActor) { }

  FVector2D GetPreferredVelocity();

  void SetVelocity(const FVector2D& Velocity);

private:
  
  carla::road::Map* WaypointMap;
  AActor* Actor;

};
