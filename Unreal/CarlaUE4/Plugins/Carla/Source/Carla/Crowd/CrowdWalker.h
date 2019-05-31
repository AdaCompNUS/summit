#pragma once
#include <carla/geom/Vector2D.h>

class CrowdWalker {

public:
  
  CrowdWalker(carla::road::Map* InWaypointMap, AActor* InActor)
      : WaypointMap(InWaypointMap), Actor(InActor) { }

  carla::geom::Vector2D GetLocation() const;

  carla::geom::Vector2D GetPreferredVelocity();

  void SetVelocity(const carla::geom::Vector2D& Velocity);

private:
  
  carla::road::Map* WaypointMap;
  AActor* Actor;

};
