#pragma once

#include <carla/geom/Vector2D.h>
#include <carla/geom/Location.h>
#include <carla/road/element/Waypoint.h>

class CrowdWalker {

public:
  
  CrowdWalker(carla::road::Map* InWaypointMap, AActor* InActor, float InMaxSpeed)
      : WaypointMap(InWaypointMap), Actor(InActor), MaxSpeed(InMaxSpeed) { }
  
  carla::geom::Location GetLocation() const;

  carla::geom::Vector2D GetLocation2D() const;

  boost::optional<carla::geom::Vector2D> GetPreferredVelocity();

  void SetVelocity(const carla::geom::Vector2D& Velocity);

private:
  
  carla::road::Map* WaypointMap;
  AActor* Actor;
  float MaxSpeed;
  TArray<carla::road::element::Waypoint> PathWaypoints;
  TArray<carla::geom::Location> PathLocations;

  void AddClosestWaypointToPath();
  bool ExtendPath();
};
