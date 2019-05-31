#include "CrowdWalker.h"
#include "Carla/Walker/WalkerController.h"
#include "Carla/Walker/WalkerControl.h"
#include <carla/geom/Location.h>
#include <carla/road/element/Waypoint.h>
#include <vector>

carla::geom::Vector2D CrowdWalker::GetPreferredVelocity() {
  // Use constructor for units conversion.
  carla::geom::Location Location(Actor->GetTransform().GetTranslation());
  
  std::vector<carla::road::element::Waypoint> NextWaypoints = WaypointMap->GetNext(
      WaypointMap->GetClosestWaypointOnRoad(Location).get(), 
      1.0);

  if (NextWaypoints.size() == 0) return carla::geom::Vector2D(0, 0);
  carla::geom::Location NextWaypointLocation = WaypointMap->ComputeTransform(NextWaypoints[FMath::RandRange(0, NextWaypoints.size() - 1)]).location;
  carla::geom::Location Offset = NextWaypointLocation - Location;
  return (carla::geom::Vector2D(Offset.x, Offset.y)).MakeUnitVector();
}

void CrowdWalker::SetVelocity(const carla::geom::Vector2D& Velocity) {
  auto Controller = Cast<AWalkerController>(Cast<APawn>(Actor)->GetController());
  FWalkerControl Control;
  Control.Direction = FVector(Velocity.x, Velocity.y, 0);
  Control.Speed = 500.0f;
  Control.Jump = false;
  Controller->ApplyWalkerControl(Control);
}
