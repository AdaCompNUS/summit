#include "CrowdWalker.h"
#include "Carla/Walker/WalkerController.h"
#include "Carla/Walker/WalkerControl.h"
#include <carla/geom/Location.h>
#include <carla/road/element/Waypoint.h>
#include <vector>

FVector2D CrowdWalker::GetPreferredVelocity() {
  carla::geom::Location Location;
  Location.x = Actor->GetTransform().GetTranslation().X;
  Location.y = Actor->GetTransform().GetTranslation().Y;
  Location.z = Actor->GetTransform().GetTranslation().Z;
  
  carla::road::element::Waypoint Waypoint = WaypointMap->GetWaypoint(Location).get();
  
  std::vector<carla::road::element::Waypoint> NextWaypoints = WaypointMap->GetNext(Waypoint, 1.0);

  if (NextWaypoints.size() == 0) return FVector2D(0, 0);

  carla::road::element::Waypoint NextWaypoint = NextWaypoints[FMath::RandRange(0, NextWaypoints.size() - 1)];
  carla::geom::Location NextWaypointLocation = WaypointMap->ComputeTransform(NextWaypoint).location;

  carla::geom::Location Offset = NextWaypointLocation - Location;
  FVector Velocity = Offset.MakeUnitVector().ToFVector();

  return FVector2D(Velocity.X, Velocity.Y);
}

void CrowdWalker::SetVelocity(const FVector2D& Velocity) {
  auto Controller = Cast<AWalkerController>(Cast<APawn>(Actor)->GetController());
  FWalkerControl Control;
  Control.Direction = FVector(Velocity.X, Velocity.Y, 0);
  Control.Speed = 1.0f;
  Control.Jump = false;
  Controller->ApplyWalkerControl(Control);
}
