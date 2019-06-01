#include "CrowdWalker.h"
#include "Carla/Walker/WalkerController.h"
#include "Carla/Walker/WalkerControl.h"
#include <carla/geom/Location.h>
#include <carla/road/element/Waypoint.h>
#include <vector>

carla::geom::Location CrowdWalker::GetLocation() const {
  return carla::geom::Location(Actor->GetTransform().GetTranslation());
}

carla::geom::Vector2D CrowdWalker::GetLocation2D() const {
  carla::geom::Location Location = GetLocation();
  return carla::geom::Vector2D(Location.x, Location.y);
}
  
void CrowdWalker::AddClosestWaypointToPath() {
  PathWaypoints.Emplace(*(WaypointMap->GetClosestWaypointOnRoad(GetLocation())));
  PathLocations.Emplace(WaypointMap->ComputeTransform(PathWaypoints.Last()).location);
}

bool CrowdWalker::ExtendPath() {
  std::vector<carla::road::element::Waypoint> NextWaypoints = 
    WaypointMap->GetNext(PathWaypoints.Last(), 0.1);

  if (NextWaypoints.size() == 0) return false;

  PathWaypoints.Emplace(NextWaypoints[FMath::RandRange(0, NextWaypoints.size() - 1)]);
  PathLocations.Emplace(WaypointMap->ComputeTransform(PathWaypoints.Last()).location);

  return true;
}

boost::optional<carla::geom::Vector2D> CrowdWalker::GetPreferredVelocity() {
  carla::geom::Location Location = GetLocation();

  // Extend local path.
  if (PathWaypoints.Num() == 0) AddClosestWaypointToPath();
  while (PathWaypoints.Num() < 500 && ExtendPath());
  if (PathWaypoints.Num() < 500) return boost::none;

  // Calculate nearest location.
  int I = 0;
  while (I < PathWaypoints.Num() - 1) {
    if ((Location - PathLocations[I]).SquaredLength() < (Location - PathLocations[I + 1]).SquaredLength()) break;
    I++;
  }
  carla::geom::Location TargetLocation = PathLocations[I + 20];

  // Shorten path.
  TArray<carla::road::element::Waypoint> NewPathWaypoints;
  TArray<carla::geom::Location> NewPathLocations;
  for (int J = I; J < PathWaypoints.Num(); J++) {
    NewPathWaypoints.Add(PathWaypoints[J]);
    NewPathLocations.Add(PathLocations[J]);
  }
  PathWaypoints = NewPathWaypoints;
  PathLocations = NewPathLocations;

  // Calculate velocity.
  carla::geom::Vector3D Offset = TargetLocation - Location;
  return MaxSpeed * carla::geom::Vector2D(Offset.x, Offset.y).MakeUnitVector();
}

void CrowdWalker::SetVelocity(const carla::geom::Vector2D& Velocity) {
  auto Controller = Cast<AWalkerController>(Cast<APawn>(Actor)->GetController());
  FWalkerControl Control;
  Control.Direction = FVector(Velocity.x, Velocity.y, 0);
  Control.Speed = 100.0f;
  Control.Jump = false;
  Controller->ApplyWalkerControl(Control);
}
