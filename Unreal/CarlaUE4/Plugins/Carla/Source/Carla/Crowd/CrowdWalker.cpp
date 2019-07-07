#include "CrowdWalker.h"
#include "Carla/Walker/WalkerController.h"
#include "Carla/Walker/WalkerControl.h"
#include <vector>

FVector FCrowdWalker::GetPosition() const {
  return Actor->GetTransform().GetTranslation();
}

FVector2D FCrowdWalker::GetPosition2D() const {
  return FVector2D(GetPosition());
}

void FCrowdWalker::AddClosestRoutePointToPath() {
  PathRoutePoints.Emplace(RouteMap->GetNearestRoutePoint(GetPosition2D()));
}

bool FCrowdWalker::ExtendPath() {
  TArray<FRoutePoint> NextRoutePoints = RouteMap->GetNextRoutePoints(PathRoutePoints.Last(), 100.0f);

  if (NextRoutePoints.Num() == 0) return false;

  PathRoutePoints.Emplace(NextRoutePoints[FMath::RandRange(0, NextRoutePoints.Num() - 1)]);

  return true;
}

boost::optional<FVector2D> FCrowdWalker::GetPreferredVelocity() {
  FVector2D Position = GetPosition2D();

  // Extend local path.
  if (PathRoutePoints.Num() == 0) AddClosestRoutePointToPath();
  while (PathRoutePoints.Num() < 50 && ExtendPath());
  if (PathRoutePoints.Num() < 50) return boost::none;

  // Calculate nearest location.
  int I = 0;
  while (I < PathRoutePoints.Num() - 1) {
    if ((Position - RouteMap->GetPosition(PathRoutePoints[I])).SizeSquared() < (Position - RouteMap->GetPosition(PathRoutePoints[I + 1])).SizeSquared()) break;
    I++;
  }
  FVector2D TargetPosition = RouteMap->GetPosition(PathRoutePoints[I + 3]);

  // Shorten path.
  TArray<FRoutePoint> NewPathRoutePoints;
  for (int J = I; J < PathRoutePoints.Num(); J++) {
    NewPathRoutePoints.Add(PathRoutePoints[J]);
  }
  PathRoutePoints = NewPathRoutePoints;

  return (TargetPosition - Position).GetSafeNormal();
}

void FCrowdWalker::SetVelocity(const FVector2D& Velocity) {
  auto Controller = Cast<AWalkerController>(Cast<APawn>(Actor)->GetController());
  FWalkerControl Control;
  Control.Direction = FVector(Velocity, 0);
  Control.Speed = 300.0f;
  Control.Jump = false;
  Controller->ApplyWalkerControl(Control);
}
