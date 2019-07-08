#include "CrowdWalker.h"
#include "Carla/Walker/WalkerController.h"
#include "Carla/Walker/WalkerControl.h"
#include "DrawDebugHelpers.h"
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
  while (PathRoutePoints.Num() < 20 && ExtendPath());
  if (PathRoutePoints.Num() < 20) return boost::none;

  // Cut nearby route points.
  int CutIndex = 0;
  for (int I = 0; I < PathRoutePoints.Num() / 2; I++) {
    if ((Position - RouteMap->GetPosition(PathRoutePoints[I])).Size() < 100.0f) {
      CutIndex = I + 1;
    }
  }

  // Shorten path.
  TArray<FRoutePoint> NewPathRoutePoints;
  for (int J = CutIndex; J < PathRoutePoints.Num(); J++) {
    NewPathRoutePoints.Add(PathRoutePoints[J]);
  }
  PathRoutePoints = NewPathRoutePoints;
  FVector2D TargetPosition = RouteMap->GetPosition(PathRoutePoints[0]);
    
  DrawDebugLine(
    Actor->GetWorld(), 
    FVector(Position, 10),
    FVector(RouteMap->GetPosition(PathRoutePoints[0]), 10),
    FColor(255, 0, 0), 
    false, 2.0, 1, 
    5.0f
  );

  for (int J = 0; J < PathRoutePoints.Num() - 1; J++) {
    DrawDebugLine(
      Actor->GetWorld(), 
      FVector(RouteMap->GetPosition(PathRoutePoints[J]), 10),
      FVector(RouteMap->GetPosition(PathRoutePoints[J + 1]), 10),
      FColor(255, 0, 0), 
      false, 2.0, 1, 
      5.0f
    );
  }

  return MaxSpeed * (TargetPosition - Position).GetSafeNormal();
}

void FCrowdWalker::SetVelocity(const FVector2D& Velocity) {
  auto Controller = Cast<AWalkerController>(Cast<APawn>(Actor)->GetController());
  FWalkerControl Control;
  Control.Direction = FVector(Velocity, 0);
  Control.Speed = 1.0f;
  Control.Jump = false;
  Controller->ApplyWalkerControl(Control);
}
