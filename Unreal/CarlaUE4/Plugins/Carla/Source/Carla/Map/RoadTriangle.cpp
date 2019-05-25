#include "RoadTriangle.h"

double FRoadTriangle::GetArea() const {
  return ((V1 - V0) ^ (V2 - V0)).Size() * 0.5f;
}

FBox FRoadTriangle::GetBounds() const {
  FBox Bounds;
  Bounds.Min.X = FMath::Min3(V0.X, V1.X, V2.X);
  Bounds.Min.Y = FMath::Min3(V0.Y, V1.Y, V2.Y);
  Bounds.Min.Z = FMath::Min3(V0.Z, V1.Z, V2.Z);
  Bounds.Max.X = FMath::Max3(V0.X, V1.X, V2.X);
  Bounds.Max.Y = FMath::Max3(V0.Y, V1.Y, V2.Y);
  Bounds.Max.Z = FMath::Max3(V0.Z, V1.Z, V2.Z);
  return Bounds;
}

bool FRoadTriangle::InPolygon(const FVector2D& Point) const {
  return false;
}

FVector2D FRoadTriangle::RandPoint() const {

  // https://math.stackexchange.com/questions/18686/uniform-random-point-in-triangle 
  float SqrtR1 = FMath::Sqrt(FMath::FRandRange(0, 1));
  float R2 = FMath::FRandRange(0, 1);
  FVector Point = (1 - SqrtR1) * V0 + (SqrtR1 * (1 - R2)) * V1 + R2 * SqrtR1 * V2;

  // Project to 2D.
  return FVector2D(Point.X, Point.Y);
}
