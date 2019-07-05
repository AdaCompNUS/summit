#include "OccupancyTriangle.h"

double FOccupancyTriangle::GetArea() const {
  return (FVector(V1 - V0, 0) ^ FVector(V2 - V0, 0)).Size() * 0.5f;
}

FBox2D FOccupancyTriangle::GetBounds() const {
  FBox2D Bounds;
  Bounds.Min.X = FMath::Min3(V0.X, V1.X, V2.X);
  Bounds.Min.Y = FMath::Min3(V0.Y, V1.Y, V2.Y);
  Bounds.Max.X = FMath::Max3(V0.X, V1.X, V2.X);
  Bounds.Max.Y = FMath::Max3(V0.Y, V1.Y, V2.Y);
  return Bounds;
}

FVector2D FOccupancyTriangle::RandPoint() const {
  // https://math.stackexchange.com/questions/18686/uniform-random-point-in-triangle 
  float SqrtR1 = FMath::Sqrt(FMath::FRandRange(0, 1));
  float R2 = FMath::FRandRange(0, 1);
  return (1 - SqrtR1) * V0 + (SqrtR1 * (1 - R2)) * V1 + R2 * SqrtR1 * V2;
}
