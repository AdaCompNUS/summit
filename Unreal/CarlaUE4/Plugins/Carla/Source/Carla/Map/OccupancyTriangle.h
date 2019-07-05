#pragma once

class FOccupancyTriangle {

public:

  FVector2D V0;
  FVector2D V1;
  FVector2D V2;

  FOccupancyTriangle(const FVector2D& V0, const FVector2D& V1, const FVector2D& V2)
    : V0(V0), V1(V1), V2(V2) { }

  double GetArea() const;

  FBox2D GetBounds() const;

  FVector2D RandPoint() const;
};
