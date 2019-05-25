#pragma once

class FRoadTriangle {

public:

  FVector V0;
  FVector V1;
  FVector V2;

  FRoadTriangle(const FVector& V0, const FVector& V1, const FVector& V2)
    : V0(V0), V1(V1), V2(V2) { }

  double GetArea() const;

  FBox GetBounds() const;

  FVector RandPoint() const;
};
