#pragma once

class FPolygon {
public:
  FPolygon(const TArray<FVector>& Vertices);

  double GetArea() const { return Area; }

  double GetMaxZ() const { return MaxZ; }

  bool InPolygon(const FVector2D& Point) const;

  FVector2D RandPoint() const;

private:
  TArray<FVector2D> Vertices;
  FBox BoundingBox;
  float Area = 0;
  float MaxZ;
};
