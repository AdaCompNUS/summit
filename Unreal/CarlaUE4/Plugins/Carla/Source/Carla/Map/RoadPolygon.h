#pragma once

class FRoadPolygon {

public:

  FRoadPolygon() : Area(0), MaxZ(0) { }

  FRoadPolygon(const TArray<FVector>& Vertices);

  double GetArea() const { return Area; }

  double GetMaxZ() const { return MaxZ; }

  bool InPolygon(const FVector2D& Point) const;

  FVector2D RandPoint() const;

private:

  TArray<FVector2D> Vertices;
  FBox Bounds;
  float Area;
  float MaxZ;
};
