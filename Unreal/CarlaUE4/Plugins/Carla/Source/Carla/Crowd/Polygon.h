#pragma once

class FPolygon {
public:
  FPolygon(const TArray<FVector2D>& Vertices);

  double GetArea() const;

  bool InPolygon(const FVector2D& Point) const;

  FVector2D RandomPoint() const;

private:
  TArray<FVector2D> Vertices;
  float Area = 0;
  float MinX;
  float MaxX;
  float MinY;
  float MaxY;
};
