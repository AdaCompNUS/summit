#pragma once

class FOccupancyArea {

public:
  FBox2D Bounds;
  float Resolution;
  int OffroadPolygonEdgeInterval;

  FOccupancyGrid OccupancyGrid;
  TArray<TArray<FVector2D>> OffroadPolygons;

  FOccupancyArea() { }

  FIntPoint Point2DToPixel(const FVector2D& Point) const;

  FVector2D PixelToPoint2D(const FIntPoint& Pixel) const;

  FVector2D RandPoint() const;

};
