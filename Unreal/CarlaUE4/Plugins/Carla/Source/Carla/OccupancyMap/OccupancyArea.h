#pragma once

class FOccupancyArea {

public:
  FBox2D Bounds;
  float Resolution;
  int OffroadPolygonEdgeInterval;

  FOccupancyGrid OccupancyGrid;
  TArray<TArray<FVector2D>> OffroadPolygons;

  FOccupancyArea() { }

  FIntPoint Point2DToPixel(const FVector2D& Point) const {
    return FIntPoint(
      FMath::FloorToInt((Point.Y - Bounds.Min.Y) / Resolution),
      FMath::FloorToInt(-(Point.X - Bounds.Max.X) / Resolution));
  };

  FVector2D PixelToPoint2D(const FIntPoint& Pixel) const {
    return FVector2D(
        Bounds.Max.X - (Pixel.Y + 0.5) * Resolution,
        Bounds.Min.Y + (Pixel.X + 0.5) * Resolution);
  };

};
