#include "OccupancyArea.h"

FIntPoint FOccupancyArea::Point2DToPixel(const FVector2D& Point) const {
  return FIntPoint(
    FMath::FloorToInt((Point.Y - Bounds.Min.Y) / Resolution),
    FMath::FloorToInt(-(Point.X - Bounds.Max.X) / Resolution));
};

FVector2D FOccupancyArea::PixelToPoint2D(const FIntPoint& Pixel) const {
  return FVector2D(
      Bounds.Max.X - (Pixel.Y + 0.5) * Resolution,
      Bounds.Min.Y + (Pixel.X + 0.5) * Resolution);
};

FVector2D FOccupancyArea::RandPoint() const {
  FIntPoint Pixel;
  do {
    Pixel.X = FMath::RandRange(0, OccupancyGrid.GetWidth() - 1);
    Pixel.Y = FMath::RandRange(0, OccupancyGrid.GetHeight() - 1);
  } while (!OccupancyGrid(Pixel.X, Pixel.Y));

  return PixelToPoint2D(Pixel);
}
