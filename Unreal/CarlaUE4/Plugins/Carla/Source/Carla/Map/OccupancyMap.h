#pragma once

#include "OccupancyTriangle.h"
#include "OccupancyGrid.h"

class FOccupancyMap {

public:

  FOccupancyMap() : Area(0), Resolution(0), OffroadPolygonEdgeInterval(0) { }
  FOccupancyMap(const TArray<FOccupancyTriangle>& OccupancyTriangles, float Resolution, int OffroadPolygonEdgeInterval);
  FOccupancyMap(const FBox& Bounds, const TArray<FOccupancyTriangle>& OccupancyTriangles, float Resolution, int OffroadPolygonEdgeInterval);

  FBox GetBounds() const { return Bounds; }

  float GetArea() const { return Area; }

  float GetResolution() const { return Resolution; }

  const TArray<TArray<FVector2D>>& GetOffroadPolygons() const { return OffroadPolygons; }

  FVector RandPoint() const;
  
  void RenderBitmap(const FString& FileName) const;

private:

  FBox Bounds;
  TArray<FOccupancyTriangle> OccupancyTriangles;
  float Area;
  
  FOccupancyGrid OccupancyGrid;
  float Resolution;

  TArray<TArray<FVector2D>> OffroadPolygons;
  int OffroadPolygonEdgeInterval;

  void InitOccupancyGrid();
  void InitOffroadPolygons();

  FIntPoint Point2DToPixel(const FVector2D& Point) const;
  FVector2D PixelToPoint2D(const FIntPoint& Pixel) const;

};
