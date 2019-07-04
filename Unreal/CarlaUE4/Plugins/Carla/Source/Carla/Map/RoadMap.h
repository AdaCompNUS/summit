#pragma once

#include "RoadTriangle.h"
#include "OccupancyGrid.h"

class FRoadMap {

public:

  FRoadMap() : Area(0), Resolution(0), OffroadPolygonEdgeInterval(0) { }
  FRoadMap(const TArray<FRoadTriangle>& RoadTriangles, float Resolution, int OffroadPolygonEdgeInterval);
  FRoadMap(const FBox& Bounds, const TArray<FRoadTriangle>& RoadTriangles, float Resolution, int OffroadPolygonEdgeInterval);

  FBox GetBounds() const { return Bounds; }

  float GetArea() const { return Area; }

  float GetResolution() const { return Resolution; }

  const TArray<TArray<FVector2D>>& GetOffroadPolygons() const { return OffroadPolygons; }

  FVector RandPoint() const;
  
  void RenderBitmap(const FString& FileName) const;

private:

  FBox Bounds;
  TArray<FRoadTriangle> RoadTriangles;
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
