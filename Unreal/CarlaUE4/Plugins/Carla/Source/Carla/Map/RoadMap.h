#pragma once

#include "RoadTriangle.h"
#include "OccupancyGrid.h"

class FRoadMap {

public:

  FRoadMap() : Area(0), Resolution(0), OffroadPolygonEdgeInterval(0) { }
  FRoadMap(const TArray<FRoadTriangle>& RoadTriangles, float Resolution, int OffroadPolygonEdgeInterval);

  FBox GetBounds() const { return Bounds; }

  float GetArea() const { return Area; }

  float GetResolution() const { return Resolution; }

  int GetOffroadPolygonEdgeInterval() const { return OffroadPolygonEdgeInterval; }

  FVector RandPoint() const;

  TArray<FVector2D> RandPath(double Radius) const;
  
  void RenderMonteCarloBitmap(const FString& FileName, int Trials) const;

private:

  TArray<FRoadTriangle> RoadTriangles;
  FBox Bounds;
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
