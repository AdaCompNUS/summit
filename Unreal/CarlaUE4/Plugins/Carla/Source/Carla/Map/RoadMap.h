#pragma once

#include "RoadTriangle.h"
#include "OccupancyGrid.h"

class FRoadMap {

public:

  FRoadMap() : Area(0), Resolution(0) { }
  FRoadMap(const TArray<FRoadTriangle>& RoadTriangles, float Resolution);

  float GetArea() const { return Area; }

  float GetResolution() const { return Resolution; }

  FVector RandPoint() const;

  TArray<FVector2D> RandPath(double Radius) const;
  
  void RenderMonteCarloBitmap(const FString& FileName, int Trials) const;

private:

  TArray<FRoadTriangle> RoadTriangles;
  float Area;
  FBox Bounds;
  float Resolution;
  FOccupancyGrid OccupancyGrid;

  FIntPoint Point2DToPixel(const FVector2D& Point) const;
  FVector2D PixelToPoint2D(const FIntPoint& Pixel) const;

};
