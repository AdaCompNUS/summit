#pragma once

class FOccupancyArea {

public:
  FBox2D Bounds;
  float Resolution;
  int OffroadPolygonEdgeInterval;

  FOccupancyGrid OccupancyGrid;
  TArray<TArray<FVector2D>> OffroadPolygons;
  TArray<TArray<FIntPoint>> OffroadPolygons_Pixel;

  FOccupancyArea() { }
};
