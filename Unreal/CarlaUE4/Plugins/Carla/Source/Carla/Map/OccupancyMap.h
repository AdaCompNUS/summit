#pragma once

#include "OccupancyTriangle.h"
#include "OccupancyGrid.h"
#include "OccupancyArea.h"
#include "aabb/AABB.h"

class FOccupancyMap {

public:

  FOccupancyMap() : Area(0) { }
  FOccupancyMap(const TArray<FOccupancyTriangle>& OccupancyTriangles);

  // TODO Optimize using AABB trees.
  FVector2D RandPoint() const;

  // TODO Optimize using AABB trees.
  FOccupancyArea GetOccupancyArea(const FBox2D& Bounds, float Resolution, int OffroadPolygonEdgeInterval) const;

private:

  TArray<FOccupancyTriangle> OccupancyTriangles;
  float Area;
};
