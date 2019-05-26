#pragma once

#include "RoadTriangle.h"
#include "AABB.h"

class FRoadMap {

public:

  FRoadMap() : Area(0) { }
  FRoadMap(const TArray<FRoadTriangle>& RoadTriangles);

  const FBox& GetBounds() const { return Bounds; }

  float GetArea() const { return Area; }

  FVector RandPoint() const;
  
  void RenderMonteCarloBitmap(const FString& FileName, float Resolution, int Trials) const;

private:

  TArray<FRoadTriangle> RoadTriangles;
  FBox Bounds;
  float Area;

};
