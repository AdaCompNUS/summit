#pragma once

#include "RoadTriangle.h"

class FRoadMap {

public:

  FRoadMap() : Area(0) { }
  FRoadMap(const TArray<FRoadTriangle>& RoadTriangles);

  bool RenderBitmap(const FString& FileName) const;

private:

  TArray<FRoadTriangle> RoadTriangles;
  float Area;

};
