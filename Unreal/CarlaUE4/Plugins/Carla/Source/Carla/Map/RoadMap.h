#pragma once

#include "RoadPolygon.h"

class FRoadMap{

public:

  FRoadMap() : Area(0) { }

  FRoadMap(const TArray<FRoadPolygon>& Polygons);

  float GetArea() const {
    return Area;
  }

  FVector RandPoint() const;

  bool RenderBitmap(const FString& FileName) const;

private:

  TArray<FRoadPolygon> RoadPolygons;
  float Area;

};
