#pragma once

#include "RoadTriangle.h"

class FRoadMap {

public:

  FRoadMap() : Area(0) { }
  FRoadMap(const TArray<FRoadTriangle>& RoadTriangles);

  FVector RandPoint() const;
  
  void RenderMonteCarloBitmap(const FString& FileName, float Resolution, int Trials) const;

private:

  TArray<FRoadTriangle> RoadTriangles;
  float Area;

};
