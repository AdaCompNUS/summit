#include "RoadMap.h"

FRoadMap::FRoadMap(const TArray<FRoadPolygon>& RoadPolygons)
  : RoadPolygons(RoadPolygons), Area(0) {
  for (const FRoadPolygon& RoadPolygon : RoadPolygons) {
    Area += RoadPolygon.GetArea();
  }
}

FVector FRoadMap::RandPoint() const {
  float V = FMath::FRandRange(0, Area);
  int I = 0;
  for (; V > 0 && I < RoadPolygons.Num(); I++) {
    V -= RoadPolygons[I].GetArea();
  }

  const FRoadPolygon& Polygon = RoadPolygons[I - 1];
  FVector2D Point2D = Polygon.RandPoint();
  FVector Point;
  Point.X = Point2D.X;
  Point.Y = Point2D.Y;
  Point.Z = Polygon.GetMaxZ();
  return Point;
}

bool FRoadMap::RenderBitmap(const FString& FileName) const {
  return true;
}
