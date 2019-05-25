#include "RoadMap.h"
#include "Runtime/Engine/Public/GeomTools.h"
#include "bitmap_image.hpp"

FRoadMap::FRoadMap(const TArray<FRoadTriangle>& RoadTriangles) 
  : RoadTriangles(RoadTriangles), Area(0) {
  for (const FRoadTriangle& RoadTriangle : RoadTriangles) {
    Area += RoadTriangle.GetArea();
  }
}

bool FRoadMap::RenderBitmap(const FString& FileName) const {
  FBox Bounds = RoadTriangles[0].GetBounds();
  UE_LOG(LogCarla, Display, TEXT("Pre : X = %f, %f, Y = %f, %f, Z = %f, %f"), 
      Bounds.Min.X, Bounds.Max.X, 
      Bounds.Min.Y, Bounds.Max.Y,
      Bounds.Min.Z, Bounds.Max.Z);

  for (int I = 1; I < RoadTriangles.Num(); I++) {
    const FRoadTriangle& RoadTriangle = RoadTriangles[I];
    Bounds.Min.X = FMath::Min(Bounds.Min.X, RoadTriangle.GetBounds().Min.X);
    Bounds.Min.Y = FMath::Min(Bounds.Min.Y, RoadTriangle.GetBounds().Min.Y);
    Bounds.Min.Z = FMath::Min(Bounds.Min.Z, RoadTriangle.GetBounds().Min.Z);
    Bounds.Max.X = FMath::Max(Bounds.Max.X, RoadTriangle.GetBounds().Max.X);
    Bounds.Max.Y = FMath::Max(Bounds.Max.Y, RoadTriangle.GetBounds().Max.Y);
    Bounds.Max.Z = FMath::Max(Bounds.Max.Z, RoadTriangle.GetBounds().Max.Z);
  }

  UE_LOG(LogCarla, Display, TEXT("Post: X = %f, %f, Y = %f, %f, Z = %f, %f"), 
      Bounds.Min.X, Bounds.Max.X, 
      Bounds.Min.Y, Bounds.Max.Y,
      Bounds.Min.Z, Bounds.Max.Z);
  
  UE_LOG(LogCarla, Display, TEXT("Num Tri = %d"), RoadTriangles.Num());
  
  bitmap_image image(Bounds.Max.X - Bounds.Min.X, Bounds.Max.Y - Bounds.Min.Y);
  image_drawer draw(image);
  draw.pen_color(255, 0, 0);
  for (const FRoadTriangle& RoadTriangle : RoadTriangles) {
    draw.triangle(
        RoadTriangle.V0.X - Bounds.Min.X,
        RoadTriangle.V0.Y - Bounds.Min.Y,
        RoadTriangle.V1.X - Bounds.Min.X,
        RoadTriangle.V1.Y - Bounds.Min.Y,
        RoadTriangle.V2.X - Bounds.Min.X,
        RoadTriangle.V2.Y - Bounds.Min.Y);
  }

  return true;
}
