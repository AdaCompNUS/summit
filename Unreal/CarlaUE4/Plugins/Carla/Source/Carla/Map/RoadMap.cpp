#include "RoadMap.h"
#include "Runtime/Engine/Public/GeomTools.h"
#include "bitmap_image.hpp"

FRoadMap::FRoadMap(const TArray<FRoadTriangle>& RoadTriangles) 
  : RoadTriangles(RoadTriangles), Area(0) {
  for (const FRoadTriangle& RoadTriangle : RoadTriangles) {
    Area += RoadTriangle.GetArea();
  }
}

bool FRoadMap::RenderBitmap(const FString& FileName, float Resolution) const {
  // Coordinates are flipped such that X is upward and Y is rightward on image.
    
  FBox Bounds = RoadTriangles[0].GetBounds();
  for (int I = 1; I < RoadTriangles.Num(); I++) {
    const FRoadTriangle& RoadTriangle = RoadTriangles[I];
    Bounds.Min.X = FMath::Min(Bounds.Min.X, RoadTriangle.GetBounds().Min.X);
    Bounds.Min.Y = FMath::Min(Bounds.Min.Y, RoadTriangle.GetBounds().Min.Y);
    Bounds.Min.Z = FMath::Min(Bounds.Min.Z, RoadTriangle.GetBounds().Min.Z);
    Bounds.Max.X = FMath::Max(Bounds.Max.X, RoadTriangle.GetBounds().Max.X);
    Bounds.Max.Y = FMath::Max(Bounds.Max.Y, RoadTriangle.GetBounds().Max.Y);
    Bounds.Max.Z = FMath::Max(Bounds.Max.Z, RoadTriangle.GetBounds().Max.Z);
  }
  FVector2D Center((Bounds.Min.X + Bounds.Max.X) / 2, (Bounds.Min.Y + Bounds.Max.Y) / 2);
  
  cartesian_canvas canvas(
      (Bounds.Max.Y - Bounds.Min.Y) / Resolution, 
      (Bounds.Max.X - Bounds.Min.X) / Resolution);
  canvas.image().clear(0);
  
  canvas.pen_color(255, 0, 0);
  for (const FRoadTriangle& RoadTriangle : RoadTriangles) {
    canvas.fill_triangle(
        (RoadTriangle.V0.Y - Center.Y) / Resolution,
        (RoadTriangle.V0.X - Center.X) / Resolution,
        (RoadTriangle.V1.Y - Center.Y) / Resolution,
        (RoadTriangle.V1.X - Center.X) / Resolution,
        (RoadTriangle.V2.Y - Center.Y) / Resolution,
        (RoadTriangle.V2.X - Center.X) / Resolution);
  }

  canvas.image().save_image(TCHAR_TO_UTF8(*FileName));
  
  UE_LOG(LogCarla, Display, TEXT("Bitmap saved to %s"), *FileName);

  return true;
}
