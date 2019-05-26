#include "RoadMap.h"
#include "Runtime/Engine/Public/GeomTools.h"
#include "bitmap_image.hpp"
#include "GeometryUtil.h"

FRoadMap::FRoadMap(const TArray<FRoadTriangle>& RoadTriangles) 
  : RoadTriangles(RoadTriangles), Area(0) {
  for (const FRoadTriangle& RoadTriangle : RoadTriangles) {
    Area += RoadTriangle.GetArea();
  }
}

FVector FRoadMap::RandPoint() const {
  float V = FMath::FRandRange(0, Area);
  int I = 0;
  for (; V > 0 && I < RoadTriangles.Num(); I++) {
    V -= RoadTriangles[I].GetArea();
  }
  return RoadTriangles[I - 1].RandPoint();
}

void FRoadMap::RenderMonteCarloBitmap(const FString& FileName, float Resolution, int Trials) const {
  // Coordinates are flipped such that X is upward and Y is rightward on image.
    
  FBox MapBounds = RoadTriangles[0].GetBounds();
  for (int I = 1; I < RoadTriangles.Num(); I++) {
    const FRoadTriangle& RoadTriangle = RoadTriangles[I];
    MapBounds.Min.X = FMath::Min(MapBounds.Min.X, RoadTriangle.GetBounds().Min.X);
    MapBounds.Min.Y = FMath::Min(MapBounds.Min.Y, RoadTriangle.GetBounds().Min.Y);
    MapBounds.Min.Z = FMath::Min(MapBounds.Min.Z, RoadTriangle.GetBounds().Min.Z);
    MapBounds.Max.X = FMath::Max(MapBounds.Max.X, RoadTriangle.GetBounds().Max.X);
    MapBounds.Max.Y = FMath::Max(MapBounds.Max.Y, RoadTriangle.GetBounds().Max.Y);
    MapBounds.Max.Z = FMath::Max(MapBounds.Max.Z, RoadTriangle.GetBounds().Max.Z);
  }
  FVector2D MapTopLeft(MapBounds.Max.X, MapBounds.Min.Y);
  
  bitmap_image image(
      (MapBounds.Max.Y - MapBounds.Min.Y) / Resolution, 
      (MapBounds.Max.X - MapBounds.Min.X) / Resolution);
  image_drawer draw(image);
  draw.pen_color(255, 255, 255);

  for (const FRoadTriangle& RoadTriangle : RoadTriangles) {
    FVector2D V0(RoadTriangle.V0.X, RoadTriangle.V0.Y);
    FVector2D V1(RoadTriangle.V1.X, RoadTriangle.V1.Y);
    FVector2D V2(RoadTriangle.V2.X, RoadTriangle.V2.Y);

    // Calculate relative triangle bounds.
    FBox Bounds = RoadTriangle.GetBounds();
    FVector2D BoundsTopLeftOffset = FVector2D(Bounds.Max.X, Bounds.Min.Y) - MapTopLeft;
    FVector2D BoundsBottomRightOffset = FVector2D(Bounds.Min.X, Bounds.Max.Y) - MapTopLeft;
    
    // Calculate bounding pixels.
    FIntPoint BoundsTopLeftPixel(
        FMath::FloorToInt(BoundsTopLeftOffset.Y / Resolution), 
        FMath::FloorToInt(-BoundsTopLeftOffset.X / Resolution));
    FIntPoint BoundsBottomRightPixel(
        FMath::FloorToInt(BoundsBottomRightOffset.Y / Resolution), 
        FMath::FloorToInt(-BoundsBottomRightOffset.X / Resolution));
    
    // Loop through bounded pixels, and consider pixel bounds.
    for (int X = BoundsTopLeftPixel.X; X <= BoundsBottomRightPixel.X; X++) {
      for (int Y = BoundsTopLeftPixel.Y; Y <= BoundsBottomRightPixel.Y; Y++) {
        FVector2D P0(MapTopLeft.X - Y * Resolution, MapTopLeft.Y + X * Resolution);
        FVector2D P1(MapTopLeft.X - Y * Resolution, MapTopLeft.Y + (X + 1) * Resolution);
        FVector2D P2(MapTopLeft.X - (Y + 1) * Resolution, MapTopLeft.Y + X * Resolution);
        FVector2D P3(MapTopLeft.X - (Y + 1) * Resolution, MapTopLeft.Y + (X + 1) * Resolution);
        
        // || used for short circuiting.
        bool Intersects = false;
        
        // Pixel vertices in triangle.
        Intersects = Intersects || FGeometryUtil::PointInTriangle(P0, V0, V1, V2);
        Intersects = Intersects || FGeometryUtil::PointInTriangle(P1, V0, V1, V2);
        Intersects = Intersects || FGeometryUtil::PointInTriangle(P2, V0, V1, V2);
        Intersects = Intersects || FGeometryUtil::PointInTriangle(P3, V0, V1, V2);
        
        // Triangle vertices in pixel.
        Intersects = Intersects || (V0.X <= P0.X && V0.X >= P3.X && V0.Y >= P0.Y && V0.Y <= P3.Y);
        Intersects = Intersects || (V1.X <= P0.X && V1.X >= P3.X && V1.Y >= P0.Y && V1.Y <= P3.Y);
        Intersects = Intersects || (V2.X <= P0.X && V2.X >= P3.X && V2.Y >= P0.Y && V2.Y <= P3.Y);

        if (Intersects) {
          draw.plot_pen_pixel(X, Y); 
        }
      }
    }
  }
  
  image.save_image(TCHAR_TO_UTF8(*FileName));
  
  UE_LOG(LogCarla, Display, TEXT("Bitmap saved to %s"), *FileName);
}
