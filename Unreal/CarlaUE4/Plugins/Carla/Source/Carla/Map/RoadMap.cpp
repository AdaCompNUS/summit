#include "RoadMap.h"
#include "Runtime/Engine/Public/GeomTools.h"
#include "bitmap_image.hpp"
#include "GeometryUtil.h"

FRoadMap::FRoadMap(const TArray<FRoadTriangle>& RoadTriangles, float Resolution) 
  : RoadTriangles(RoadTriangles), Resolution(Resolution) {
  
  Bounds = RoadTriangles[0].GetBounds();
  Area = RoadTriangles[0].GetArea();

  for (int I = 0; I < RoadTriangles.Num(); I++) {
    const FRoadTriangle& RoadTriangle = RoadTriangles[I];
    FBox TriBounds = RoadTriangle.GetBounds();
    Bounds.Min.X = FMath::Min(Bounds.Min.X, TriBounds.Min.X);
    Bounds.Min.Y = FMath::Min(Bounds.Min.Y, TriBounds.Min.Y);
    Bounds.Min.Z = FMath::Min(Bounds.Min.Z, TriBounds.Min.Z);
    Bounds.Max.X = FMath::Max(Bounds.Max.X, TriBounds.Max.X);
    Bounds.Max.Y = FMath::Max(Bounds.Max.Y, TriBounds.Max.Y);
    Bounds.Max.Z = FMath::Max(Bounds.Max.Z, TriBounds.Max.Z);
    Area += RoadTriangle.GetArea();
  }
  
  FVector2D TopLeft(Bounds.Max.X, Bounds.Min.Y);
  
  // Coordinates are flipped such that X is upward and Y is rightward on occupancy grid.

  OccupancyGrid = FOccupancyGrid(
      FMath::FloorToInt((Bounds.Max.Y - Bounds.Min.Y) / Resolution),
      FMath::FloorToInt((Bounds.Max.X - Bounds.Min.X) / Resolution));

  for (const FRoadTriangle& RoadTriangle : RoadTriangles) {
    FVector2D V0(RoadTriangle.V0.X, RoadTriangle.V0.Y);
    FVector2D V1(RoadTriangle.V1.X, RoadTriangle.V1.Y);
    FVector2D V2(RoadTriangle.V2.X, RoadTriangle.V2.Y);

    // Calculate relative triangle bounds.
    FBox TriBounds = RoadTriangle.GetBounds();
    FVector2D TriBoundsTopLeftOffset = FVector2D(TriBounds.Max.X, TriBounds.Min.Y) - TopLeft;
    FVector2D TriBoundsBottomRightOffset = FVector2D(TriBounds.Min.X, TriBounds.Max.Y) - TopLeft;
    
    // Calculate bounding pixels.
    FIntPoint TriBoundsTopLeftPixel(
        FMath::FloorToInt(TriBoundsTopLeftOffset.Y / Resolution), 
        FMath::FloorToInt(-TriBoundsTopLeftOffset.X / Resolution));
    FIntPoint TriBoundsBottomRightPixel(
        FMath::FloorToInt(TriBoundsBottomRightOffset.Y / Resolution), 
        FMath::FloorToInt(-TriBoundsBottomRightOffset.X / Resolution));

    // Clip to occupancy grid.
    TriBoundsTopLeftPixel.X = FMath::Max(0,
        FMath::Min(OccupancyGrid.GetWidth() - 1, TriBoundsTopLeftPixel.X));
    TriBoundsTopLeftPixel.Y = FMath::Max(0, 
        FMath::Min(OccupancyGrid.GetHeight() - 1, TriBoundsTopLeftPixel.Y));
    TriBoundsBottomRightPixel.X = FMath::Max(0, 
        FMath::Min(OccupancyGrid.GetWidth() - 1, TriBoundsBottomRightPixel.X));
    TriBoundsBottomRightPixel.Y = FMath::Max(0, 
        FMath::Min(OccupancyGrid.GetHeight() - 1, TriBoundsBottomRightPixel.Y));
    
    // Loop through bounded pixels, and consider pixel bounds.
    for (int X = TriBoundsTopLeftPixel.X; X <= TriBoundsBottomRightPixel.X; X++) {
      for (int Y = TriBoundsTopLeftPixel.Y; Y <= TriBoundsBottomRightPixel.Y; Y++) {
        FVector2D P0(TopLeft.X - Y * Resolution, TopLeft.Y + X * Resolution);
        FVector2D P1(TopLeft.X - Y * Resolution, TopLeft.Y + (X + 1) * Resolution);
        FVector2D P2(TopLeft.X - (Y + 1) * Resolution, TopLeft.Y + X * Resolution);
        FVector2D P3(TopLeft.X - (Y + 1) * Resolution, TopLeft.Y + (X + 1) * Resolution);
        
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
          OccupancyGrid(X, Y) = true; 
        }
      }
    }
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

void FRoadMap::RenderMonteCarloBitmap(const FString& FileName, int Trials) const {
  
  cartesian_canvas canvas(OccupancyGrid.GetWidth(), OccupancyGrid.GetHeight());
  canvas.image().clear(0);
  image_drawer draw(canvas.image());

  draw.pen_color(255, 255, 255);
  for (int Y = 0; Y < OccupancyGrid.GetHeight(); Y++) {
    for (int X = 0; X < OccupancyGrid.GetWidth(); X++) {
      if (OccupancyGrid(X, Y)) {
        draw.plot_pen_pixel(X, Y);
      }
    }
  }

  canvas.pen_color(0, 0, 255);
  for (int I = 0; I < Trials; I++) {
    FVector Point = RandPoint();
    canvas.fill_circle(
        ((Point.Y - (Bounds.Min.Y + Bounds.Max.Y) / 2)) / Resolution,
        ((Point.X - (Bounds.Min.X + Bounds.Max.X) / 2)) / Resolution,
        5);
  }
  
  canvas.image().save_image(TCHAR_TO_UTF8(*FileName));
  
  UE_LOG(LogCarla, Display, TEXT("Bitmap saved to %s"), *FileName);
}
