#include "OccupancyMap.h"
#include "Runtime/Engine/Public/GeomTools.h"
#include "GeometryUtil.h"
#include "Algo/Reverse.h"

FOccupancyMap::FOccupancyMap(const TArray<FOccupancyTriangle>& OccupancyTriangles) 
    : OccupancyTriangles(OccupancyTriangles), Area(0) {
  for (const FOccupancyTriangle& OccupancyTriangle : OccupancyTriangles) {
    Area += OccupancyTriangle.GetArea();
  }
}

FVector2D FOccupancyMap::RandPoint() const {
  float V = FMath::FRandRange(0, Area);
  int I = 0;
  for (; V > 0 && I < OccupancyTriangles.Num(); I++) {
    V -= OccupancyTriangles[I].GetArea();
  }
  return OccupancyTriangles[I - 1].RandPoint();
}

FOccupancyArea FOccupancyMap::GetOccupancyArea(const FBox2D& Bounds, float Resolution, int OffroadPolygonEdgeInterval) const {
  FOccupancyArea OccupancyArea;
  OccupancyArea.Bounds = Bounds;
  OccupancyArea.Resolution = Resolution;
  OccupancyArea.OffroadPolygonEdgeInterval = OffroadPolygonEdgeInterval;

  // ===== Helper functions =====
  // Coordinates are flipped such that X is upward and Y is rightward on occupancy grid.
  FVector2D TopLeft(Bounds.Max.X, Bounds.Min.Y);

  // ===== Occupancy grid calculation =====
  OccupancyArea.OccupancyGrid = FOccupancyGrid(
      FMath::FloorToInt((Bounds.Max.Y - Bounds.Min.Y) / Resolution),
      FMath::FloorToInt((Bounds.Max.X - Bounds.Min.X) / Resolution));

  for (const FOccupancyTriangle& OccupancyTriangle : OccupancyTriangles) {
    const FVector2D& V0 = OccupancyTriangle.V0;
    const FVector2D& V1 = OccupancyTriangle.V1;
    const FVector2D& V2 = OccupancyTriangle.V2; 
    FBox2D TriBounds = OccupancyTriangle.GetBounds();

    // Calculate bounding pixels.
    FIntPoint TriBoundsTopLeftPixel = OccupancyArea.Point2DToPixel(FVector2D(TriBounds.Max.X, TriBounds.Min.Y));
    FIntPoint TriBoundsBottomRightPixel = OccupancyArea.Point2DToPixel(FVector2D(TriBounds.Min.X, TriBounds.Max.Y));
    
    // Loop through bounded pixels, and consider pixel bounds.
    for (int X = TriBoundsTopLeftPixel.X; X <= TriBoundsBottomRightPixel.X; X++) {
      if (X < 0 || X >= OccupancyArea.OccupancyGrid.GetWidth()) continue;
      for (int Y = TriBoundsTopLeftPixel.Y; Y <= TriBoundsBottomRightPixel.Y; Y++) {
        if (Y < 0 || Y >= OccupancyArea.OccupancyGrid.GetHeight()) continue;

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
          OccupancyArea.OccupancyGrid(X, Y) = true; 
        }
      }
    }
  }


  
  // ===== Offroad polygons calculation =====
  FOccupancyGrid ProcessedGrid = FOccupancyGrid(
      OccupancyArea.OccupancyGrid.GetWidth(), 
      OccupancyArea.OccupancyGrid.GetHeight());

  for (int Y = 0; Y < ProcessedGrid.GetHeight(); Y++){
    for (int X = 0; X < ProcessedGrid.GetWidth(); X++) {
      if (ProcessedGrid(X, Y)) continue;
      if (OccupancyArea.OccupancyGrid(X, Y)) continue;
      
      TArray<FIntPoint> ObstaclePixels;

      FIntPoint ObstacleBoundsMin;
      FIntPoint ObstacleBoundsMax;
      bool HasObstacleBounds = false;



      // ===== Breadth-first search for obstacle pixels. ===== 
      TQueue<FIntPoint, EQueueMode::Spsc> Queue;
      Queue.Enqueue(FIntPoint(X, Y));
      FIntPoint CurrentPixel;
      int Size = 0;
      while (Queue.Dequeue(CurrentPixel)) {
        if (ProcessedGrid(CurrentPixel)) continue;
        
        if (!HasObstacleBounds) {
          ObstacleBoundsMin.X = ObstacleBoundsMax.X = CurrentPixel.X;
          ObstacleBoundsMin.Y = ObstacleBoundsMax.Y = CurrentPixel.Y;
          HasObstacleBounds = true;
        } else {
          ObstacleBoundsMin.X = FMath::Min(ObstacleBoundsMin.X, CurrentPixel.X);
          ObstacleBoundsMax.X = FMath::Max(ObstacleBoundsMax.X, CurrentPixel.X);
          ObstacleBoundsMin.Y = FMath::Min(ObstacleBoundsMin.Y, CurrentPixel.Y);
          ObstacleBoundsMax.Y = FMath::Max(ObstacleBoundsMax.Y, CurrentPixel.Y);
        }

        ProcessedGrid(CurrentPixel) = true;
        ObstaclePixels.Add(CurrentPixel);
    
        for (const FIntPoint& NextPixel : { 
            FIntPoint(CurrentPixel.X + 1, CurrentPixel.Y),
            FIntPoint(CurrentPixel.X, CurrentPixel.Y + 1),
            FIntPoint(CurrentPixel.X - 1, CurrentPixel.Y),
            FIntPoint(CurrentPixel.X, CurrentPixel.Y - 1)}) {
          if (NextPixel.X < 0) continue;
          if (NextPixel.X >= ProcessedGrid.GetWidth()) continue;
          if (NextPixel.Y < 0) continue;
          if (NextPixel.Y >= ProcessedGrid.GetHeight()) continue;
          if (OccupancyArea.OccupancyGrid(NextPixel)) continue;
          if (ProcessedGrid(NextPixel)) continue;

          Queue.Enqueue(NextPixel);
        }
      }



      // ===== Search for perimeter edge halfs. =====
      FOccupancyGrid ObstacleGrid(
          ObstacleBoundsMax.X - ObstacleBoundsMin.X + 1,
          ObstacleBoundsMax.Y - ObstacleBoundsMin.Y + 1);
      FOccupancyGrid EdgeHalfGrid(
          2 * ObstacleGrid.GetWidth() + 1, 
          2 * ObstacleGrid.GetHeight() + 1);

      for (const FIntPoint& Pixel : ObstaclePixels) {
        ObstacleGrid(Pixel - ObstacleBoundsMin) = true;
      }

      FIntPoint StartEdgeHalf;
      for (const FIntPoint& Pixel : ObstaclePixels) {
        FIntPoint PixelR = Pixel - ObstacleBoundsMin;

        if (PixelR.X - 1 < 0 || !ObstacleGrid(PixelR.X - 1, PixelR.Y)) {
          EdgeHalfGrid(StartEdgeHalf = FIntPoint(2 * PixelR.X, 2 * PixelR.Y + 1)) = true; 
        }
        if (PixelR.X + 1 >= ObstacleGrid.GetWidth() || !ObstacleGrid(PixelR.X + 1, PixelR.Y)) {
          EdgeHalfGrid(StartEdgeHalf = FIntPoint(2 * (PixelR.X + 1), 2 * PixelR.Y + 1)) = true;
        }
        if (PixelR.Y - 1 < 0 || !ObstacleGrid(PixelR.X, PixelR.Y - 1)) {
          EdgeHalfGrid(StartEdgeHalf = FIntPoint(2 * PixelR.X + 1, 2 * PixelR.Y)) = true;
        }
        if (PixelR.Y + 1 >= ObstacleGrid.GetHeight() || !ObstacleGrid(PixelR.X, PixelR.Y + 1)) {
          EdgeHalfGrid(StartEdgeHalf = FIntPoint(2 * PixelR.X + 1, 2 * (PixelR.Y + 1))) = true;
        }
      }

      // ===== Edge half traversal. =====
      FIntPoint CurrentEdgeHalf = StartEdgeHalf;
      TArray<FVector2D>& OffroadPolygon = OccupancyArea.OffroadPolygons.Emplace_GetRef();
      for (int I = 0;;I++) {
        EdgeHalfGrid(CurrentEdgeHalf) = false;
        if (I % OffroadPolygonEdgeInterval == 0) {
          OccupancyArea.Point2DToPixel(OffroadPolygon.Emplace_GetRef(
              Bounds.Max.X - Resolution * (ObstacleBoundsMin.Y + CurrentEdgeHalf.Y * 0.5f), 
              Bounds.Min.Y + Resolution * (ObstacleBoundsMin.X + CurrentEdgeHalf.X * 0.5f)));
        }

        bool HasNext = false;
        for (const FIntPoint& Offset : {
            // Make sure to check these diagonals first.
            FIntPoint(-1, -1), 
            FIntPoint(-1, 1), 
            FIntPoint(1, -1), 
            FIntPoint(1, 1), 
            FIntPoint(-2, 0), 
            FIntPoint(0, -2), 
            FIntPoint(2, 0), 
            FIntPoint(0, 2)}) {

          FIntPoint TestEdgeHalf = CurrentEdgeHalf + Offset;
          if (TestEdgeHalf.X < 0 || TestEdgeHalf.X >= EdgeHalfGrid.GetWidth()) continue;
          if (TestEdgeHalf.Y < 0 || TestEdgeHalf.Y >= EdgeHalfGrid.GetHeight()) continue;
          if (!EdgeHalfGrid(TestEdgeHalf)) continue;

          // Check if crossing horizontally illegally.
          if (CurrentEdgeHalf.X % 2 == 0 && Offset.X == 2) {
            if (ObstacleGrid(CurrentEdgeHalf.X / 2, CurrentEdgeHalf.Y / 2)) {
              continue;
            }
          } else if (CurrentEdgeHalf.X % 2 == 0 && Offset.X == -2) {
            if (ObstacleGrid(CurrentEdgeHalf.X / 2 - 1, CurrentEdgeHalf.Y / 2)) {
              continue;
            }
          } else if (CurrentEdgeHalf.Y % 2 == 0 && Offset.Y == 2) {
            if (ObstacleGrid(CurrentEdgeHalf.X / 2, CurrentEdgeHalf.Y / 2)) {
              continue;
            }
          } else if (CurrentEdgeHalf.Y % 2 == 0 && Offset.Y == -2) {
            if (ObstacleGrid(CurrentEdgeHalf.X / 2, CurrentEdgeHalf.Y / 2 - 1)) {
              continue;
            }
          }
            
          HasNext = true;
          CurrentEdgeHalf = TestEdgeHalf;
          break;
        }
        
        if (!HasNext) break;
      }
    }
  }

  return OccupancyArea;
}
