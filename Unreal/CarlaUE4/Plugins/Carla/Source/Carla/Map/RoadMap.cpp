#include "RoadMap.h"
#include "Runtime/Engine/Public/GeomTools.h"
#include "bitmap_image.hpp"
#include "GeometryUtil.h"
#include "Algo/Reverse.h"

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

  InitOccupancyGrid();
  InitPerimeterPolygons();
}

void FRoadMap::InitOccupancyGrid() {
  // Coordinates are flipped such that X is upward and Y is rightward on occupancy grid.

  FVector2D TopLeft(Bounds.Max.X, Bounds.Min.Y);

  OccupancyGrid = FOccupancyGrid(
      FMath::FloorToInt((Bounds.Max.Y - Bounds.Min.Y) / Resolution),
      FMath::FloorToInt((Bounds.Max.X - Bounds.Min.X) / Resolution));

  for (const FRoadTriangle& RoadTriangle : RoadTriangles) {
    FVector2D V0(RoadTriangle.V0.X, RoadTriangle.V0.Y);
    FVector2D V1(RoadTriangle.V1.X, RoadTriangle.V1.Y);
    FVector2D V2(RoadTriangle.V2.X, RoadTriangle.V2.Y); 
    FBox TriBounds = RoadTriangle.GetBounds();

    // Calculate bounding pixels.
    FIntPoint TriBoundsTopLeftPixel = Point2DToPixel(FVector2D(TriBounds.Max.X, TriBounds.Min.Y));
    FIntPoint TriBoundsBottomRightPixel = Point2DToPixel(FVector2D(TriBounds.Min.X, TriBounds.Max.Y));

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
  
void FRoadMap::InitPerimeterPolygons() {
  FOccupancyGrid ProcessedGrid = FOccupancyGrid(
      OccupancyGrid.GetWidth(), 
      OccupancyGrid.GetHeight());

  for (int Y = 0; Y < ProcessedGrid.GetHeight(); Y++){
    for (int X = 0; X < ProcessedGrid.GetWidth(); X++) {
      if (ProcessedGrid(X, Y)) continue;
      if (OccupancyGrid(X, Y)) continue;
      
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
          if (OccupancyGrid(NextPixel)) continue;
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
      while (true) {
        EdgeHalfGrid(CurrentEdgeHalf) = false;

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

          if (EdgeHalfGrid(TestEdgeHalf)) {
            HasNext = true;
            CurrentEdgeHalf = TestEdgeHalf;
            break;
          }
        }
        
        if (!HasNext) break;
      }
    }
  }

}

FIntPoint FRoadMap::Point2DToPixel(const FVector2D& Point) const {
  return FIntPoint(
    FMath::FloorToInt((Point.Y - Bounds.Min.Y) / Resolution),
    FMath::FloorToInt(-(Point.X - Bounds.Max.X) / Resolution));
}
  
FVector2D FRoadMap::PixelToPoint2D(const FIntPoint& Pixel) const {
  return FVector2D(
      Bounds.Max.X - (Pixel.Y + 0.5) * Resolution,
      Bounds.Min.Y + (Pixel.X + 0.5) * Resolution);
}

FVector FRoadMap::RandPoint() const {
  float V = FMath::FRandRange(0, Area);
  int I = 0;
  for (; V > 0 && I < RoadTriangles.Num(); I++) {
    V -= RoadTriangles[I].GetArea();
  }
  return RoadTriangles[I - 1].RandPoint();
}
  
TArray<FVector2D> FRoadMap::RandPath(double Radius) const {
  struct TNode {
    const TNode* Parent;
    FIntPoint Pixel;
    TArray<TNode> Nodes; 
    TNode(const TNode* Parent, const FIntPoint& Pixel) : Parent(Parent), Pixel(Pixel) { 
      // To ensure that data does not rellocate lest invalidating references to 
      // previous locations.
      Nodes.Reserve(8);
    }
  };

  FOccupancyGrid VisitedGrid(OccupancyGrid.GetWidth(), OccupancyGrid.GetHeight());
  FVector RootPoint = RandPoint();
  FVector2D RootPoint2D(RootPoint.X, RootPoint.Y);

  TArray<TNode*> Nodes;

  TNode* RootNode = new TNode(nullptr, Point2DToPixel(RootPoint2D));
  Nodes.Add(RootNode);

  TQueue<TNode*, EQueueMode::Spsc> Queue;
  Queue.Enqueue(RootNode);
  TSet<const TNode*> EdgeNodes;

  TNode* CurrentNode;
  while (Queue.Dequeue(CurrentNode)) {
    if (VisitedGrid(CurrentNode->Pixel.X, CurrentNode->Pixel.Y)) continue;
    VisitedGrid(CurrentNode->Pixel.X, CurrentNode->Pixel.Y) = true;

    for (const FIntPoint& NextPixel : { 
        FIntPoint(CurrentNode->Pixel.X + 3, CurrentNode->Pixel.Y),
        FIntPoint(CurrentNode->Pixel.X - 3, CurrentNode->Pixel.Y),
        FIntPoint(CurrentNode->Pixel.X, CurrentNode->Pixel.Y + 3),
        FIntPoint(CurrentNode->Pixel.X, CurrentNode->Pixel.Y - 3),
        FIntPoint(CurrentNode->Pixel.X + 3, CurrentNode->Pixel.Y + 3),
        FIntPoint(CurrentNode->Pixel.X + 3, CurrentNode->Pixel.Y - 3),
        FIntPoint(CurrentNode->Pixel.X - 3, CurrentNode->Pixel.Y + 3),
        FIntPoint(CurrentNode->Pixel.X - 3, CurrentNode->Pixel.Y - 3)}) {

      if (NextPixel.X < 0) continue;
      if (NextPixel.X >= VisitedGrid.GetWidth()) continue;
      if (NextPixel.Y < 0) continue;
      if (NextPixel.Y >= VisitedGrid.GetHeight()) continue;
      if (VisitedGrid(NextPixel)) continue;
      if (!OccupancyGrid(NextPixel)) continue;
      if (FVector2D::Distance(RootPoint2D, PixelToPoint2D(NextPixel)) > Radius) {
        if (CurrentNode->Parent) {
          EdgeNodes.Add(CurrentNode->Parent);
          continue;
        }
      }

      Queue.Enqueue(&(CurrentNode->Nodes.Emplace_GetRef(CurrentNode, NextPixel)));
    } 
  }

  UE_LOG(LogCarla, Display, TEXT("Num of vertices = %d"), EdgeNodes.Num());

  const TNode* EdgeNode = EdgeNodes[FSetElementId::FromInteger(FMath::RandRange(0, EdgeNodes.Num() - 1))];
  TArray<FVector2D> Path;
  while (EdgeNode) {
    Path.Add(PixelToPoint2D(EdgeNode->Pixel));
    EdgeNode = EdgeNode->Parent;
  }
  Algo::Reverse(Path);

  return Path;
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
  
  /*
  TArray<FVector2D> Path = RandPath(10000);
  draw.pen_color(255, 0, 0);
  for (int I = 0; I < Path.Num() - 1; I++) {
    // TODO check for possible out of bounds.
    FIntPoint Start = Point2DToPixel(Path[I]);
    FIntPoint End = Point2DToPixel(Path[I + 1]);
    draw.line_segment(Start.X, Start.Y, End.X, End.Y);
  }
  */

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
