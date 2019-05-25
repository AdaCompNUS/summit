#include "RoadPolygon.h"

FRoadPolygon::FRoadPolygon(const TArray<FVector>& Vertices) {
  int Count = Vertices.Num();
  Area = Vertices[Count - 1].X * Vertices[0].Y - Vertices[0].X * Vertices[Count - 1].Y;

  bool HasValue = false;
  for (int Index = 0; Index < Count; Index++) {
    const FVector& Vertex = Vertices[Index];

    // Copy and flatten vertex.
    this->Vertices.Emplace(Vertex.X, Vertex.Y);
    
    // Shoelace formula for simple polygon area.
    if (Index < Count - 1) {
      Area += Vertices[Index].X * Vertices[Index + 1].Y - Vertices[Index + 1].X * Vertices[Index].Y;
    }

    if (!HasValue) {
      Bounds.Min.X = Bounds.Max.X = Vertex.X;
      Bounds.Min.Y = Bounds.Max.Y = Vertex.Y;
      MaxZ = Vertex.Z;
      HasValue = true;
    }
    else {
      Bounds.Min.X = FMath::Min(Bounds.Min.X, Vertex.X);
      Bounds.Max.X = FMath::Max(Bounds.Max.X, Vertex.X);
      Bounds.Min.Y = FMath::Min(Bounds.Min.Y, Vertex.Y);
      Bounds.Max.Y = FMath::Max(Bounds.Max.Y, Vertex.Y);
      MaxZ = FMath::Max(MaxZ, Vertex.Z);
    }
  }

  Area = FMath::Abs(Area) / 2;
}

bool FRoadPolygon::InPolygon(const FVector2D& Point) const {
  bool C = 0;
  for (int I = 0, J = Vertices.Num() - 1; I < Vertices.Num(); J = I++) {
    if (((Vertices[I].Y > Point.Y) != (Vertices[J].Y > Point.Y)) && 
        (Point.X < (Vertices[J].X - Vertices[I].X) * (Point.Y - Vertices[I].Y) / (Vertices[J].Y - Vertices[I].Y) + Vertices[I].X)) {
      C = !C;
    }
  }
  return C;
}

FVector2D FRoadPolygon::RandPoint() const {
  FVector2D Point;
  do {
    Point = FVector2D(FMath::FRandRange(Bounds.Min.X, Bounds.Max.X), FMath::FRandRange(Bounds.Min.Y, Bounds.Max.Y));
  } while (!InPolygon(Point));
  return Point;
}
