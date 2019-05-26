#include "GeometryUtil.h"

bool FGeometryUtil::PointInTriangle(const FVector2D& Point, const FVector2D& V0, const FVector2D& V1, const FVector2D& V2) {

  // https://stackoverflow.com/a/14382692

  float A = 0.5f * (-V1.Y * V2.X + V0.Y * (-V1.X + V2.X) + V0.X * (V1.Y - V2.Y) + V1.X * V2.Y);
  int sign = A < 0 ? -1 : 1;
  float s = (V0.Y * V2.X - V0.X * V2.Y + (V2.Y - V0.Y) * Point.X + (V0.X - V2.X) * Point.Y) * sign;
  float t = (V0.X * V1.Y - V0.Y * V1.X + (V0.Y - V1.Y) * Point.X + (V1.X - V0.X) * Point.Y) * sign;
    
  return s > 0 && t > 0 && (s + t) < 2 * A * sign;
}

/*
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
*/
