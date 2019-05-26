#pragma once

class FGeometryUtil {

public:

  static bool PointInTriangle(const FVector2D& Point, const FVector2D& V0, const FVector2D& V1, const FVector2D& V2);

private:
  
  FGeometryUtil() {}
};
