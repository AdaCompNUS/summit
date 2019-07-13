#pragma once

#include "carla/geom/Vector2D.h"

namespace carla {
namespace geom {

class Triangle2D {
  public:
    Vector2D v0;
    Vector2D v1;
    Vector2D v2;

    Triangle2D() = default;

    Triangle2D(const Vector2D &v0, const Vector2D &v1, const Vector2D &v2)
        : v0(v0), v1(v1), v2(v2) { }
    
    bool operator==(const Triangle2D &rhs) const {
      return (v0 == rhs.v0) && (v1 == rhs.v1) && (v2 == rhs.v2);
    }

    bool operator!=(const Triangle2D &rhs) const {
      return !(*this == rhs);
    }
   
};
   

}
}
