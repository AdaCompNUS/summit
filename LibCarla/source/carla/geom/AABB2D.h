#pragma once

#include "carla/geom/Vector2D.h"

namespace carla {
namespace geom {

class AABB2D {
  public:
    Vector2D bounds_min;
    Vector2D bounds_max;

    AABB2D() = default;

    AABB2D(const Vector2D& bounds_min, const Vector2D bounds_max)
      : bounds_min(bounds_min), bounds_max(bounds_max) { }

    bool operator==(const AABB2D& rhs) const {
      return (bounds_min == rhs.bounds_min) && (bounds_max == rhs.bounds_max);
    }

    bool operator!=(const AABB2D& rhs) const {
      return !(*this == rhs);
    }
};

}
}
