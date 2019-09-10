#pragma once

#include "carla/geom/Vector2D.h"

namespace carla {
namespace geom {

class Segment2D {
  public:
    Vector2D start;
    Vector2D end;

    Segment2D() = default;

    Segment2D(const Vector2D& start, const Vector2D end) 
      : start(start), end(end) { }

    bool operator==(const Segment2D& rhs) const {
      return (start == rhs.start) && (end == rhs.end);
    }

    bool operator!=(const Segment2D& rhs) const {
      return !(*this == rhs);
    }
};

}
}
