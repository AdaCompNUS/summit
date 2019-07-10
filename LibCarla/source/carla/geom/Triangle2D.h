#pragma once

namespace carla {
namespace geom {

class Triangle2D {
  public:
    carla::geom::Vector2D v0;
    carla::geom::Vector2D v1;
    carla::geom::Vector2D v2;

    Triangle2D() = default;

    Triangle2D(const carla::geom::Vector2D &v0, const carla::geom::Vector2D &v1, const carla::geom::Vector2D &v2)
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
