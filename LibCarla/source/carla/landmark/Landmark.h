#pragma once

#include <string>
#include <vector>

#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"

namespace carla {
namespace landmark {

class Landmark {
public:

  Landmark(const std::vector<geom::Vector2D>& outline);
  
  static std::vector<Landmark> Load(const std::string& file, const geom::Vector2D& offset = geom::Vector2D(0, 0));

  const std::vector<geom::Vector2D>& Outline() const { return _outline; }

  std::vector<geom::Vector3D> GetWallMeshTriangles(float height) const;
  std::vector<geom::Vector3D> GetOutlineMeshTriangles(float height) const;

private:

  std::vector<geom::Vector2D> _outline;
  std::vector<size_t> _triangulation;

};

}
}



