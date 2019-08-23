#pragma once

#include <string>

namespace carla {
namespace landmark {

class LandmarkMap {
public:

  static LandmarkMap Load(const std::string& file);

};

}
}



