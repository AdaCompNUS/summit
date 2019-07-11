#pragma once

#include <cstdint>

namespace carla {
namespace lanenetwork {

struct RoutePoint {
  int64_t id;
  float offset;

  RoutePoint(int64_t id, float offset) : id(id), offset(offset) { }
};

}
}
