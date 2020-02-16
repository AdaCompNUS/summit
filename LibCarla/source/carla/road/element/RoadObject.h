// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/road/element/RoadObjectVisitor.h"
#include "carla/NonCopyable.h"

#include <map>
#include <string>
#include <vector>

namespace carla {
namespace road {
namespace element {

  class RoadObject : private NonCopyable {
  public:

    virtual ~RoadObject() = default;

    virtual void AcceptVisitor(RoadObjectVisitor &) = 0;

    /// Distance from road's start location.
    double GetDistance() const {
      return _s;
    }

  protected:

    RoadObject(double distance = 0.0) : _s(distance) {}

  private:

    double _s;
  };

} // namespace element
} // namespace road
} // namespace carla
