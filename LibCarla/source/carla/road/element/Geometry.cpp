// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/element/Geometry.h"

#include "carla/Debug.h"
#include "carla/Exception.h"
#include "carla/geom/Location.h"
#include "carla/geom/Math.h"

#include <cephes/fresnel.h>

#include <cmath>
#include <stdexcept>

namespace carla {
namespace road {
namespace element {

  void DirectedPoint::ApplyLateralOffset(float lateral_offset) {
    /// @todo Z axis??
    auto normal_x =  std::sin(static_cast<float>(tangent));
    auto normal_y = -std::cos(static_cast<float>(tangent));
    location.x += lateral_offset * normal_x;
    location.y += lateral_offset * normal_y;
  }

  DirectedPoint GeometryLine::PosFromDist(double dist) const {
    DEBUG_ASSERT(_length > 0.0);
    dist = geom::Math::Clamp(dist, 0.0, _length);
    DirectedPoint p(_start_position, _heading);
    p.location.x += static_cast<float>(dist * std::cos(p.tangent));
    p.location.y += static_cast<float>(dist * std::sin(p.tangent));
    return p;
  }

  DirectedPoint GeometryArc::PosFromDist(double dist) const {
    dist = geom::Math::Clamp(dist, 0.0, _length);
    DEBUG_ASSERT(_length > 0.0);
    DEBUG_ASSERT(std::fabs(_curvature) > 1e-15);
    const double radius = 1.0 / _curvature;
    constexpr double pi_half = geom::Math::Pi<double>() / 2.0;
    DirectedPoint p(_start_position, _heading);
    p.location.x += static_cast<float>(radius * std::cos(p.tangent + pi_half));
    p.location.y += static_cast<float>(radius * std::sin(p.tangent + pi_half));
    p.tangent += dist * _curvature;
    p.location.x -= static_cast<float>(radius * std::cos(p.tangent + pi_half));
    p.location.y -= static_cast<float>(radius * std::sin(p.tangent + pi_half));
    return p;
  }

  DirectedPoint GeometrySpiral::PosFromDist(double dist) const {
    // not working yet with negative values
    dist = geom::Math::Clamp(dist, 0.0, _length);
    DEBUG_ASSERT(_length > 0.0);
    DEBUG_ASSERT(std::fabs(_curve_end) > 1e-15);
    const double radius = 1.0 / _curve_end;
    const double extra_norm = 1.0 / std::sqrt(geom::Math::Pi<double>() / 2.0);
    const double norm = 1.0 / std::sqrt(2.0 * radius * _length);
    const double length = dist * norm;
    double S, C;
    fresnl(length * extra_norm, &S, &C);
    S /= (norm * extra_norm);
    C /= (norm * extra_norm);
    DirectedPoint p(_start_position, _heading);
    const double cos_a = std::cos(p.tangent);
    const double sin_a = std::sin(p.tangent);
    p.location.x += static_cast<float>(C * cos_a - S * sin_a);
    p.location.y += static_cast<float>(S * cos_a + C * sin_a);
    p.tangent += length * length;

    return p;
  }

  /// @todo
  std::pair<float, float> GeometrySpiral::DistanceTo(const geom::Location &) const {
    throw_exception(std::runtime_error("not implemented"));
  }
  
  DirectedPoint GeometryParamPoly3::PosFromDist(double dist) const {
    DEBUG_ASSERT(_length > 0.0);
    dist = geom::Math::Clamp(dist / _length, 0.0, 1.0);
    DirectedPoint p(_start_position, _heading);
    constexpr double pi_half = geom::Math::Pi<double>() / 2.0;
    const double u = _aU + _bU * dist + _cU * dist * dist + _dU * dist * dist * dist;
    const double v = _aV + _bV * dist + _cV * dist * dist + _dV * dist * dist * dist;
    p.location.x += static_cast<float>(u * std::cos(p.tangent));
    p.location.y += static_cast<float>(u * std::sin(p.tangent));
    p.location.x += static_cast<float>(v * std::cos(p.tangent + pi_half));
    p.location.y += static_cast<float>(v * std::sin(p.tangent + pi_half));

    const double du = _bU + 2 * _cU * dist + 3 * _dU * dist * dist;
    const double dv = _bV + 2 * _cV * dist + 3 * _dV * dist * dist;
    const double dheading = std::atan2(dv, du);
    p.tangent += dheading;

    return p;
  }

  std::pair<float, float> GeometryParamPoly3::DistanceTo(const geom::Location &) const {
    throw_exception(std::runtime_error("not implemented"));
  }

} // namespace element
} // namespace road
} // namespace carla
