// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/Map.h"

#include "carla/Exception.h"
#include "carla/road/element/LaneCrossingCalculator.h"
#include "carla/road/element/RoadInfoGeometry.h"
#include "carla/road/element/RoadInfoLaneWidth.h"
#include "carla/road/element/RoadInfoMarkRecord.h"
#include "carla/road/element/RoadInfoLaneOffset.h"
#include "carla/road/element/RoadObjectCrosswalk.h"
#include "carla/road/element/RoadInfoElevation.h"
#include "carla/geom/Math.h"

#include <stdexcept>

namespace carla {
namespace road {

  using namespace carla::road::element;

  /// We use this epsilon to shift the waypoints away from the edges of the lane
  /// sections to avoid floating point precision errors.
  static constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

  // ===========================================================================
  // -- Static local methods ---------------------------------------------------
  // ===========================================================================

  template <typename T>
  static std::vector<T> ConcatVectors(std::vector<T> dst, std::vector<T> src) {
    if (src.size() > dst.size()) {
      return ConcatVectors(src, dst);
    }
    dst.insert(
        dst.end(),
        std::make_move_iterator(src.begin()),
        std::make_move_iterator(src.end()));
    return dst;
  }

  static double GetDistanceAtStartOfLane(const Lane &lane) {
    if (lane.GetId() <= 0) {
      return lane.GetDistance() + 10.0 * EPSILON;
    } else {
      return lane.GetDistance() + lane.GetLength() - 10.0 * EPSILON;
    }
  }

  static double GetDistanceAtEndOfLane(const Lane &lane) {
    if (lane.GetId() > 0) {
      return lane.GetDistance() + 10.0 * EPSILON;
    } else {
      return lane.GetDistance() + lane.GetLength() - 10.0 * EPSILON;
    }
  }

  /// Return a waypoint for each drivable lane on @a lane_section.
  template <typename FuncT>
  static void ForEachDrivableLaneImpl(
      RoadId road_id,
      const LaneSection &lane_section,
      double distance,
      FuncT &&func) {
    for (const auto &pair : lane_section.GetLanes()) {
      const auto &lane = pair.second;
      if ((static_cast<uint32_t>(lane.GetType()) & static_cast<uint32_t>(Lane::LaneType::Driving)) > 0) {
        std::forward<FuncT>(func)(Waypoint{
            road_id,
            lane_section.GetId(),
            lane.GetId(),
            distance < 0.0 ? GetDistanceAtStartOfLane(lane) : distance});
      }
    }
  }

  template <typename FuncT>
  static void ForEachLaneImpl(
      RoadId road_id,
      const LaneSection &lane_section,
      double distance,
      Lane::LaneType lane_type,
      FuncT &&func) {
    for (const auto &pair : lane_section.GetLanes()) {
      const auto &lane = pair.second;
      if ((static_cast<uint32_t>(lane.GetType()) & static_cast<uint32_t>(lane_type)) > 0) {
        std::forward<FuncT>(func)(Waypoint{
            road_id,
            lane_section.GetId(),
            lane.GetId(),
            distance < 0.0 ? GetDistanceAtStartOfLane(lane) : distance});
      }
    }
  }

  /// Return a waypoint for each drivable lane on each lane section of @a road.
  template <typename FuncT>
  static void ForEachDrivableLane(const Road &road, FuncT &&func) {
    for (const auto &lane_section : road.GetLaneSections()) {
      ForEachDrivableLaneImpl(
          road.GetId(),
          lane_section,
          -1.0, // At start of the lane
          std::forward<FuncT>(func));
    }
  }

  /// Return a waypoint for each drivable lane of the specified type on each lane section of @a road.
  template <typename FuncT>
  static void ForEachLane(const Road &road, Lane::LaneType lane_type, FuncT &&func) {
    for (const auto &lane_section : road.GetLaneSections()) {
      ForEachLaneImpl(
          road.GetId(),
          lane_section,
          -1.0, // At start of the lane
          lane_type,
          std::forward<FuncT>(func));
    }
  }

  /// Return a waypoint for each drivable lane at @a distance on @a road.
  template <typename FuncT>
  static void ForEachDrivableLaneAt(const Road &road, double distance, FuncT &&func) {
    for (const auto &lane_section : road.GetLaneSectionsAt(distance)) {
      ForEachDrivableLaneImpl(
          road.GetId(),
          lane_section,
          distance,
          std::forward<FuncT>(func));
    }
  }

  /// Returns a pair containing first = width, second = tangent,
  /// for an specific Lane given an s and a iterator over lanes
  template <typename T>
  static std::pair<double, double> ComputeTotalLaneWidth(
      const T container,
      const double s,
      const LaneId lane_id) {

    // lane_id can't be 0
    RELEASE_ASSERT(lane_id != 0);

    const bool negative_lane_id = lane_id < 0;
    double dist = 0.0;
    double tangent = 0.0;
    for (const auto &lane : container) {
      auto info = lane.second.template GetInfo<RoadInfoLaneWidth>(s);
      RELEASE_ASSERT(info != nullptr);
      const auto current_polynomial = info->GetPolynomial();
      auto current_dist = current_polynomial.Evaluate(s);
      auto current_tang = current_polynomial.Tangent(s);
      if (lane.first != lane_id) {
        dist += negative_lane_id ? current_dist : -current_dist;
        tangent += current_tang;
      } else if (lane.first == lane_id) {
        current_dist *= 0.5;
        dist += negative_lane_id ? current_dist : -current_dist;
        tangent += current_tang * 0.5;
        break;
      }
    }
    return std::make_pair(dist, tangent);
  }

  /// Assumes road_id and section_id are valid.
  static bool IsLanePresent(const MapData &data, Waypoint waypoint) {
    const auto &section = data.GetRoad(waypoint.road_id).GetLaneSectionById(waypoint.section_id);
    return section.ContainsLane(waypoint.lane_id);
  }

  // ===========================================================================
  // -- Map: Geometry ----------------------------------------------------------
  // ===========================================================================

  boost::optional<Waypoint> Map::GetClosestWaypointOnRoad(
      const geom::Location &pos,
      uint32_t lane_type) const {
    std::vector<Rtree::TreeElement> query_result =
        _rtree.GetNearestNeighboursWithFilter(Rtree::BPoint(pos.x, pos.y, pos.z),
        [&](Rtree::TreeElement const &element) {
          const Lane &lane = GetLane(element.second.first);
          return (lane_type & static_cast<uint32_t>(lane.GetType())) > 0;
        });

    if (query_result.size() == 0) {
      return boost::optional<Waypoint>{};
    }

    Rtree::BSegment segment = query_result.front().first;
    Rtree::BPoint s1 = segment.first;
    Rtree::BPoint s2 = segment.second;
    auto distance_to_segment = geom::Math::DistanceSegmentToPoint(pos,
        geom::Vector3D(s1.get<0>(), s1.get<1>(), s1.get<2>()),
        geom::Vector3D(s2.get<0>(), s2.get<1>(), s2.get<2>()));

    Waypoint result_start = query_result.front().second.first;
    Waypoint result_end = query_result.front().second.second;

    if (result_start.lane_id < 0) {
      double delta_s = distance_to_segment.first;
      double final_s = result_start.s + delta_s;
      if (final_s >= result_end.s) {
        return result_end;
      } else if (delta_s <= 0) {
        return result_start;
      } else {
        return GetNext(result_start, distance_to_segment.first).front();
      }
    } else {
      double delta_s = distance_to_segment.first;
      double final_s = result_start.s - delta_s;
      if (final_s <= result_end.s) {
        return result_end;
      } else if (delta_s <= 0) {
        return result_start;
      } else {
        return GetNext(result_start, distance_to_segment.first).front();
      }
    }
  }

  boost::optional<Waypoint> Map::GetWaypoint(
      const geom::Location &pos,
      uint32_t lane_type) const {
    boost::optional<Waypoint> w = GetClosestWaypointOnRoad(pos, lane_type);

    if (!w.has_value()) {
      return w;
    }

    const auto dist = geom::Math::Distance2D(ComputeTransform(*w).location, pos);
    const auto lane_width_info = GetLane(*w).GetInfo<RoadInfoLaneWidth>(w->s);
    const auto half_lane_width =
        lane_width_info->GetPolynomial().Evaluate(w->s) * 0.5;

    if (dist < half_lane_width) {
      return w;
    }

    return boost::optional<Waypoint>{};
  }

  boost::optional<Waypoint> Map::GetWaypoint(
      RoadId road_id,
      LaneId lane_id,
      float s) const {

    // define the waypoint with the known parameters
    Waypoint waypoint;
    waypoint.road_id = road_id;
    waypoint.lane_id = lane_id;
    waypoint.s = s;

    // check the road
    if (!_data.ContainsRoad(waypoint.road_id)) {
      return boost::optional<Waypoint>{};
    }
    const Road &road = _data.GetRoad(waypoint.road_id);

    // check the 's' distance
    if (s < 0.0f || s >= road.GetLength())
    {
      return boost::optional<Waypoint>{};
    }

    // check the section
    bool lane_found = false;
    for (auto &section : road.GetLaneSectionsAt(s)) {
      if (section.ContainsLane(lane_id)) {
        waypoint.section_id = section.GetId();
        lane_found = true;
        break;
      }
    }

    // check the lane id
    if (!lane_found) {
      return boost::optional<Waypoint>{};
    }

    return waypoint;
  }

  geom::Transform Map::ComputeTransform(Waypoint waypoint) const {
    const auto &road = _data.GetRoad(waypoint.road_id);

    // must s be smaller (or eq) than road lenght and bigger (or eq) than 0?
    RELEASE_ASSERT(waypoint.s <= road.GetLength());
    RELEASE_ASSERT(waypoint.s >= 0.0);

    const auto &lane_section = road.GetLaneSectionById(waypoint.section_id);
    const std::map<LaneId, Lane> &lanes = lane_section.GetLanes();

    // check that lane_id exists on the current s
    RELEASE_ASSERT(!lanes.empty());
    RELEASE_ASSERT(waypoint.lane_id >= lanes.begin()->first);
    RELEASE_ASSERT(waypoint.lane_id <= lanes.rbegin()->first);

    float lane_width = 0.0f;
    float lane_tangent = 0.0f;

    if (waypoint.lane_id < 0) {
      // right lane
      const auto side_lanes = MakeListView(
          std::make_reverse_iterator(lanes.lower_bound(0)), lanes.rend());
      const auto computed_width =
          ComputeTotalLaneWidth(side_lanes, waypoint.s, waypoint.lane_id);
      lane_width = static_cast<float>(computed_width.first);
      lane_tangent = static_cast<float>(computed_width.second);
    } else if (waypoint.lane_id > 0) {
      // left lane
      const auto side_lanes = MakeListView(lanes.lower_bound(1), lanes.end());
      const auto computed_width =
          ComputeTotalLaneWidth(side_lanes, waypoint.s, waypoint.lane_id);
      lane_width = static_cast<float>(computed_width.first);
      lane_tangent = static_cast<float>(computed_width.second);
    }

    // log_warning("Got to computetransform, road: ", road.GetId(), " ",
    // road.GetLength());
    // get a directed point in s and apply the computed lateral offet
    DirectedPoint dp = road.GetDirectedPointIn(waypoint.s);

    // compute the tangent of the laneOffset
    const auto lane_offset_info = road.GetInfo<RoadInfoLaneOffset>(waypoint.s);
    const auto lane_offset_tangent = static_cast<float>(lane_offset_info->GetPolynomial().Tangent(waypoint.s));

    lane_tangent -= lane_offset_tangent;

    // Unreal's Y axis hack
    lane_tangent *= -1;

    geom::Rotation rot(
        geom::Math::ToDegrees(static_cast<float>(dp.pitch)),
        geom::Math::ToDegrees(-static_cast<float>(dp.tangent)), // Unreal's Y axis hack
        0.0f);

    dp.ApplyLateralOffset(lane_width);

    if (waypoint.lane_id > 0) {
      rot.yaw += 180.0f + geom::Math::ToDegrees(lane_tangent);
      rot.pitch = 360.0f - rot.pitch;
    } else {
      rot.yaw -= geom::Math::ToDegrees(lane_tangent);
    }

    // Unreal's Y axis hack
    dp.location.y *= -1;

    return geom::Transform(dp.location, rot);
  }

  // ===========================================================================
  // -- Map: Road information --------------------------------------------------
  // ===========================================================================

  Lane::LaneType Map::GetLaneType(const Waypoint waypoint) const {
    return GetLane(waypoint).GetType();
  }

  double Map::GetLaneWidth(const Waypoint waypoint) const {
    const auto s = waypoint.s;

    const auto &lane = GetLane(waypoint);
    RELEASE_ASSERT(lane.GetRoad() != nullptr);
    RELEASE_ASSERT(s <= lane.GetRoad()->GetLength());

    const auto lane_width_info = lane.GetInfo<RoadInfoLaneWidth>(s);
    RELEASE_ASSERT(lane_width_info != nullptr);

    return lane_width_info->GetPolynomial().Evaluate(s);
  }

  JuncId Map::GetJunctionId(RoadId road_id) const {
    return _data.GetRoad(road_id).GetJunctionId();
  }

  bool Map::IsJunction(RoadId road_id) const {
    return _data.GetRoad(road_id).IsJunction();
  }

  std::pair<const RoadInfoMarkRecord *, const RoadInfoMarkRecord *>
  Map::GetMarkRecord(const Waypoint waypoint) const {
    // if lane Id is 0, just return a pair of nulls
    if (waypoint.lane_id == 0)
      return std::make_pair(nullptr, nullptr);

    const auto s = waypoint.s;

    const auto &current_lane = GetLane(waypoint);
    RELEASE_ASSERT(current_lane.GetRoad() != nullptr);
    RELEASE_ASSERT(s <= current_lane.GetRoad()->GetLength());

    const auto inner_lane_id = waypoint.lane_id < 0 ?
        waypoint.lane_id + 1 :
        waypoint.lane_id - 1;

    const auto &inner_lane = current_lane.GetRoad()->GetLaneById(waypoint.section_id, inner_lane_id);

    auto current_lane_info = current_lane.GetInfo<RoadInfoMarkRecord>(s);
    auto inner_lane_info = inner_lane.GetInfo<RoadInfoMarkRecord>(s);

    return std::make_pair(current_lane_info, inner_lane_info);
  }

  std::vector<LaneMarking> Map::CalculateCrossedLanes(
      const geom::Location &origin,
      const geom::Location &destination) const {
    return LaneCrossingCalculator::Calculate(*this, origin, destination);
  }

  std::vector<geom::Location> Map::GetAllCrosswalkZones() const {
    std::vector<geom::Location> result;

    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      std::vector<const RoadObjectCrosswalk *> crosswalks = road.GetObjects<RoadObjectCrosswalk>();
      if (crosswalks.size() > 0) {
        for (auto crosswalk : crosswalks) {
          // waypoint only at start position
          std::vector<geom::Location> points;
          Waypoint waypoint;
          geom::Transform base;
          for (const auto &section : road.GetLaneSectionsAt(crosswalk->GetS())) {
            // get the section with the center lane
            for (const auto &lane : section.GetLanes()) {
              // is the center line
              if (lane.first == 0) {
                // get the center point
                waypoint.road_id = pair.first;
                waypoint.section_id = section.GetId();
                waypoint.lane_id = 0;
                waypoint.s = crosswalk->GetS();
                base = ComputeTransform(waypoint);
              }
            }
          }

          // move perpendicular ('t')
          geom::Transform pivot = base;
          pivot.rotation.yaw -= geom::Math::ToDegrees<float>(static_cast<float>(crosswalk->GetHeading()));
          pivot.rotation.yaw -= 90;   // move perpendicular to 's' for the lateral offset
          geom::Vector3D v(static_cast<float>(crosswalk->GetT()), 0.0f, 0.0f);
          pivot.TransformPoint(v);
          // restore pivot position and orientation
          pivot = base;
          pivot.location = v;
          pivot.rotation.yaw -= geom::Math::ToDegrees<float>(static_cast<float>(crosswalk->GetHeading()));

          // calculate all the corners
          for (auto corner : crosswalk->GetPoints()) {
            geom::Vector3D v2(
                static_cast<float>(corner.u),
                static_cast<float>(corner.v),
                static_cast<float>(corner.z));
            // set the width larger to contact with the sidewalk (in case they have gutter area)
            if (corner.u < 0) {
              v2.x -= 1.0f;
            } else {
              v2.x += 1.0f;
            }
            pivot.TransformPoint(v2);
            result.push_back(v2);
          }
        }
      }
    }
    return result;
  }

  // ===========================================================================
  // -- Map: Waypoint generation -----------------------------------------------
  // ===========================================================================

  std::vector<Waypoint> Map::GetSuccessors(const Waypoint waypoint) const {
    const auto &next_lanes = GetLane(waypoint).GetNextLanes();
    std::vector<Waypoint> result;
    result.reserve(next_lanes.size());
    for (auto *next_lane : next_lanes) {
      RELEASE_ASSERT(next_lane != nullptr);
      const auto lane_id = next_lane->GetId();
      RELEASE_ASSERT(lane_id != 0);
      const auto *section = next_lane->GetLaneSection();
      RELEASE_ASSERT(section != nullptr);
      const auto *road = next_lane->GetRoad();
      RELEASE_ASSERT(road != nullptr);
      const auto distance = GetDistanceAtStartOfLane(*next_lane);
      result.emplace_back(Waypoint{road->GetId(), section->GetId(), lane_id, distance});
    }
    return result;
  }

  std::vector<Waypoint> Map::GetPredecessors(const Waypoint waypoint) const {
    const auto &prev_lanes = GetLane(waypoint).GetPreviousLanes();
    std::vector<Waypoint> result;
    result.reserve(prev_lanes.size());
    for (auto *next_lane : prev_lanes) {
      RELEASE_ASSERT(next_lane != nullptr);
      const auto lane_id = next_lane->GetId();
      RELEASE_ASSERT(lane_id != 0);
      const auto *section = next_lane->GetLaneSection();
      RELEASE_ASSERT(section != nullptr);
      const auto *road = next_lane->GetRoad();
      RELEASE_ASSERT(road != nullptr);
      const auto distance = GetDistanceAtEndOfLane(*next_lane);
      result.emplace_back(Waypoint{road->GetId(), section->GetId(), lane_id, distance});
    }
    return result;
  }

  std::vector<Waypoint> Map::GetNext(
      const Waypoint waypoint,
      const double distance) const {
    RELEASE_ASSERT(distance > 0.0);
    const auto &lane = GetLane(waypoint);
    const bool forward = (waypoint.lane_id <= 0);
    const double signed_distance = forward ? distance : -distance;
    const double relative_s = waypoint.s - lane.GetDistance() + EPSILON;
    const double remaining_lane_length = forward ? lane.GetLength() - relative_s : relative_s;
    DEBUG_ASSERT(remaining_lane_length >= 0.0);

    // If after subtracting the distance we are still in the same lane, return
    // same waypoint with the extra distance.
    if (distance <= remaining_lane_length) {
      Waypoint result = waypoint;
      result.s += signed_distance;
      result.s += forward ? -EPSILON : EPSILON;
      RELEASE_ASSERT(result.s > 0.0);
      return { result };
    }

    // If we run out of remaining_lane_length we have to go to the successors.
    std::vector<Waypoint> result;
    for (const auto &successor : GetSuccessors(waypoint)) {
      DEBUG_ASSERT(
          successor.road_id != waypoint.road_id ||
          successor.section_id != waypoint.section_id ||
          successor.lane_id != waypoint.lane_id);
      result = ConcatVectors(result, GetNext(successor, distance - remaining_lane_length));
    }
    return result;
  }

  std::vector<Waypoint> Map::GetPrevious(
      const Waypoint waypoint,
      const double distance) const {
    RELEASE_ASSERT(distance > 0.0);
    const auto &lane = GetLane(waypoint);
    const bool forward = !(waypoint.lane_id <= 0);
    const double signed_distance = forward ? distance : -distance;
    const double relative_s = waypoint.s - lane.GetDistance() + EPSILON;
    const double remaining_lane_length = forward ? lane.GetLength() - relative_s : relative_s;
    DEBUG_ASSERT(remaining_lane_length >= 0.0);

    // If after subtracting the distance we are still in the same lane, return
    // same waypoint with the extra distance.
    if (distance <= remaining_lane_length) {
      Waypoint result = waypoint;
      result.s += signed_distance;
      result.s += forward ? -EPSILON : EPSILON;
      RELEASE_ASSERT(result.s > 0.0);
      return { result };
    }

    // If we run out of remaining_lane_length we have to go to the successors.
    std::vector<Waypoint> result;
    for (const auto &successor : GetPredecessors(waypoint)) {
      DEBUG_ASSERT(
          successor.road_id != waypoint.road_id ||
          successor.section_id != waypoint.section_id ||
          successor.lane_id != waypoint.lane_id);
      result = ConcatVectors(result, GetPrevious(successor, distance - remaining_lane_length));
    }
    return result;
  }

  boost::optional<Waypoint> Map::GetRight(Waypoint waypoint) const {
    RELEASE_ASSERT(waypoint.lane_id != 0);
    if (waypoint.lane_id > 0) {
      ++waypoint.lane_id;
    } else {
      --waypoint.lane_id;
    }
    return IsLanePresent(_data, waypoint) ? waypoint : boost::optional<Waypoint>{};
  }

  boost::optional<Waypoint> Map::GetLeft(Waypoint waypoint) const {
    RELEASE_ASSERT(waypoint.lane_id != 0);
    if (std::abs(waypoint.lane_id) == 1) {
      waypoint.lane_id *= -1;
    } else if (waypoint.lane_id > 0) {
      --waypoint.lane_id;
    } else {
      ++waypoint.lane_id;
    }
    return IsLanePresent(_data, waypoint) ? waypoint : boost::optional<Waypoint>{};
  }

  std::vector<Waypoint> Map::GenerateWaypoints(const double distance) const {
    RELEASE_ASSERT(distance > 0.0);
    std::vector<Waypoint> result;
    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      for (double s = EPSILON; s < (road.GetLength() - EPSILON); s += distance) {
        ForEachDrivableLaneAt(road, s, [&](auto &&waypoint) {
          result.emplace_back(waypoint);
        });
      }
    }
    return result;
  }

  std::vector<Waypoint> Map::GenerateWaypointsOnRoadEntries(Lane::LaneType lane_type) const {
    std::vector<Waypoint> result;
    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      // right lanes start at s 0
      for (const auto &lane_section : road.GetLaneSectionsAt(0.0)) {
        for (const auto &lane : lane_section.GetLanes()) {
          // add only the right (negative) lanes
          if (lane.first < 0 &&
              static_cast<uint32_t>(lane.second.GetType()) & static_cast<uint32_t>(lane_type)) {
            result.emplace_back(Waypoint{ road.GetId(), lane_section.GetId(), lane.second.GetId(), 0.0 });
          }
        }
      }
      // left lanes start at s max
      const auto road_len = road.GetLength();
      for (const auto &lane_section : road.GetLaneSectionsAt(road_len)) {
        for (const auto &lane : lane_section.GetLanes()) {
          // add only the left (positive) lanes
          if (lane.first > 0 &&
              static_cast<uint32_t>(lane.second.GetType()) & static_cast<uint32_t>(lane_type)) {
            result.emplace_back(
              Waypoint{ road.GetId(), lane_section.GetId(), lane.second.GetId(), road_len });
          }
        }
      }
    }
    return result;
  }

  std::vector<std::pair<Waypoint, Waypoint>> Map::GenerateTopology() const {
    std::vector<std::pair<Waypoint, Waypoint>> result;
    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      ForEachDrivableLane(road, [&](auto &&waypoint) {
        for (auto &&successor : GetSuccessors(waypoint)) {
          result.push_back({waypoint, successor});
        }
      });
    }
    return result;
  }

  std::vector<std::pair<Waypoint, Waypoint>> Map::GetJunctionWaypoints(JuncId id, Lane::LaneType lane_type) const {
    std::vector<std::pair<Waypoint, Waypoint>> result;
    const Junction * junction = GetJunction(id);
    for(auto &connections : junction->GetConnections()) {
      const Road &road = _data.GetRoad(connections.second.connecting_road);
      ForEachLane(road, lane_type, [&](auto &&waypoint) {
        const auto& lane = GetLane(waypoint);
        const double final_s = GetDistanceAtEndOfLane(lane);
        Waypoint lane_end(waypoint);
        lane_end.s = final_s;
        result.push_back({waypoint, lane_end});
      });
    }
    return result;
  }

  // ===========================================================================
  // -- Map: Private functions -------------------------------------------------
  // ===========================================================================

  const Lane &Map::GetLane(Waypoint waypoint) const {
    return _data.GetRoad(waypoint.road_id).GetLaneById(waypoint.section_id, waypoint.lane_id);
  }

  // Checks whether the geometry is straight or not
  bool IsLineStraight(const Road &road, const Lane &lane, element::GeometryType geometry_type) {
    if (geometry_type != element::GeometryType::LINE) {
      return false;
    }
    auto lane_offsets = lane.GetInfos<element::RoadInfoLaneOffset>();
    for (auto *lane_offset : lane_offsets) {
      if (abs(lane_offset->GetPolynomial().GetC()) > 0 ||
      abs(lane_offset->GetPolynomial().GetD()) > 0) {
        return false;
      }
    }
    auto elevations = road.GetInfos<element::RoadInfoElevation>();
    for (auto *elevation : elevations) {
      if (abs(elevation->GetPolynomial().GetC()) > 0 ||
      abs(elevation->GetPolynomial().GetD()) > 0) {
        return false;
      }
    }
    return true;
  }

  // Adds a new element to the rtree element list using the position of the
  // waypoints
  // both ends of the segment
  void Map::AddElementToRtree(
      std::vector<Rtree::TreeElement> &rtree_elements,
      geom::Transform &current_transform,
      geom::Transform &next_transform,
      Waypoint &current_waypoint,
      Waypoint &next_waypoint) {
    Rtree::BPoint init =
        Rtree::BPoint(
        current_transform.location.x,
        current_transform.location.y,
        current_transform.location.z);
    Rtree::BPoint end =
        Rtree::BPoint(
        next_transform.location.x,
        next_transform.location.y,
        next_transform.location.z);
    rtree_elements.emplace_back(std::make_pair(Rtree::BSegment(init, end),
        std::make_pair(current_waypoint, next_waypoint)));
  }
  // Adds a new element to the rtree element list using the position of the
  // waypoints, both ends of the segment
  void Map::AddElementToRtreeAndUpdateTransforms(
      std::vector<Rtree::TreeElement> &rtree_elements,
      geom::Transform &current_transform,
      Waypoint &current_waypoint,
      Waypoint &next_waypoint) {
    geom::Transform next_transform = ComputeTransform(next_waypoint);
    AddElementToRtree(rtree_elements, current_transform, next_transform,
    current_waypoint, next_waypoint);
    current_waypoint = next_waypoint;
    current_transform = next_transform;
  }

  // returns the remaining length of the geometry depending on the lane
  // direction
  double GetRemainingLength(
      const Lane &lane,
      double geometry_start_s,
      double geometry_end_s,
      double current_s) {
    if (lane.GetId() < 0) {
      return (geometry_end_s - current_s);
    } else {
      return (current_s - geometry_start_s);
    }
  }

  void Map::CreateRtree() {
    const double epsilon = 0.000001; // small delta in the road (set to 1
                                     // micrometer to prevent numeric errors)
    const double min_delta_s = 1;    // segments of minimum 1m through the road

    // 1.8 degrees, maximum angle in a curve to place a segment
    constexpr double angle_threshold = geom::Math::Pi<double>() / 100.0;

    // Generate waypoints at start of every road and for everey lane
    std::vector<Waypoint> topology = GenerateWaypointsOnRoadEntries(Lane::LaneType::Any);

    std::vector<Rtree::TreeElement> rtree_elements; // container of segments and
                                                    // waypoints
    // Loop through all road lanes
    for (auto &waypoint : topology) {
      auto &lane_start_waypoint = waypoint;

      auto current_waypoint = lane_start_waypoint;

      const Road &road = _data.GetRoad(current_waypoint.road_id);

      // loop through every geometry in the road
      while (current_waypoint.s <= road.GetLength() && current_waypoint.s >= 0) {
        const Lane &lane = GetLane(current_waypoint);
        const auto *geometry = road.GetInfo<element::RoadInfoGeometry>(current_waypoint.s);
        // start s of this geometry in the road
        double geometry_start_s = geometry->GetGeometry().GetStartOffset();
        // end s this geometry in the road
        double geometry_end_s = geometry_start_s + geometry->GetGeometry().GetLength();

        geom::Transform current_transform = ComputeTransform(current_waypoint);

        // Save computation time in straight lines
        if (IsLineStraight(road, lane, geometry->GetGeometry().GetType())) {
          double delta_s = min_delta_s;
          double remaining_length =
              GetRemainingLength(lane, geometry_start_s, geometry_end_s, current_waypoint.s);
          remaining_length -= epsilon;
          delta_s = remaining_length;
          if (delta_s < epsilon) {
            break;
          }
          auto next = GetNext(current_waypoint, delta_s);
          RELEASE_ASSERT(next.size() == 1);
          RELEASE_ASSERT(next.front().road_id == current_waypoint.road_id);
          auto next_waypoint = next.front();

          AddElementToRtreeAndUpdateTransforms(
              rtree_elements,
              current_transform,
              current_waypoint,
              next_waypoint);
        } else {
          auto next_waypoint = current_waypoint;
          // Loop until the end of the geometry
          while (true) {
            double delta_s = min_delta_s;
            double remaining_length =
                GetRemainingLength(
                    lane,
                    geometry_start_s,
                    geometry_end_s,
                    next_waypoint.s);
            remaining_length -= epsilon;
            delta_s = std::min(delta_s, remaining_length);
            if (delta_s < epsilon) {
              AddElementToRtreeAndUpdateTransforms(
                  rtree_elements,
                  current_transform,
                  current_waypoint,
                  next_waypoint);
              break;
            }

            auto next = GetNext(next_waypoint, delta_s);
            if (next.size() != 1 ||
            current_waypoint.road_id != next.front().road_id) {
              AddElementToRtreeAndUpdateTransforms(
                  rtree_elements,
                  current_transform,
                  current_waypoint,
                  next_waypoint);
              break;
            }
            next_waypoint = next.front();

            geom::Transform next_transform = ComputeTransform(next_waypoint);
            double angle = geom::Math::GetVectorAngle(
                current_transform.GetForwardVector(), next_transform.GetForwardVector());

            if (abs(angle) > angle_threshold) {
              AddElementToRtree(
                  rtree_elements,
                  current_transform,
                  next_transform,
                  current_waypoint,
                  next_waypoint);
              current_waypoint = next_waypoint;
              current_transform = next_transform;
            }
          }
        }
        auto next = GetNext(current_waypoint, 10.0 * epsilon);
        if (next.size() != 1 || next.front().road_id != current_waypoint.road_id) {
          break;
        } else {
          current_waypoint = next.front();
        }
      }
    }
    _rtree.InsertElements(rtree_elements);
  }

  Junction* Map::GetJunction(JuncId id) {
    return _data.GetJunction(id);
  }

  const Junction* Map::GetJunction(JuncId id) const {
    return _data.GetJunction(id);
  }

} // namespace road
} // namespace carla
