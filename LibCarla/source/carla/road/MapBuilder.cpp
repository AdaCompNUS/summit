// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/StringUtil.h"
#include "carla/road/MapBuilder.h"
#include "carla/road/element/RoadInfoElevation.h"
#include "carla/road/element/RoadInfoGeometry.h"
#include "carla/road/element/RoadInfoLaneAccess.h"
#include "carla/road/element/RoadInfoLaneBorder.h"
#include "carla/road/element/RoadInfoLaneHeight.h"
#include "carla/road/element/RoadInfoLaneMaterial.h"
#include "carla/road/element/RoadInfoLaneOffset.h"
#include "carla/road/element/RoadInfoLaneRule.h"
#include "carla/road/element/RoadInfoLaneVisibility.h"
#include "carla/road/element/RoadInfoLaneWidth.h"
#include "carla/road/element/RoadInfoMarkRecord.h"
#include "carla/road/element/RoadInfoMarkTypeLine.h"
#include "carla/road/element/RoadInfoSpeed.h"
#include "carla/road/element/RoadInfoVisitor.h"
#include "carla/road/InformationSet.h"
#include "carla/road/general/Validity.h"
#include "carla/road/signal/Signal.h"
#include "carla/road/signal/SignalReference.h"
#include "carla/road/signal/SignalDependency.h"

#include <iterator>
#include <memory>

using namespace carla::road::element;

namespace carla {
namespace road {

  boost::optional<Map> MapBuilder::Build() {

    CreatePointersBetweenRoadSegments();

    for (auto &&info : _temp_road_info_container) {
      DEBUG_ASSERT(info.first != nullptr);
      info.first->_info = InformationSet(std::move(info.second));
    }

    for (auto &&info : _temp_lane_info_container) {
      DEBUG_ASSERT(info.first != nullptr);
      info.first->_info = InformationSet(std::move(info.second));
    }

    // remove temporal already used information
    _temp_road_info_container.clear();
    _temp_lane_info_container.clear();

    // _map_data is a memeber of MapBuilder so you must especify if
    // you want to keep it (will return copy -> Map(const Map &))
    // or move it (will return move -> Map(Map &&))
    return Map{std::move(_map_data)};
  }

  // called from profiles parser
  void MapBuilder::AddRoadElevationProfile(
      Road *road,
      const double s,
      const double a,
      const double b,
      const double c,
      const double d) {
    DEBUG_ASSERT(road != nullptr);
    auto elevation = std::make_unique<RoadInfoElevation>(s, a, b, c, d);
    _temp_road_info_container[road].emplace_back(std::move(elevation));
  }

  // called from lane parser
  void MapBuilder::CreateLaneAccess(
      Lane *lane,
      const double s,
      const std::string restriction) {
    DEBUG_ASSERT(lane != nullptr);
    _temp_lane_info_container[lane].emplace_back(std::make_unique<RoadInfoLaneAccess>(s, restriction));
  }

  void MapBuilder::CreateLaneBorder(
      Lane *lane,
      const double s,
      const double a,
      const double b,
      const double c,
      const double d) {
    DEBUG_ASSERT(lane != nullptr);
    _temp_lane_info_container[lane].emplace_back(std::make_unique<RoadInfoLaneBorder>(s, a, b, c, d));
  }

  void MapBuilder::CreateLaneHeight(
      Lane *lane,
      const double s,
      const double inner,
      const double outer) {
    DEBUG_ASSERT(lane != nullptr);
    _temp_lane_info_container[lane].emplace_back(std::make_unique<RoadInfoLaneHeight>(s, inner, outer));
  }

  void MapBuilder::CreateLaneMaterial(
      Lane *lane,
      const double s,
      const std::string surface,
      const double friction,
      const double roughness) {
    DEBUG_ASSERT(lane != nullptr);
    _temp_lane_info_container[lane].emplace_back(std::make_unique<RoadInfoLaneMaterial>(s, surface, friction,
        roughness));
  }

  void MapBuilder::CreateLaneRule(
      Lane *lane,
      const double s,
      const std::string value) {
    DEBUG_ASSERT(lane != nullptr);
    _temp_lane_info_container[lane].emplace_back(std::make_unique<RoadInfoLaneRule>(s, value));
  }

  void MapBuilder::CreateLaneVisibility(
      Lane *lane,
      const double s,
      const double forward,
      const double back,
      const double left,
      const double right) {
    DEBUG_ASSERT(lane != nullptr);
    _temp_lane_info_container[lane].emplace_back(std::make_unique<RoadInfoLaneVisibility>(s, forward, back,
        left, right));
  }

  void MapBuilder::CreateLaneWidth(
      Lane *lane,
      const double s,
      const double a,
      const double b,
      const double c,
      const double d) {
    DEBUG_ASSERT(lane != nullptr);
    _temp_lane_info_container[lane].emplace_back(std::make_unique<RoadInfoLaneWidth>(s, a, b, c, d));
  }

  void MapBuilder::CreateRoadMark(
      Lane *lane,
      const int road_mark_id,
      const double s,
      const std::string type,
      const std::string weight,
      const std::string color,
      const std::string material,
      const double width,
      std::string lane_change,
      const double height,
      const std::string type_name,
      const double type_width) {
    DEBUG_ASSERT(lane != nullptr);
    RoadInfoMarkRecord::LaneChange lc;

    StringUtil::ToLower(lane_change);

    if (lane_change == "increase") {
      lc = RoadInfoMarkRecord::LaneChange::Increase;
    } else if (lane_change == "decrease") {
      lc = RoadInfoMarkRecord::LaneChange::Decrease;
    } else if (lane_change == "both") {
      lc = RoadInfoMarkRecord::LaneChange::Both;
    } else {
      lc = RoadInfoMarkRecord::LaneChange::None;
    }
    _temp_lane_info_container[lane].emplace_back(std::make_unique<RoadInfoMarkRecord>(s, road_mark_id, type,
        weight, color,
        material, width, lc, height, type_name, type_width));
  }

  void MapBuilder::CreateRoadMarkTypeLine(
      Lane *lane,
      const int road_mark_id,
      const double length,
      const double space,
      const double tOffset,
      const double s,
      const std::string rule,
      const double width) {
    DEBUG_ASSERT(lane != nullptr);
    auto it = MakeRoadInfoIterator<RoadInfoMarkRecord>(_temp_lane_info_container[lane]);
    for (; !it.IsAtEnd(); ++it) {
      if (it->GetRoadMarkId() == road_mark_id) {
        it->GetLines().emplace_back(std::make_unique<RoadInfoMarkTypeLine>(s, road_mark_id, length, space,
            tOffset, rule, width));
        break;
      }
    }

  }

  void MapBuilder::CreateLaneSpeed(
      Lane *lane,
      const double s,
      const double max,
      const std::string /*unit*/) {
    DEBUG_ASSERT(lane != nullptr);
    _temp_lane_info_container[lane].emplace_back(std::make_unique<RoadInfoSpeed>(s, max));
  }

  void MapBuilder::AddSignal(
      const uint32_t road_id,
      const uint32_t signal_id,
      const double s,
      const double t,
      const std::string name,
      const std::string dynamic,
      const std::string orientation,
      const double zOffset,
      const std::string country,
      const std::string type,
      const std::string subtype,
      const double value,
      const std::string unit,
      const double height,
      const double width,
      const std::string text,
      const double hOffset,
      const double pitch,
      const double roll) {
    auto signals = _map_data.GetRoad(road_id).getSignals();
    DEBUG_ASSERT(signals != nullptr);
    signals->emplace(signal_id,
        signal::Signal(road_id, signal_id, s, t, name, dynamic,
        orientation, zOffset, country, type, subtype, value, unit, height, width,
        text, hOffset, pitch, roll));
  }

  void MapBuilder::AddValidityToLastAddedSignal(
      const uint32_t road_id,
      const uint32_t signal_id,
      const int32_t from_lane,
      const int32_t to_lane) {
    _map_data.GetRoad(road_id).GetSignal(signal_id)->AddValidity(general::Validity(signal_id, from_lane,
        to_lane));
  }

  // build road objects
  carla::road::Road *MapBuilder::AddRoad(
      const RoadId road_id,
      const std::string name,
      const double length,
      const JuncId junction_id,
      const RoadId predecessor,
      const RoadId successor) {

    // add it
    auto road = &(_map_data._roads.emplace(road_id, Road()).first->second);

    // set road data
    road->_map_data = &_map_data;
    road->_id = road_id;
    road->_name = name;
    road->_length = length;
    road->_junction_id = junction_id;
    (junction_id != -1) ? road->_is_junction = true : road->_is_junction = false;
    road->_successor = successor;
    road->_predecessor = predecessor;

    return road;
  }

  carla::road::LaneSection *MapBuilder::AddRoadSection(
      Road *road,
      const SectionId id,
      const double s) {
    DEBUG_ASSERT(road != nullptr);
    carla::road::LaneSection &sec = road->_lane_sections.Emplace(id, s);
    sec._road = road;
    return &sec;
  }

  carla::road::Lane *MapBuilder::AddRoadSectionLane(
      carla::road::LaneSection *section,
      const int32_t lane_id,
      const uint32_t lane_type,
      const bool lane_level,
      const int32_t predecessor,
      const int32_t successor) {
    DEBUG_ASSERT(section != nullptr);

    // add the lane
    auto *lane = &((section->_lanes.emplace(lane_id, Lane()).first)->second);

    // set lane data
    lane->_id = lane_id;
    lane->_lane_section = section;
    lane->_level = lane_level;
    lane->_type = static_cast<carla::road::Lane::LaneType>(lane_type);
    lane->_successor = successor;
    lane->_predecessor = predecessor;

    return lane;
  }

  void MapBuilder::AddRoadGeometryLine(
      Road *road,
      const double s,
      const double x,
      const double y,
      const double hdg,
      const double length) {
    DEBUG_ASSERT(road != nullptr);
    const geom::Location location(static_cast<float>(x), static_cast<float>(y), 0.0f);
    auto line_geometry = std::make_unique<GeometryLine>(
        s,
        length,
        hdg,
        location);

    _temp_road_info_container[road].emplace_back(std::unique_ptr<RoadInfo>(new RoadInfoGeometry(s,
        std::move(line_geometry))));
  }

  void MapBuilder::CreateRoadSpeed(
      Road *road,
      const double s,
      const std::string /*type*/,
      const double max,
      const std::string /*unit*/) {
    DEBUG_ASSERT(road != nullptr);
    _temp_road_info_container[road].emplace_back(std::make_unique<RoadInfoSpeed>(s, max));
  }

  void MapBuilder::CreateSectionOffset(
      Road *road,
      const double s,
      const double a,
      const double b,
      const double c,
      const double d) {
    DEBUG_ASSERT(road != nullptr);
    _temp_road_info_container[road].emplace_back(std::make_unique<RoadInfoLaneOffset>(s, a, b, c, d));
  }

  void MapBuilder::AddRoadGeometryArc(
      Road *road,
      const double s,
      const double x,
      const double y,
      const double hdg,
      const double length,
      const double curvature) {
    DEBUG_ASSERT(road != nullptr);
    const geom::Location location(static_cast<float>(x), static_cast<float>(y), 0.0f);
    auto arc_geometry = std::make_unique<GeometryArc>(
        s,
        length,
        hdg,
        location,
        curvature);

    _temp_road_info_container[road].emplace_back(std::unique_ptr<RoadInfo>(new RoadInfoGeometry(s,
        std::move(arc_geometry))));
  }

  void MapBuilder::AddRoadGeometrySpiral(
      carla::road::Road * /*road*/,
      const double /*s*/,
      const double /*x*/,
      const double /*y*/,
      const double /*hdg*/,
      const double /*length*/,
      const double /*curvStart*/,
      const double /*curvEnd*/) {
    throw_exception(std::runtime_error("geometry spiral not supported"));
  }

  void MapBuilder::AddRoadGeometryPoly3(
      carla::road::Road * /*road*/,
      const double /*s*/,
      const double /*x*/,
      const double /*y*/,
      const double /*hdg*/,
      const double /*length*/,
      const double /*a*/,
      const double /*b*/,
      const double /*c*/,
      const double /*d*/) {
    throw_exception(std::runtime_error("geometry poly3 not supported"));
  }

  void MapBuilder::AddRoadGeometryParamPoly3(
      carla::road::Road * /*road*/,
      const double /*s*/,
      const double /*x*/,
      const double /*y*/,
      const double /*hdg*/,
      const double /*length*/,
      const double /*aU*/,
      const double /*bU*/,
      const double /*cU*/,
      const double /*dU*/,
      const double /*aV*/,
      const double /*bV*/,
      const double /*cV*/,
      const double /*dV*/,
      const std::string /*p_range*/) {
    throw_exception(std::runtime_error("geometry poly3 not supported"));
  }

  void MapBuilder::AddJunction(const int32_t id, const std::string name) {
    _map_data.GetJunctions().emplace(id, Junction(id, name));
  }

  void MapBuilder::AddConnection(
      const JuncId junction_id,
      const ConId connection_id,
      const RoadId incoming_road,
      const RoadId connecting_road) {
    DEBUG_ASSERT(_map_data.GetJunction(junction_id) != nullptr);
    _map_data.GetJunction(junction_id)->GetConnections().emplace(connection_id,
        Junction::Connection(connection_id, incoming_road, connecting_road));
  }

  void MapBuilder::AddLaneLink(
      const JuncId junction_id,
      const ConId connection_id,
      const LaneId from,
      const LaneId to) {
    DEBUG_ASSERT(_map_data.GetJunction(junction_id) != nullptr);
    _map_data.GetJunction(junction_id)->GetConnection(connection_id)->AddLaneLink(from, to);
  }

  void MapBuilder::AddValidityToSignal(
      const uint32_t road_id,
      const uint32_t signal_id,
      const int32_t from_lane,
      const int32_t to_lane) {
    DEBUG_ASSERT(_map_data.GetRoad(road_id).GetSignal(signal_id) != nullptr);
    _map_data.GetRoad(road_id).GetSignal(signal_id)->AddValidity(general::Validity(signal_id, from_lane,
        to_lane));
  }

  void MapBuilder::AddValidityToSignalReference(
      const uint32_t road_id,
      const uint32_t signal_reference_id,
      const int32_t from_lane,
      const int32_t to_lane) {
    DEBUG_ASSERT(_map_data.GetRoad(road_id).GetSignalRef(signal_reference_id) != nullptr);
    _map_data.GetRoad(road_id).GetSignalRef(signal_reference_id)->AddValidity(general::Validity(
        signal_reference_id, from_lane, to_lane));
  }

  void MapBuilder::AddSignalReference(
      const uint32_t road_id,
      const uint32_t signal_reference_id,
      const double s_position,
      const double t_position,
      const std::string signal_reference_orientation) {
    DEBUG_ASSERT(_map_data.GetRoad(road_id).getSignalReferences() != nullptr);
    _map_data.GetRoad(road_id).getSignalReferences()->emplace(signal_reference_id,
        signal::SignalReference(road_id, signal_reference_id, s_position, t_position,
        signal_reference_orientation));
  }

  void MapBuilder::AddDependencyToSignal(
      const RoadId road_id,
      const SignId signal_id,
      const uint32_t dependency_id,
      const std::string dependency_type) {
    DEBUG_ASSERT(_map_data.GetRoad(road_id).GetSignal(signal_id) != nullptr);
    _map_data.GetRoad(road_id).GetSignal(signal_id)->AddDependency(signal::SignalDependency(
        road_id,
        signal_id,
        dependency_id,
        dependency_type));
  }

  Lane *MapBuilder::GetLane(
      const RoadId road_id,
      const LaneId lane_id,
      const double s) {
    return &_map_data.GetRoad(road_id).GetLaneByDistance(s, lane_id);
  }

  Road *MapBuilder::GetRoad(
      const RoadId road_id) {
    return &_map_data.GetRoad(road_id);
  }

  // return the pointer to a lane object
  Lane *MapBuilder::GetEdgeLanePointer(RoadId road_id, bool from_start, LaneId lane_id) {

    if (!_map_data.ContainsRoad(road_id)) {
      return nullptr;
    }
    Road &road = _map_data.GetRoad(road_id);

    // get the lane section
    LaneSection *section;
    if (from_start) {
      section = road.GetStartSection(lane_id);
    } else {
      section = road.GetEndSection(lane_id);
    }

    // get the lane
    DEBUG_ASSERT(section != nullptr);
    return section->GetLane(lane_id);
  }

  // return a list of pointers to all lanes from a lane (using road and junction
  // info)
  std::vector<Lane *> MapBuilder::GetLaneNext(
      RoadId road_id,
      SectionId section_id,
      LaneId lane_id) {
    std::vector<Lane *> result;

    if (!_map_data.ContainsRoad(road_id)) {
      return result;
    }
    Road &road = _map_data.GetRoad(road_id);

    // get the section
    LaneSection &section = road._lane_sections.GetById(section_id);

    // get the lane
    Lane *lane = section.GetLane(lane_id);
    DEBUG_ASSERT(lane != nullptr);

    // successor and predecessor (road and lane)
    LaneId next;
    RoadId next_road;
    if (lane_id <= 0) {
      next_road = road.GetSuccessor();
      next = lane->GetSuccessor();
    } else {
      next_road = road.GetPredecessor();
      next = lane->GetPredecessor();
    }

    // check to see if next is a road or a junction
    bool next_is_junction = !_map_data.ContainsRoad(next_road);
    double s = section.GetDistance();

    // check if we are in a lane section in the middle
    if ((lane_id > 0 && s > 0) ||
        (lane_id <= 0 && road._lane_sections.upper_bound(s) != road._lane_sections.end())) {
      // check if lane has a next link (if not, it deads in the middle section)
      if (next != 0 || (lane_id == 0 && next == 0)) {
        // change to next / prev section
        if (lane_id <= 0) {
          result.push_back(road.GetNextLane(s, next));
        } else {
          result.push_back(road.GetPrevLane(s, next));
        }
      }
    } else if (!next_is_junction) {
      // change to another road / junction
      if (next != 0 || (lane_id == 0 && next == 0)) {
        // single road
        result.push_back(GetEdgeLanePointer(next_road, (next <= 0), next));
      }
    } else {
      // several roads (junction)

      /// @todo Is it correct to use a road id as section id? (NS: I just added
      /// this cast to avoid compiler warnings).
      auto next_road_as_junction = static_cast<JuncId>(next_road);
      auto options = GetJunctionLanes(next_road_as_junction, road_id, lane_id);
      for (auto opt : options) {
        result.push_back(GetEdgeLanePointer(opt.first, (opt.second <= 0), opt.second));
      }
    }

    return result;
  }

  std::vector<std::pair<RoadId, LaneId>> MapBuilder::GetJunctionLanes(
      JuncId junction_id,
      RoadId road_id,
      LaneId lane_id) {
    std::vector<std::pair<RoadId, LaneId>> result;

    // get the junction
    Junction *junction = _map_data.GetJunction(junction_id);
    if (junction == nullptr) {
      return result;
    }

    // check all connections
    for (auto con : junction->_connections) {
      // only connections for our road
      if (con.second.incoming_road == road_id) {
        // for center lane it is always next lane id 0, we don't need to search
        // because it is not in the junction
        if (lane_id == 0) {
          result.push_back(std::make_pair(con.second.connecting_road, 0));
        } else {
          // check all lane links
          for (auto link : con.second.lane_links) {
            // is our lane id ?
            if (link.from == lane_id) {
              // add as option
              result.push_back(std::make_pair(con.second.connecting_road, link.to));
            }
          }
        }
      }
    }

    return result;
  }

  // assign pointers to the next lanes
  void MapBuilder::CreatePointersBetweenRoadSegments(void) {
    // process each lane to define its nexts
    for (auto &road : _map_data._roads) {
      for (auto &section : road.second._lane_sections) {
        for (auto &lane : section.second._lanes) {

          // assign the next lane pointers
          lane.second._next_lanes = GetLaneNext(road.first, section.second._id, lane.first);

          // add to each lane found, this as its predecessor
          for (auto next_lane : lane.second._next_lanes) {
            // add as previous
            DEBUG_ASSERT(next_lane != nullptr);
            next_lane->_prev_lanes.push_back(&lane.second);
          }

        }
      }
    }

    // process each lane to define its nexts
    for (auto &road : _map_data._roads) {
      for (auto &section : road.second._lane_sections) {
        for (auto &lane : section.second._lanes) {

          // add next roads
          for (auto next_lane : lane.second._next_lanes) {
            DEBUG_ASSERT(next_lane != nullptr);
            // avoid same road
            if (next_lane->GetRoad() != &road.second) {
              if (std::find(road.second._nexts.begin(), road.second._nexts.end(),
                  next_lane->GetRoad()) == road.second._nexts.end()) {
                road.second._nexts.push_back(next_lane->GetRoad());
              }
            }
          }

          // add prev roads
          for (auto prev_lane : lane.second._prev_lanes) {
            DEBUG_ASSERT(prev_lane != nullptr);
            // avoid same road
            if (prev_lane->GetRoad() != &road.second) {
              if (std::find(road.second._prevs.begin(), road.second._prevs.end(),
                  prev_lane->GetRoad()) == road.second._prevs.end()) {
                road.second._prevs.push_back(prev_lane->GetRoad());
              }
            }
          }

        }
      }
    }
  }

} // namespace road
} // namespace carla
