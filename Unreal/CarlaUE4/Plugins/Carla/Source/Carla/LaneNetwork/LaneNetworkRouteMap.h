#pragma once

#include "RouteMap/RouteMap.h"
#include <compiler/disable-ue4-macros.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <compiler/enable-ue4-macros.h>

class FLaneNetworkRouteMap : public FRouteMap {
public:

  FLaneNetworkRouteMap() { }

  FLaneNetworkRouteMap(const FLaneNetwork* LaneNetwork);

  FRoutePoint GetNearestRoutePoint(const FVector2D& Position) override;

  TArray<FRoutePoint> GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) override;

private:
  
  typedef std::pair<bool, long long> lane_segment;
  typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point;
  typedef boost::geometry::model::segment<rt_point> rt_segment;
  typedef std::pair<rt_segment, int> rt_value;
  typedef boost::geometry::index::rtree<rt_value, boost::geometry::index::rstar<16> > rt_tree;

  const FLaneNetwork* LaneNetwork;

  TArray<lane_segment> Segments;
  rt_tree LanesIndex;
};
