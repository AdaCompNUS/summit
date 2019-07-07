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

  FRoutePoint GetNearestRoutePoint(const FVector2D& Position) const override;

  TArray<FRoutePoint> GetNextRoutePoints(const FRoutePoint& RoutePoint, float LookaheadDistance) const override;

private:
  
  typedef std::pair<bool, long long> network_segment;
  typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point;
  typedef boost::geometry::model::segment<rt_point> rt_segment;
  typedef std::pair<rt_segment, int> rt_value;
  typedef boost::geometry::index::rtree<rt_value, boost::geometry::index::rstar<16> > rt_tree;

  const FLaneNetwork* LaneNetwork;

  TArray<network_segment> Segments;
  TMap<long long, int> LaneIDToSegmentIDMap;
  TMap<long long, int> LaneConnectionIDToSegmentIDMap;
  rt_tree SegmentsIndex;
  
  static FVector2D ToUE2D(const FVector2D& Position) { 
    return 100.0f * FVector2D(Position.Y, Position.X);
  }

  static FVector2D ToNetwork(const FVector2D& Position) {
    return FVector2D(Position.Y, Position.X) / 100.0f;
  }

  static float ToUE(float Distance) {
    return 100.0f;
  }

  static float ToNetwork(float Distance) {
    return Distance / 100.0f;
  }
};
