#pragma once

#include "OccupancyTriangle.h"
#include "OccupancyGrid.h"
#include "OccupancyArea.h"

#include <compiler/disable-ue4-macros.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <compiler/enable-ue4-macros.h>

typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> rt_point;
typedef boost::geometry::model::box<rt_point> rt_box;
typedef std::pair<rt_box, int> rt_value;
typedef bgi::rtree<rt_value, bgi::rstar<16>> rt_tree;

class FOccupancyMap {

public:

  FOccupancyMap() : Area(0) { }
  FOccupancyMap(const TArray<FOccupancyTriangle>& OccupancyTriangles);

  // TODO Optimize using AABB trees.
  FVector2D RandPoint() const;

  FOccupancyArea GetOccupancyArea(const FBox2D& Bounds, float Resolution, int OffroadPolygonEdgeInterval) const;

private:

  TArray<FOccupancyTriangle> OccupancyTriangles;
  rt_tree OccupancyTrianglesIndex;
  float Area;

};
