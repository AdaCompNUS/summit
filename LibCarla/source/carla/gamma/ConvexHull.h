/* 
 * Author: Yuanfu Luo <yuanfu@comp.nus.edu.sg>
 */

#include <algorithm>
#include "Definitions.h"
#include "RVOSimulator.h"

namespace RVO {

	// Returns a new list of points representing the convex hull of
	// the given set of points. The convex hull excludes collinear points.
	// This algorithm runs in O(n log n) time.
	std::vector<Vector2> makeConvexHull(const std::vector<Vector2> &points);


	// Returns the convex hull, assuming that each points[i] <= points[i + 1]. Runs in O(n) time.
	std::vector<Vector2> makeConvexHullPresorted(const std::vector<Vector2> &points);

}
