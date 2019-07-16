/*
 * Convex hull algorithm - Library (C++)
 * 
 * Copyright (c) 2017 Project Nayuki
 * https://www.nayuki.io/page/convex-hull-algorithm
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program (see COPYING.txt and COPYING.LESSER.txt).
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RVO_CONVEX_HULL_H_
#define RVO_CONVEX_HULL_H_

#include "ConvexHull.h"

namespace RVO {

	using std::vector;


	vector<Vector2> makeConvexHull(const vector<Vector2> &points) {
		vector<Vector2> newPoints = points;
		std::sort(newPoints.begin(), newPoints.end());
		return makeConvexHullPresorted(newPoints);
	}


	vector<Vector2> makeConvexHullPresorted(const vector<Vector2> &points) {
		if (points.size() <= 1)
			return vector<Vector2>(points);
		
		// Andrew's monotone chain algorithm. Positive y coordinates correspond to "up"
		// as per the mathematical convention, instead of "down" as per the computer
		// graphics convention. This doesn't affect the correctness of the result.
		
		vector<Vector2> upperHull;
		for (int i=0; i<points.size();i++) {
			Vector2 p = points[i];
			while (upperHull.size() >= 2) {
				Vector2 &q = *(upperHull.end() - 1);  // Same as .back()
				Vector2 &r = *(upperHull.end() - 2);
				if ((q.x() - r.x()) * (p.y() - r.y()) >= (q.y() - r.y()) * (p.x() - r.x()))
					upperHull.pop_back();
				else
					break;
			}
			upperHull.push_back(p);
		}
		upperHull.pop_back();
		
		vector<Vector2> lowerHull;
		for (vector<Vector2>::const_reverse_iterator it = points.rbegin(); it != points.rend(); ++it) {
			const Vector2 &p = *it;
			while (lowerHull.size() >= 2) {
				Vector2 &q = *(lowerHull.end() - 1);  // Same as .back()
				Vector2 &r = *(lowerHull.end() - 2);
				if ((q.x() - r.x()) * (p.y() - r.y()) >= (q.y() - r.y()) * (p.x() - r.x()))
					lowerHull.pop_back();
				else
					break;
			}
			lowerHull.push_back(p);
		}
		lowerHull.pop_back();
		
		if (!(upperHull.size() == 1 && upperHull == lowerHull))
			upperHull.insert(upperHull.end(), lowerHull.begin(), lowerHull.end());
		return upperHull;
	}
}

#endif /* RVO_CONVEX_HULL_H_ */