#ifndef RVO_MINKOWSKI_H_
#define RVO_MINKOWSKI_H_

#include "ConvexHull.h"

namespace RVO {
	class Minkowski {
		public:
		static std::vector<Vector2> Sum(std::vector<Vector2> a_points, std::vector<Vector2> b_points) {
			std::vector<Vector2> sum; // = new std::vector<Vector2>();
			for(size_t i=0; i<a_points.size(); i++){
				Vector2 a_p = a_points[i];
				for(size_t j=0; j<b_points.size(); j++){
					Vector2 b_p = b_points[j];
					sum.push_back (a_p + b_p);
				}
			}
			//return makeConvexHull(sum);
			std::vector<Vector2> results = makeConvexHull(sum);
			std::reverse(results.begin(), results.end()); // return the points in counter-clockwise order
			return results;
		}

		static std::vector<Vector2> Diff(std::vector<Vector2> a_points, std::vector<Vector2> b_points) {
			std::vector<Vector2> diff;// = new std::vector<Vector2>();
			for(size_t i=0; i<a_points.size(); i++){
				Vector2 a_p = a_points[i];
				for(size_t j=0; j<b_points.size(); j++){
					Vector2 b_p = b_points[j];
					diff.push_back (a_p - b_p);
				}
			}
			//return makeConvexHull(diff);
			std::vector<Vector2> results = makeConvexHull(diff);
			std::reverse(results.begin(), results.end()); // return the points in counter-clockwise order
			return results;
		}
	};
}

#endif /* RVO_MINKOWSKI_H_ */
