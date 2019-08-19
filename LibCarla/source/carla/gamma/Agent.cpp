/*
 * Agent.cpp
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#include "Agent.h"

#include "KdTree.h"
#include "Obstacle.h"
#include <limits>

namespace RVO {
	Agent::Agent(RVOSimulator *sim) : maxNeighbors_(0), maxSpeed_(0.0f), neighborDist_(0.0f), radius_(0.0f), sim_(sim), timeHorizon_(0.0f), timeHorizonObst_(0.0f), id_(0) { }

	void Agent::computeNeighbors()
	{
		obstacleNeighbors_.clear();
		float rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + radius_);
		sim_->kdTree_->computeObstacleNeighbors(this, rangeSq);

		agentNeighbors_.clear();

		if (maxNeighbors_ > 0) {
			rangeSq = sqr(neighborDist_);
			sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
		}
	}

	void Agent::computeObstacleOrcaLinesDisc(){
		const float invTimeHorizonObst = 1.0f / timeHorizonObst_;

		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {

			const Obstacle *obstacle1 = obstacleNeighbors_[i].second;
			const Obstacle *obstacle2 = obstacle1->nextObstacle_;

			const Vector2 relativePosition1 = obstacle1->point_ - position_;
			const Vector2 relativePosition2 = obstacle2->point_ - position_;

			/*
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;

			for (size_t j = 0; j < orcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}

			if (alreadyCovered) {
				continue;
			}

			/* Not yet covered. Check for collisions. */

			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);

			const float radiusSq = sqr(radius_);

			const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);

			Line line;

			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
					orcaLines_.push_back(line);
				}

				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
					orcaLines_.push_back(line);
				}

				continue;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = Vector2(0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				orcaLines_.push_back(line);
				continue;
			}

			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */

			Vector2 leftLegDirection, rightLegDirection;

			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle2 = obstacle1;

				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle1 = obstacle2;

				const float leg2 = std::sqrt(distSq2 - radiusSq);
				leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}

				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}

			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */

			const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;

			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;

			if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}

			if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}

			/* Compute cut-off centers. */
			const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - position_);
			const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - position_);
			const Vector2 cutoffVec = rightCutoff - leftCutoff;

			/* Project current velocity on velocity obstacle. */

			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((velocity_ - leftCutoff) * leftLegDirection);
			const float tRight = ((velocity_ - rightCutoff) * rightLegDirection);

			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vector2 unitW = normalize(velocity_ - leftCutoff);

				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vector2 unitW = normalize(velocity_ - rightCutoff);

				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}

			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (rightCutoff + tRight * rightLegDirection)));

			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}

				line.direction = leftLegDirection;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}

				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
		}
	}

	void Agent::computeObstacleOrcaLinesPoly(){
		const float invTimeHorizonObst = 1.0f / timeHorizonObst_;

		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {

			const Obstacle *obstacle1 = obstacleNeighbors_[i].second;
			const Obstacle *obstacle2 = obstacle1->nextObstacle_;

			const Vector2 relativePosition1 = obstacle1->point_ - position_;
			const Vector2 relativePosition2 = obstacle2->point_ - position_;

			/*
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;

			for (size_t j = 0; j < orcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}

			if (alreadyCovered) {
				continue;
			}

			/* Not yet covered. Check for collisions. */

			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);

			const float radiusSq = sqr(radius_);

			const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);

			Line line;

			if (s < 0.0f && inCollision(bounding_corners_, obstacle1->point_)) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
					orcaLines_.push_back(line);
				}

				continue;
			}
			else if (s > 1.0f && inCollision(bounding_corners_, obstacle2->point_)) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && det(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = Vector2(0.0f, 0.0f);
					line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
					orcaLines_.push_back(line);
				}

				continue;
			}
			else if (s >= 0.0f && inCollision(bounding_corners_, relativePosition1 + s * obstacleVector + position_)) {
				/* Collision with obstacle segment. */
				line.point = Vector2(0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				orcaLines_.push_back(line);
				continue;
			}

			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */

			Vector2 leftLegDirection, rightLegDirection;

			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle2 = obstacle1;

				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle1 = obstacle2;

				const float leg2 = std::sqrt(distSq2 - radiusSq);
				leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}

				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}

			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */

			const Obstacle *const leftNeighbor = obstacle1->prevObstacle_;

			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;

			if (obstacle1->isConvex_ && det(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}

			if (obstacle2->isConvex_ && det(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}

			/* Compute cut-off centers. */
			const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - position_);
			const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - position_);
			const Vector2 cutoffVec = rightCutoff - leftCutoff;

			/* Project current velocity on velocity obstacle. */

			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((velocity_ - leftCutoff) * leftLegDirection);
			const float tRight = ((velocity_ - rightCutoff) * rightLegDirection);

			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vector2 unitW = normalize(velocity_ - leftCutoff);

				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vector2 unitW = normalize(velocity_ - rightCutoff);

				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
				orcaLines_.push_back(line);
				continue;
			}

			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (rightCutoff + tRight * rightLegDirection)));

			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}

				line.direction = leftLegDirection;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}

				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				orcaLines_.push_back(line);
				continue;
			}
		}
	}

	void Agent::computeAgentOrcaLinesDisc(){

		const float invTimeHorizon = 1.0f / timeHorizon_;

		/* Create agent ORCA lines. */
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) {
			const Agent *const other = agentNeighbors_[i].second;

			float attention = computeAttention (other);
			if (attention == 0.0f)
				continue;

			const Vector2 relativePosition = other->position_ - position_;
			const Vector2 relativeVelocity = velocity_ - other->velocity_;
			const float distSq = absSq(relativePosition);
			const float combinedRadius = radius_ + other->radius_;
			const float combinedRadiusSq = sqr(combinedRadius);

			Line line;
			Vector2 u;

			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);

				const float dotProduct1 = w * relativePosition;

				if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector2 unitW = w / wLength;

					line.direction = Vector2(unitW.y(), -unitW.x());
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);

					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}
					else {
						/* Project on right leg. */
						line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}

					const float dotProduct2 = relativeVelocity * line.direction;

					u = dotProduct2 * line.direction - relativeVelocity;
				}
			}
			else {
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / sim_->timeStep_;

				/* Vector from cutoff center to relative velocity. */
				const Vector2 w = relativeVelocity - invTimeStep * relativePosition;

				const float wLength = abs(w);
				const Vector2 unitW = w / wLength;

				line.direction = Vector2(unitW.y(), -unitW.x());
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}

			line.point = velocity_ + computeResponsibility(other) * u;
			orcaLines_.push_back(line);
		}
	}

	void Agent::computeAgentOrcaLinesPoly(){

		const float invTimeHorizon = 1.0f / timeHorizon_;

		/* Create agent ORCA lines. */
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) {

			const Agent *const other = agentNeighbors_[i].second;

			float attention = computeAttention (other);
			if (attention == 0.0f)
				continue;

			std::vector<Vector2> minkowski_diff = Minkowski::Diff(other->bounding_corners_, bounding_corners_);

			bool in_collision = inCollision (minkowski_diff, Vector2(0,0));

			const Vector2 relativeVelocity = velocity_ - other->velocity_;

			Line line;
			Vector2 u;

			if (!in_collision) {
				Vector2 left_most_vector = Vector2 (0.0f, 0.0f);
				Vector2 right_most_vector = Vector2 (0.0f, 0.0f);

				computeLeftAndRightMostVector (left_most_vector, right_most_vector, minkowski_diff);

				float shortest_dist_sq = std::numeric_limits<float>::infinity();
				float tmp_shortest_dist_sq;
				Vector2 nearest_point = Vector2 (0.0f, 0.0f);
				Vector2 tmp_nearest_point = Vector2 (0.0f, 0.0f);

				size_t shortest_line_idx = 0;

				for(size_t j=0; j<minkowski_diff.size() - 1; j++){
					if (leftOf (right_most_vector, left_most_vector, minkowski_diff [j]) < 0 || minkowski_diff [j] == right_most_vector) {
						//it is okay not to multiply invTimeHorizon in the condition, because it is only used for pruning out the the unnecessary edges of the minkowski diff
						continue;
					}
						
					tmp_shortest_dist_sq = distSqPointLineSegment(invTimeHorizon * minkowski_diff[j], invTimeHorizon * minkowski_diff[j+1], relativeVelocity, tmp_nearest_point);
					if (tmp_shortest_dist_sq < shortest_dist_sq){
						shortest_dist_sq = tmp_shortest_dist_sq;
						nearest_point = tmp_nearest_point;
						shortest_line_idx = j;
					}
				}

				if (leftOf (right_most_vector, left_most_vector, minkowski_diff [minkowski_diff.size() - 1]) < 0 || minkowski_diff [minkowski_diff.size() - 1] == right_most_vector) {
					;
				} else {
					tmp_shortest_dist_sq = distSqPointLineSegment(invTimeHorizon * minkowski_diff[minkowski_diff.size() - 1], invTimeHorizon * minkowski_diff[0], relativeVelocity, tmp_nearest_point);
					if (tmp_shortest_dist_sq < shortest_dist_sq){
						shortest_dist_sq = tmp_shortest_dist_sq;
						nearest_point = tmp_nearest_point;
						shortest_line_idx = minkowski_diff.size() - 1;
					}
				}

				// note that both right_most_vector and left_most_vector go through origin
				tmp_shortest_dist_sq = distSqPointLineSegment(invTimeHorizon * right_most_vector, 10000 * invTimeHorizon * right_most_vector, relativeVelocity, tmp_nearest_point);
				if (tmp_shortest_dist_sq < shortest_dist_sq){
					shortest_dist_sq = tmp_shortest_dist_sq;
					nearest_point = tmp_nearest_point;
					shortest_line_idx = minkowski_diff.size();
				}

				tmp_shortest_dist_sq = distSqPointLineSegment(invTimeHorizon * left_most_vector, 10000 * invTimeHorizon * left_most_vector, relativeVelocity, tmp_nearest_point);
				if (tmp_shortest_dist_sq < shortest_dist_sq){
					shortest_dist_sq = tmp_shortest_dist_sq;
					nearest_point = tmp_nearest_point;
					shortest_line_idx = minkowski_diff.size() + 1 ;
				}


				//shortest_line_start and shortest_line_end are used to tell whether relative velocity is on 
				// the left side or the right side of the nearest edge, hence we don't need to multiply invTimeHorizon
				Vector2 shortest_line_start = Vector2 (0, 0);
				Vector2 shortest_line_end = Vector2 (0, 0);
				if (shortest_line_idx < minkowski_diff.size() - 1) {
					shortest_line_start = minkowski_diff [shortest_line_idx];
					shortest_line_end = minkowski_diff [shortest_line_idx + 1];
				} else if (shortest_line_idx == minkowski_diff.size() - 1) {
					shortest_line_start = minkowski_diff [shortest_line_idx];
					shortest_line_end = minkowski_diff [0];
				} else if (shortest_line_idx == minkowski_diff.size()) {
					shortest_line_start = right_most_vector;
					shortest_line_end = 10000*right_most_vector;
				} else if (shortest_line_idx == minkowski_diff.size() + 1) {
					shortest_line_start = 10000*left_most_vector;
					shortest_line_end = left_most_vector;
				}

				u = nearest_point - relativeVelocity;
				float uLength = abs (u);
				Vector2 unitU = u / uLength;
				if (leftOf (shortest_line_start, shortest_line_end, relativeVelocity) > 0) {
					line.direction = Vector2 (unitU.y (), -unitU.x ()); //rotate 90 degree clockwisely
				} else {
					line.direction = Vector2 (-unitU.y (), unitU.x ()); //rotate 90 degree counter-clockwisely
				}

				line.point = velocity_ + computeResponsibility(other) * u;

			} else {
				/* Collision. Project on cut-off circle of time timeStep. */
				//const float invTimeStep = 1.0f / sim_->timeStep_;

				float shortest_dist_sq = std::numeric_limits<float>::infinity();
				float tmp_shortest_dist_sq;
				Vector2 nearest_point = Vector2 (0.0f, 0.0f);
				Vector2 tmp_nearest_point = Vector2 (0.0f, 0.0f);

				size_t shortest_line_idx = 0;

				for(size_t j=0; j<minkowski_diff.size() - 1; j++){
					tmp_shortest_dist_sq = distSqPointLineSegment(minkowski_diff[j], minkowski_diff[j+1], Vector2 (0.0f, 0.0f), tmp_nearest_point);
					if (tmp_shortest_dist_sq < shortest_dist_sq){
						shortest_dist_sq = tmp_shortest_dist_sq;
						nearest_point = tmp_nearest_point;
						shortest_line_idx = j;
					}
				}

				tmp_shortest_dist_sq = distSqPointLineSegment(minkowski_diff[minkowski_diff.size() - 1], minkowski_diff[0], Vector2 (0.0f, 0.0f), tmp_nearest_point);
				if (tmp_shortest_dist_sq < shortest_dist_sq){
					shortest_dist_sq = tmp_shortest_dist_sq;
					nearest_point = tmp_nearest_point;
					shortest_line_idx = minkowski_diff.size() - 1;
				}

				Vector2 shortest_line_start = minkowski_diff[shortest_line_idx];
				Vector2 shortest_line_end = Vector2 (0, 0);
				if (shortest_line_idx == minkowski_diff.size() - 1) {
					shortest_line_end = minkowski_diff [0];
				} else {
					shortest_line_end = minkowski_diff [shortest_line_idx + 1];
				}

				u = nearest_point - Vector2 (0.0f, 0.0f);
				float uLength = abs (u);
				Vector2 unitU = u / uLength;
				if (leftOf (shortest_line_start, shortest_line_end, Vector2 (0.0f, 0.0f)) >= 0) {
					line.direction = Vector2 (unitU.y (), -unitU.x ()); //rotate 90 degree clockwisely
				} else {
					line.direction = Vector2 (-unitU.y (), unitU.x ()); //rotate 90 degree counter-clockwisely
				}

				line.point = Vector2 (0.0f, 0.0f);
			}

			
			orcaLines_.push_back(line);
		}
	}

	// void Agent::computeAgentOrcaLinesPoly(){

	// 	const float invTimeHorizon = 1.0f / timeHorizon_;

	// 	/* Create agent ORCA lines. */
	// 	for (size_t i = 0; i < agentNeighbors_.size(); ++i) {

	// 		const Agent *const other = agentNeighbors_[i].second;

	// 		float attention = computeAttention (other);
	// 		if (attention == 0.0f)
	// 			continue;

	// 		std::vector<Vector2> minkowski_diff = Minkowski::Diff(other->bounding_corners_, bounding_corners_);

	// 		bool in_collision = inCollision (minkowski_diff, Vector2(0,0));

	// 		const Vector2 relativeVelocity = velocity_ - other->velocity_;

	// 		Line line;
	// 		Vector2 u;

	// 		if (!in_collision) {
	// 			Vector2 left_most_vector = Vector2 (0.0f, 0.0f);
	// 			Vector2 right_most_vector = Vector2 (0.0f, 0.0f);

	// 			computeLeftAndRightMostVector (left_most_vector, right_most_vector, minkowski_diff);

	// 			float shortest_dist_sq = std::numeric_limits<float>::infinity();
	// 			float tmp_shortest_dist_sq;
	// 			Vector2 nearest_point = Vector2 (0.0f, 0.0f);
	// 			Vector2 tmp_nearest_point = Vector2 (0.0f, 0.0f);

	// 			size_t shortest_line_idx = 0;

	// 			for(size_t j=0; j<minkowski_diff.size() - 1; j++){
	// 				if (leftOf (right_most_vector, left_most_vector, minkowski_diff [j]) < 0 || minkowski_diff [j] == right_most_vector) {
	// 					//it is okay not to multiply invTimeHorizon in the condition, because it is only used for pruning out the the unnecessary edges of the minkowski diff
	// 					continue;
	// 				}
						
	// 				tmp_shortest_dist_sq = distSqPointLineSegment(invTimeHorizon * minkowski_diff[j], invTimeHorizon * minkowski_diff[j+1], relativeVelocity, tmp_nearest_point);
	// 				if (tmp_shortest_dist_sq < shortest_dist_sq){
	// 					shortest_dist_sq = tmp_shortest_dist_sq;
	// 					nearest_point = tmp_nearest_point;
	// 					shortest_line_idx = j;
	// 				}
	// 			}

	// 			if (leftOf (right_most_vector, left_most_vector, minkowski_diff [minkowski_diff.size() - 1]) < 0 || minkowski_diff [minkowski_diff.size() - 1] == right_most_vector) {
	// 				;
	// 			} else {
	// 				tmp_shortest_dist_sq = distSqPointLineSegment(invTimeHorizon * minkowski_diff[minkowski_diff.size() - 1], invTimeHorizon * minkowski_diff[0], relativeVelocity, tmp_nearest_point);
	// 				if (tmp_shortest_dist_sq < shortest_dist_sq){
	// 					shortest_dist_sq = tmp_shortest_dist_sq;
	// 					nearest_point = tmp_nearest_point;
	// 					shortest_line_idx = minkowski_diff.size() - 1;
	// 				}
	// 			}

	// 			// note that both right_most_vector and left_most_vector go through origin
	// 			tmp_shortest_dist_sq = distSqPointLineSegment(invTimeHorizon * right_most_vector, 10000 * invTimeHorizon * right_most_vector, relativeVelocity, tmp_nearest_point);
	// 			if (tmp_shortest_dist_sq < shortest_dist_sq){
	// 				shortest_dist_sq = tmp_shortest_dist_sq;
	// 				nearest_point = tmp_nearest_point;
	// 				shortest_line_idx = minkowski_diff.size();
	// 			}

	// 			tmp_shortest_dist_sq = distSqPointLineSegment(invTimeHorizon * left_most_vector, 10000 * invTimeHorizon * left_most_vector, relativeVelocity, tmp_nearest_point);
	// 			if (tmp_shortest_dist_sq < shortest_dist_sq){
	// 				shortest_dist_sq = tmp_shortest_dist_sq;
	// 				nearest_point = tmp_nearest_point;
	// 				shortest_line_idx = minkowski_diff.size() + 1 ;
	// 			}


	// 			//shortest_line_start and shortest_line_end are used to tell whether relative velocity is on 
	// 			// the left side or the right side of the nearest edge, hence we don't need to multiply invTimeHorizon
	// 			Vector2 shortest_line_start = Vector2 (0, 0);
	// 			Vector2 shortest_line_end = Vector2 (0, 0);
	// 			if (shortest_line_idx < minkowski_diff.size() - 1) {
	// 				shortest_line_start = minkowski_diff [shortest_line_idx];
	// 				shortest_line_end = minkowski_diff [shortest_line_idx + 1];
	// 			} else if (shortest_line_idx == minkowski_diff.size() - 1) {
	// 				shortest_line_start = minkowski_diff [shortest_line_idx];
	// 				shortest_line_end = minkowski_diff [0];
	// 			} else if (shortest_line_idx == minkowski_diff.size()) {
	// 				shortest_line_start = right_most_vector;
	// 				shortest_line_end = 10000*right_most_vector;
	// 			} else if (shortest_line_idx == minkowski_diff.size() + 1) {
	// 				shortest_line_start = 10000*left_most_vector;
	// 				shortest_line_end = left_most_vector;
	// 			}

	// 			u = nearest_point - relativeVelocity;
	// 			float uLength = abs (u);
	// 			Vector2 unitU = u / uLength;
	// 			if (leftOf (shortest_line_start, shortest_line_end, relativeVelocity) > 0) {
	// 				line.direction = Vector2 (unitU.y (), -unitU.x ()); //rotate 90 degree clockwisely
	// 			} else {
	// 				line.direction = Vector2 (-unitU.y (), unitU.x ()); //rotate 90 degree counter-clockwisely
	// 			}

	// 		} else {
	// 			/* Collision. Project on cut-off circle of time timeStep. */
	// 			//const float invTimeStep = 1.0f / sim_->timeStep_;

	// 			float shortest_dist_sq = std::numeric_limits<float>::infinity();
	// 			float tmp_shortest_dist_sq;
	// 			Vector2 nearest_point = Vector2 (0.0f, 0.0f);
	// 			Vector2 tmp_nearest_point = Vector2 (0.0f, 0.0f);

	// 			size_t shortest_line_idx = 0;

	// 			for(size_t j=0; j<minkowski_diff.size() - 1; j++){
	// 				tmp_shortest_dist_sq = distSqPointLineSegment(minkowski_diff[j], minkowski_diff[j+1], relativeVelocity, tmp_nearest_point);
	// 				if (tmp_shortest_dist_sq < shortest_dist_sq){
	// 					shortest_dist_sq = tmp_shortest_dist_sq;
	// 					nearest_point = tmp_nearest_point;
	// 					shortest_line_idx = j;
	// 				}
	// 			}

	// 			tmp_shortest_dist_sq = distSqPointLineSegment(minkowski_diff[minkowski_diff.size() - 1], minkowski_diff[0], relativeVelocity, tmp_nearest_point);
	// 			if (tmp_shortest_dist_sq < shortest_dist_sq){
	// 				shortest_dist_sq = tmp_shortest_dist_sq;
	// 				nearest_point = tmp_nearest_point;
	// 				shortest_line_idx = minkowski_diff.size() - 1;
	// 			}

	// 			Vector2 shortest_line_start = minkowski_diff[shortest_line_idx];
	// 			Vector2 shortest_line_end = Vector2 (0, 0);
	// 			if (shortest_line_idx == minkowski_diff.size() - 1) {
	// 				shortest_line_end = minkowski_diff [0];
	// 			} else {
	// 				shortest_line_end = minkowski_diff [shortest_line_idx + 1];
	// 			}

	// 			u = nearest_point - relativeVelocity;
	// 			float uLength = abs (u);
	// 			Vector2 unitU = u / uLength;
	// 			if (leftOf (shortest_line_start, shortest_line_end, relativeVelocity) > 0) {
	// 				line.direction = Vector2 (unitU.y (), -unitU.x ()); //rotate 90 degree clockwisely
	// 			} else {
	// 				line.direction = Vector2 (-unitU.y (), unitU.x ()); //rotate 90 degree counter-clockwisely
	// 			}
	// 		}

	// 		line.point = velocity_ + computeResponsibility(other) * u;
	// 		orcaLines_.push_back(line);
	// 	}
	// }

	/* Search for the best new velocity. */
	void Agent::computeNewVelocity()
	{
		orcaLines_.clear();

		if(tag_ != "People"){
			if (GammaParams::use_polygon) {
				computeObstacleOrcaLinesPoly ();
			} else {
				computeObstacleOrcaLinesDisc ();
			}
		}

		// if(tag_ != "People"){
		// 	computeLaneConstrains ();
		// }
			

		const size_t numObstLines = orcaLines_.size();

		if (GammaParams::use_polygon) {
			computeAgentOrcaLinesPoly ();
		} else {
			computeAgentOrcaLinesDisc ();
		}

		if (GammaParams::consider_kinematics) {
			//computeKinematicVelSet (max_tracking_angle_);
			computeKinematicVelSet ();
		}

		size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);

		if (lineFail < orcaLines_.size()) {
			linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
		}
	}

	void Agent::computeLaneConstrains (){

		float max_sideward_speed_allowed = 0.1f;
		if(left_lane_constrained_ && path_forward_ != Vector2(0.0f, 0.0f)){
			Vector2 left_sideward_vec = normalize(path_forward_.rotate(90.0f));
			Line line;
			line.point = Vector2(0.0f, 0.0f) + max_sideward_speed_allowed * left_sideward_vec;
			// the feasible space is on the left side of a vector
			line.direction = normalize(path_forward_.rotate(180.0f));
			orcaLines_.push_back(line);

			path_forward_ = Vector2(0.0f, 0.0f);
			left_lane_constrained_ = false;
		}


		if(right_lane_constrained_  && path_forward_ != Vector2(0.0f, 0.0f) ){
			Vector2 right_sideward_vec = normalize(path_forward_.rotate(-90.0f));
			Line line;
			line.point = Vector2(0.0f, 0.0f) + max_sideward_speed_allowed * right_sideward_vec;
			// the feasible space is on the left side of a vector
			line.direction = normalize(path_forward_);
			orcaLines_.push_back(line);

			path_forward_ = Vector2(0.0f, 0.0f);
			right_lane_constrained_ = false;
		}
	}


	 void Agent::computeLeftAndRightMostVector(Vector2 &left_most_vector, Vector2 &right_most_vector, std::vector<Vector2> &minkowski_diff){
		
		left_most_vector = minkowski_diff [0];
		right_most_vector = minkowski_diff [0];

		Vector2 origin = Vector2 (0.0f, 0.0f);

		for (size_t i = 1; i < minkowski_diff.size(); i++) {
			if(leftOf(origin, left_most_vector, minkowski_diff [i]) > 0){ // the new vector is on the left side of the current left most vector
				left_most_vector = minkowski_diff [i];
			}else if(leftOf(origin, right_most_vector, minkowski_diff [i]) < 0){ // the new vector is on the right side of the current right most vector
				right_most_vector = minkowski_diff [i];
			}
		}
	}


	bool Agent::inCollision(const std::vector<Vector2> &polygon, const Vector2 &ref_point){

		for(size_t i=0; i<polygon.size()-1; i++){
			if (leftOf (polygon[i], polygon[i+1], ref_point) < 0) // the reference point is outside the polygon
				return false;
		}

		if (leftOf (polygon [polygon.size() - 1], polygon [0], ref_point) < 0)
			return false;

		return true;
	}

	void Agent::computeKinematicVelSet (float _max_tracking_bound){
		if(tag_ == "Car" || tag_ == "Scooter") {
			Line line;
			line.point = Vector2(0.0f, 0.0f);
			// the feasible space is on the left side of a vector
			line.direction = normalize(heading_.rotate(-_max_tracking_bound)); // rotate clockwise by |_max_tracking_bound| angle
			orcaLines_.push_back(line);
			line.direction = normalize(heading_.rotate(-(180.0f-_max_tracking_bound)));
			orcaLines_.push_back(line);
		}
	}

	void Agent::computeKinematicVelSet (){
		if(velocity_convex_.size() >= 3) {

			std::vector<Vector2> estimated_k; //estimated_k is the estimated kinematically feasible velocity set, which is the velocity_convex_ rotated to current heading of the agent

			float angle_diff = 57.296f * getSignedAngleRadOfTwoVector(heading_, Vector2(0.0f, 1.0f)); // 57.296f = 180.0f / 3.1415926f
			for(size_t i=0; i<velocity_convex_.size(); i++){
				estimated_k.push_back(velocity_convex_[i].rotate(angle_diff));
			}

			for(size_t i=0; i<estimated_k.size()-1; i++){
				Line line;
				line.point = estimated_k[i];
				line.direction = normalize(estimated_k[i+1] - estimated_k[i]); 
				orcaLines_.push_back(line);
			}

			Line line;
			line.point = estimated_k[estimated_k.size()-1];
			line.direction = normalize(estimated_k[0] - estimated_k[estimated_k.size()-1]); 
			orcaLines_.push_back(line);
		}
	}

	float Agent::computeResponsibility(const Agent * other){
		if(!GammaParams::use_dynamic_resp || tag_ == other->tag_)
			return 0.5f;

		Vector2 relativePosition = other->position_ - position_;
		float dist = abs (relativePosition);


		float agt_res = 1.0f + 2.5f*res_dec_rate_ - res_dec_rate_ * dist;
		float other_agt_res = 1.0f + 2.5f*other->res_dec_rate_ - other->res_dec_rate_ * dist;

		if (agt_res < 0.1f) {
			agt_res = 0.1f;
		}
		if (other_agt_res < 0.1f) {
			other_agt_res = 0.1f;
		}

		//normalize to 1
		if(agt_res == 0) return 0.0f;
		else 
			agt_res /= (agt_res + other_agt_res);

		return agt_res;
	}

	float Agent::computeAttention(const Agent * other){
		if (!GammaParams::use_dynamic_att)
			return 1.0f;

		if (heading_ * (other->position_ - position_) >= 0.0f) { //  other agent is in front of the ego agent
			float dist_sq = absSq (other->position_ - position_);
			if (dist_sq > r_front_*r_front_)
				return 0.0f;
			return 1.0f;

		} else { // other agent is behind the ego agent
			float dist_sq = absSq (other->position_ - position_);
			if (dist_sq > r_rear_*r_rear_)
				return 0.0f;
			return 1.0f;
		}
	}


	void Agent::insertAgentNeighbor(const Agent *agent, float &rangeSq)
	{
		if (this != agent) {
			const float distSq = absSq(position_ - agent->position_);

			if (distSq < rangeSq) {
				if (agentNeighbors_.size() < maxNeighbors_) {
					agentNeighbors_.push_back(std::make_pair(distSq, agent));
				}

				size_t i = agentNeighbors_.size() - 1;

				while (i != 0 && distSq < agentNeighbors_[i - 1].first) {
					agentNeighbors_[i] = agentNeighbors_[i - 1];
					--i;
				}

				agentNeighbors_[i] = std::make_pair(distSq, agent);

				if (agentNeighbors_.size() == maxNeighbors_) {
					rangeSq = agentNeighbors_.back().first;
				}
			}
		}
	}

	void Agent::insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq)
	{
		const Obstacle *const nextObstacle = obstacle->nextObstacle_;

		const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, position_);

		if (distSq < rangeSq) {
			obstacleNeighbors_.push_back(std::make_pair(distSq, obstacle));

			size_t i = obstacleNeighbors_.size() - 1;

			while (i != 0 && distSq < obstacleNeighbors_[i - 1].first) {
				obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
				--i;
			}

			obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
		}
	}

	void Agent::update()
	{
		velocity_ = newVelocity_;
		position_ += velocity_ * sim_->timeStep_;

		if(absSq(velocity_) != 0)
			heading_ = normalize (velocity_);

	}

	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo, float radius, const Vector2 &optVelocity, bool directionOpt, Vector2 &result)
	{
		const float dotProduct = lines[lineNo].point * lines[lineNo].direction;
		const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(lines[lineNo].point);

		if (discriminant < 0.0f) {
			/* Max speed circle fully invalidates line lineNo. */
			return false;
		}

		const float sqrtDiscriminant = std::sqrt(discriminant);
		float tLeft = -dotProduct - sqrtDiscriminant;
		float tRight = -dotProduct + sqrtDiscriminant;

		for (size_t i = 0; i < lineNo; ++i) {
			const float denominator = det(lines[lineNo].direction, lines[i].direction);
			const float numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);

			if (std::fabs(denominator) <= RVO_EPSILON) {
				/* Lines lineNo and i are (almost) parallel. */
				if (numerator < 0.0f) {
					return false;
				}
				else {
					continue;
				}
			}

			const float t = numerator / denominator;

			if (denominator >= 0.0f) {
				/* Line i bounds line lineNo on the right. */
				tRight = std::min(tRight, t);
			}
			else {
				/* Line i bounds line lineNo on the left. */
				tLeft = std::max(tLeft, t);
			}

			if (tLeft > tRight) {
				return false;
			}
		}

		if (directionOpt) {
			/* Optimize direction. */
			if (optVelocity * lines[lineNo].direction > 0.0f) {
				/* Take right extreme. */
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				/* Take left extreme. */
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
		}
		else {
			/* Optimize closest point. */
			const float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

			if (t < tLeft) {
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
			else if (t > tRight) {
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				result = lines[lineNo].point + t * lines[lineNo].direction;
			}
		}

		return true;
	}

	size_t linearProgram2(const std::vector<Line> &lines, float radius, const Vector2 &optVelocity, bool directionOpt, Vector2 &result)
	{
		if (directionOpt) {
			/*
			 * Optimize direction. Note that the optimization velocity is of unit
			 * length in this case.
			 */
			result = optVelocity * radius;
		}
		else if (absSq(optVelocity) > sqr(radius)) {
			/* Optimize closest point and outside circle. */
			result = normalize(optVelocity) * radius;
		}
		else {
			/* Optimize closest point and inside circle. */
			result = optVelocity;
		}

		for (size_t i = 0; i < lines.size(); ++i) {
			if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				const Vector2 tempResult = result;

				if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
					result = tempResult;
					return i;
				}
			}
		}

		return lines.size();
	}

	void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Vector2 &result)
	{
		float distance = 0.0f;

		for (size_t i = beginLine; i < lines.size(); ++i) {
			if (det(lines[i].direction, lines[i].point - result) > distance) {
				/* Result does not satisfy constraint of line i. */
				std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));

				for (size_t j = numObstLines; j < i; ++j) {
					Line line;

					float determinant = det(lines[i].direction, lines[j].direction);

					if (std::fabs(determinant) <= RVO_EPSILON) {
						/* Line i and line j are parallel. */
						if (lines[i].direction * lines[j].direction > 0.0f) {
							/* Line i and line j point in the same direction. */
							continue;
						}
						else {
							/* Line i and line j point in opposite direction. */
							line.point = 0.5f * (lines[i].point + lines[j].point);
						}
					}
					else {
						line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
					}

					line.direction = normalize(lines[j].direction - lines[i].direction);
					projLines.push_back(line);
				}

				const Vector2 tempResult = result;

				if (linearProgram2(projLines, radius, Vector2(-lines[i].direction.y(), lines[i].direction.x()), true, result) < projLines.size()) {
					/* This should in principle not happen.  The result is by definition
					 * already in the feasible region of this linear program. If it fails,
					 * it is due to small floating point error, and the current result is
					 * kept.
					 */
					result = tempResult;
				}

				distance = det(lines[i].direction, lines[i].point - result);
			}
		}
	}
}
