/*
 * RVOSimulator.cpp
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

#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"


#ifdef _OPENMP
#include <omp.h>
#endif

namespace RVO {
	RVOSimulator::RVOSimulator() : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(0.0f)
	{
		kdTree_ = new KdTree(this);

	}

	RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity) : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(timeStep)
	{
		kdTree_ = new KdTree(this);
		defaultAgent_ = new Agent(this);

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;
	}

	RVOSimulator::~RVOSimulator()
	{
		if (defaultAgent_ != NULL) {
			delete defaultAgent_;
		}

		for (size_t i = 0; i < agents_.size(); ++i) {
			delete agents_[i];
		}

		for (size_t i = 0; i < obstacles_.size(); ++i) {
			delete obstacles_[i];
		}

		delete kdTree_;
	}

	void RVOSimulator::clearAllAgents()
	{
		for(size_t i=0; i< agents_.size(); i++)
		{
			if(agents_[i]!=NULL){
				delete agents_[i];
				agents_[i] = NULL;
			}
		}
		agents_.clear();
		kdTree_->clearAllAgents();
	}


	size_t RVOSimulator::addAgent(const Vector2 &position)
	{
		if (defaultAgent_ == NULL) {
			return RVO_ERROR;
		}

		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizon_ = defaultAgent_->timeHorizon_;
		agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
		agent->velocity_ = defaultAgent_->velocity_;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
	{
		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->timeHorizonObst_ = timeHorizonObst;
		agent->velocity_ = velocity;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity, std::string tag, float max_tracking_angle)
	{
		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->timeHorizonObst_ = timeHorizonObst;
		agent->velocity_ = velocity;

		agent->id_ = agents_.size();

		agent->tag_ = tag;
		agent->max_tracking_angle_ = max_tracking_angle;


		agents_.push_back(agent);

		return agents_.size() - 1;
	}


	size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity, std::string tag, float max_tracking_angle, int tracking_id)
	{
		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->timeHorizonObst_ = timeHorizonObst;
		agent->velocity_ = velocity;

		agent->id_ = agents_.size();

		agent->tag_ = tag;
		agent->max_tracking_angle_ = max_tracking_angle;

		agent->tracking_id_ = tracking_id;



		agents_.push_back(agent);

		return agents_.size() - 1;
	}


	size_t RVOSimulator::addAgent(const AgentParams agt, int tracking_id)
	{
		Agent *agent = new Agent(this);

		agent->position_ = agt.position;
		agent->maxNeighbors_ = static_cast<size_t> (agt.maxNeighbors);
		agent->maxSpeed_ = agt.maxSpeed;
		agent->neighborDist_ = agt.neighborDist;
		agent->radius_ = agt.radius;
		agent->timeHorizon_ = agt.timeHorizon;
		agent->timeHorizonObst_ = agt.timeHorizonObst;
		agent->velocity_ = agt.velocity;

		agent->id_ = agents_.size();

		agent->tag_ = agt.tag;
		agent->max_tracking_angle_ = agt.max_tracking_angle;

		agent->tracking_id_ = tracking_id;

		agent->r_front_ = agt.r_front;
		agent->r_rear_ = agt.r_rear;
		agent->res_dec_rate_ = agt.res_dec_rate;


		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	void RVOSimulator::setAgent(int agentNo, const AgentParams agt)
	{
		Agent *agent = agents_[static_cast<size_t>(agentNo)];

		agent->position_ = agt.position;
		agent->maxNeighbors_ = static_cast<size_t> (agt.maxNeighbors);
		agent->maxSpeed_ = agt.maxSpeed;
		agent->neighborDist_ = agt.neighborDist;
		agent->radius_ = agt.radius;
		agent->timeHorizon_ = agt.timeHorizon;
		agent->timeHorizonObst_ = agt.timeHorizonObst;
		
		agent->velocity_ = agt.velocity;//?

		//agent->id_ = agents_.size();

		agent->tag_ = agt.tag;
		agent->max_tracking_angle_ = agt.max_tracking_angle;

		agent->r_front_ = agt.r_front;
		agent->r_rear_ = agt.r_rear;
		agent->res_dec_rate_ = agt.res_dec_rate;

		//agent->tracking_id_ = tracking_id;
	}


	void RVOSimulator::setAgentID(int agentNo, int tracking_id){
		agents_[static_cast<size_t>(agentNo)]->tracking_id_ = tracking_id;
	}

	int RVOSimulator::getAgentID(int agentNo){
		return agents_[static_cast<size_t>(agentNo)]->tracking_id_;
	}

	std::string RVOSimulator::getAgentTag(int agentNo){
		return agents_[static_cast<size_t>(agentNo)]->tag_;
	}

	Vector2 RVOSimulator::getAgentHeading(int agentNo)
	{
		return agents_[static_cast<size_t>(agentNo)]->heading_;
	}
	void RVOSimulator::setAgentBoundingBoxCorners(int agentNo, std::vector<Vector2> corners)
	{
		agents_[static_cast<size_t>(agentNo)]->bounding_corners_ = corners;
	}

	void RVOSimulator::setAgentHeading(int agentNo, Vector2 heading){
		agents_[static_cast<size_t>(agentNo)]->heading_ = heading;
	}

	void RVOSimulator::setAgentMaxTrackingAngle(int agentNo, float max_tracking_angle){
		agents_[static_cast<size_t>(agentNo)]->max_tracking_angle_ = max_tracking_angle;
	}

	void RVOSimulator::setAgentAttentionRadius(int agentNo, float r_front, float r_rear){
		agents_[static_cast<size_t>(agentNo)]->r_front_ = r_front;
		agents_[static_cast<size_t>(agentNo)]->r_rear_ = r_rear;
	}

	void RVOSimulator::setAgentResDecRate(int agentNo, float res_dec_rate){
		agents_[static_cast<size_t>(agentNo)]->res_dec_rate_ = res_dec_rate;
	}

////////////////////////////////

	size_t RVOSimulator::addObstacle(const std::vector<Vector2> &vertices)
	{
		if (vertices.size() < 2) {
			return RVO_ERROR;
		}

		const size_t obstacleNo = obstacles_.size();

		for (size_t i = 0; i < vertices.size(); ++i) {
			Obstacle *obstacle = new Obstacle();
			obstacle->point_ = vertices[i];

			if (i != 0) {
				obstacle->prevObstacle_ = obstacles_.back();
				obstacle->prevObstacle_->nextObstacle_ = obstacle;
			}

			if (i == vertices.size() - 1) {
				obstacle->nextObstacle_ = obstacles_[obstacleNo];
				obstacle->nextObstacle_->prevObstacle_ = obstacle;
			}

			obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);

			if (vertices.size() == 2) {
				obstacle->isConvex_ = true;
			}
			else {
				obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
			}

			obstacle->id_ = obstacles_.size();

			obstacles_.push_back(obstacle);
		}

		return obstacleNo;
	}

	void RVOSimulator::doStep()
	{
		kdTree_->buildAgentTree();

#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[static_cast<size_t>(i)]->computeNeighbors();
			agents_[static_cast<size_t>(i)]->computeNewVelocity();
		}

#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[static_cast<size_t>(i)]->update();
		}

		globalTime_ += timeStep_;
	}

	float RVOSimulator::getSignedAngleRadOfTwoVector(Vector2 a, Vector2 b){
		float theta = static_cast<float>(atan2 (a.y (), a.x ()) - atan2 (b.y (), b.x ()));
		if (theta > GammaParams::GAMMA_PI)
			theta -= 2 * GammaParams::GAMMA_PI;
		if (theta < - GammaParams::GAMMA_PI)
			theta += 2 * GammaParams::GAMMA_PI;

		return theta;
	}

	// return next position and heading for tracking pref_vel by applying bicycle model for dt time
	std::vector<Vector2> RVOSimulator::bicycleMove (Vector2 cur_pos, Vector2 cur_heading, Vector2 pref_vel, float dt, float max_speed, float car_len, float max_tracking_angle_deg) {

		std::vector<Vector2> pos_heading;

		float speed_ = abs (pref_vel); //this is equivalent to making k0 infinite, where k0 is the control param for acceleration P controller: a = k0*(v* - v)

		if (speed_ == 0) {
			pos_heading.push_back (cur_pos);
			pos_heading.push_back (cur_heading);
			return pos_heading;
		} else if (speed_ > max_speed){
			speed_ = max_speed;
		} else if (speed_ < -max_speed){
			speed_ = -max_speed;
		}

		float max_tracking_angle_rad = max_tracking_angle_deg * GammaParams::GAMMA_PI / 180.0f;
			
		float steer_ = (car_len/6.8f)*getSignedAngleRadOfTwoVector(pref_vel, cur_heading);
		if (steer_ > max_tracking_angle_rad)
			steer_ = max_tracking_angle_rad;
		if (steer_ < -max_tracking_angle_rad)
			steer_ = -max_tracking_angle_rad;

		float distance = speed_ * dt;

		float turn = static_cast<float>(tan (steer_) * distance / car_len);

		if ( static_cast<float>(fabs(turn)) < 0.0001f) { // use straight line model
			cur_heading = cur_heading.rotate (turn * 180.0f / GammaParams::GAMMA_PI);
			cur_pos += cur_heading * distance;
		}else { // use bicycle model
			float yaw =  static_cast<float>(atan2(cur_heading.y(), cur_heading.x()));
			float radius = distance / turn;

			float cx = static_cast<float>(cur_pos.x() -  sin (yaw) * radius);
			float cy = static_cast<float>(cur_pos.y() +  cos (yaw) * radius);
			yaw = static_cast<float>(fmod(yaw + turn, 2 * GammaParams::GAMMA_PI));
			float new_x = static_cast<float>(cx +  sin (yaw) * radius);
			float new_y = static_cast<float>(cy -  cos (yaw) * radius);

			cur_pos = Vector2 (new_x, new_y);
			cur_heading = cur_heading.rotate (turn * 180.0f / GammaParams::GAMMA_PI);
		}
		
		pos_heading.push_back (cur_pos);
		pos_heading.push_back (cur_heading);
		return pos_heading;

	}


	// return next position and heading for tracking pref_vel by applying holonomic model for dt time
	std::vector<Vector2> RVOSimulator::holonomicMove (Vector2 cur_pos, Vector2 cur_heading, Vector2 pref_vel, float dt, float max_speed, float car_len, float max_tracking_angle_deg) {

		std::vector<Vector2> pos_heading;

		float speed_ = abs (pref_vel); //this is equivalent to making k0 infinite, where k0 is the control param for acceleration P controller: a = k0*(v* - v)

		if (speed_ > max_speed){
			speed_ = max_speed;
		} else if (speed_ < -max_speed){
			speed_ = -max_speed;
		}

		float max_tracking_angle_rad = max_tracking_angle_deg * GammaParams::GAMMA_PI / 180.0f;
			
		float steer_ = getSignedAngleRadOfTwoVector(pref_vel, cur_heading);
		if (steer_ > max_tracking_angle_rad)
			steer_ = max_tracking_angle_rad;
		if (steer_ < -max_tracking_angle_rad)
			steer_ = -max_tracking_angle_rad;

		float distance = speed_ * dt;
		
		cur_heading = cur_heading.rotate (steer_ * 180.0f / GammaParams::GAMMA_PI);
		cur_pos += cur_heading * distance;
	
		pos_heading.push_back (cur_pos);
		pos_heading.push_back (cur_heading);
		
		return pos_heading;

	}

	size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
	}

	size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}

	float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}

	float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}

	size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->agentNeighbors_.size();
	}

	size_t RVOSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_.size();
	}

	size_t RVOSimulator::getAgentNumORCALines(size_t agentNo) const
	{
		return agents_[agentNo]->orcaLines_.size();
	}

	size_t RVOSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
	}

	const Line &RVOSimulator::getAgentORCALine(size_t agentNo, size_t lineNo) const
	{
		return agents_[agentNo]->orcaLines_[lineNo];
	}

	const Vector2 &RVOSimulator::getAgentPosition(size_t agentNo) const
	{
		return agents_[agentNo]->position_;
	}

	const Vector2 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity_;
	}

	float RVOSimulator::getAgentRadius(size_t agentNo) const
	{
		return agents_[agentNo]->radius_;
	}

	float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizon_;
	}

	float RVOSimulator::getAgentTimeHorizonObst(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizonObst_;
	}

	const Vector2 &RVOSimulator::getAgentVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}

	float RVOSimulator::getGlobalTime() const
	{
		return globalTime_;
	}

	size_t RVOSimulator::getNumAgents() const
	{
		return agents_.size();
	}

	size_t RVOSimulator::getNumObstacleVertices() const
	{
		return obstacles_.size();
	}

	const Vector2 &RVOSimulator::getObstacleVertex(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->point_;
	}

	size_t RVOSimulator::getNextObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->nextObstacle_->id_;
	}

	size_t RVOSimulator::getPrevObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->prevObstacle_->id_;
	}

	float RVOSimulator::getTimeStep() const
	{
		return timeStep_;
	}

	void RVOSimulator::processObstacles()
	{
		kdTree_->buildObstacleTree();
	}

	bool RVOSimulator::queryVisibility(const Vector2 &point1, const Vector2 &point2, float radius) const
	{
		return kdTree_->queryVisibility(point1, point2, radius);
	}

	void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
	{
		if (defaultAgent_ == NULL) {
			defaultAgent_ = new Agent(this);
		}

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;
	}

	void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		agents_[agentNo]->maxNeighbors_ = maxNeighbors;
	}

	void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		agents_[agentNo]->maxSpeed_ = maxSpeed;
	}

	void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		agents_[agentNo]->neighborDist_ = neighborDist;
	}

	void RVOSimulator::setAgentPosition(size_t agentNo, const Vector2 &position)
	{
		agents_[agentNo]->position_ = position;
	}

	void RVOSimulator::setAgentPrefVelocity(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity_ = prefVelocity;
	}

	void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_ = radius;
	}

	void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
	{
		agents_[agentNo]->timeHorizon_ = timeHorizon;
	}

	void RVOSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
	{
		agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
	}

	void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector2 &velocity)
	{
		agents_[agentNo]->velocity_ = velocity;
	}

	void RVOSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}
}
