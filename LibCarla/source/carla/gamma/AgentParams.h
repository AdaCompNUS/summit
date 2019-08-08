#ifndef AGENT_PARAMS_H_
#define AGENT_PARAMS_H_

#include "Vector2.h"
#include <string>
#include <vector>
#include <unordered_map>


struct AgentParams{

	RVO::Vector2  position;
	float neighborDist;
	int maxNeighbors;
	float timeHorizon;
	float timeHorizonObst;
	float radius;
	float maxSpeed;
	RVO::Vector2 velocity;
	std::string tag;
	float max_tracking_angle;
	float len_ref_to_front;
	float len_ref_to_side;
	float len_ref_to_back;
	float len_rear_axle_to_front_axle;
	float error_bound;
	float pref_speed;

	float r_front;
	float r_rear;
	float res_dec_rate;

	AgentParams(){
		position = RVO::Vector2(0.0f, 0.0f);
		neighborDist = 8.0f;
		maxNeighbors = 5;
		timeHorizon = 3.0f;
		timeHorizonObst = 0.01f;
		radius = 0.25f;
		maxSpeed = 5.0f;
		velocity = RVO::Vector2(0.0f, 0.0f);
		tag = "People";
		max_tracking_angle = 90.0f;
		len_ref_to_front = 0.1f;
		len_ref_to_side = 0.22f;
		len_ref_to_back = 0.1f;
		len_rear_axle_to_front_axle = 0.2f;
		error_bound = 0.0f;
		pref_speed = 1.2f;

		r_front = 6.0f;
		r_rear = 0.0f;
		res_dec_rate = 0.6f;
	}


	AgentParams(RVO::Vector2 _position, float _neighborDist, int _maxNeighbors,
				float _timeHorizon, float _timeHorizonObst, float _radius, 
				float _maxSpeed, RVO::Vector2 _velocity, std::string _tag,
				float _max_tracking_angle, float _len_ref_to_front, float _len_ref_to_side,
				float _len_ref_to_back, float _len_rear_axle_to_front_axle, float _error_bound, float _pref_speed, float _r_front, float _r_rear, float _res_dec_rate){
		position = _position;
		neighborDist = _neighborDist;
		maxNeighbors = _maxNeighbors;
		timeHorizon = _timeHorizon;
		timeHorizonObst = _timeHorizonObst;
		radius = _radius;
		maxSpeed = _maxSpeed;
		velocity = _velocity;
		tag = _tag;
		max_tracking_angle = _max_tracking_angle;
		len_ref_to_front = _len_ref_to_front;
		len_ref_to_side = _len_ref_to_side;
		len_ref_to_back = _len_ref_to_back;
		len_rear_axle_to_front_axle = _len_rear_axle_to_front_axle;
		error_bound = _error_bound;
		pref_speed = _pref_speed;

		r_front = _r_front;
		r_rear = _r_rear;
		res_dec_rate = _res_dec_rate;
	}




	static const AgentParams& getDefaultAgentParam(std::string tag){

		static auto create_lookup = []() { //lambda function
			std::unordered_map<std::string, AgentParams> lookup;

			float ped_r_front_list = 6.0f; 
			float ped_r_rear_list = 2.0f; 
			float ped_res_dec_rate_list = 0.6f;

			float veh_r_front_list = 8.0f;
			float veh_r_rear_list = 4.0f;
			float veh_res_dec_rate_list = 0.01f;

			float bicycle_r_front_list = 8.0f;
			float bicycle_r_rear_list = 3.0f;
			float bicycle_res_dec_rate_list = 0.3f;

			// default setting for pedestrians
			RVO::Vector2 ped_position = RVO::Vector2(0.0f, 0.0f);
			float ped_neighborDist = 8.0f;
			int ped_maxNeighbors = 5;
			float ped_timeHorizon = 3.0f;
			float ped_timeHorizonObst = 0.01f;
			float ped_radius = 0.25f;
			float ped_maxSpeed = 5.0f;
			RVO::Vector2 ped_velocity = RVO::Vector2(0.0f, 0.0f);
			std::string ped_tag = "People";
			float ped_max_tracking_angle = 90.0f;
			float ped_len_ref_to_front = 0.1f;
			float ped_len_ref_to_side = 0.22f;
			float ped_len_ref_to_back = 0.1f;
			float ped_len_rear_axle_to_front_axle = 0.2f;
			float ped_error_bound = 0.0f;
			float ped_pref_speed = 1.2f;

			AgentParams ped = AgentParams(	ped_position, ped_neighborDist, ped_maxNeighbors, ped_timeHorizon, ped_timeHorizonObst, ped_radius, ped_maxSpeed, ped_velocity, ped_tag, 
								ped_max_tracking_angle, ped_len_ref_to_front, ped_len_ref_to_side, ped_len_ref_to_back, ped_len_rear_axle_to_front_axle, ped_error_bound, ped_pref_speed, ped_r_front_list, ped_r_rear_list, ped_res_dec_rate_list);
			

			lookup[ped_tag]=ped;


			// default setting for scooter
			RVO::Vector2 scooter_position = RVO::Vector2(0.0f, 0.0f);
			float scooter_neighborDist = 8.0f;
			int scooter_maxNeighbors = 5;
			float scooter_timeHorizon = 5.0f;
			float scooter_timeHorizonObst = 0.01f;
			float scooter_radius = 1.0f;
			float scooter_maxSpeed = 10.0f;
			RVO::Vector2 scooter_velocity = RVO::Vector2(0.0f, 0.0f);
			std::string scooter_tag = "Scooter";
			float scooter_max_tracking_angle = 45.0f;
			float scooter_len_ref_to_front = 1.25f;
			float scooter_len_ref_to_side = 0.6f;
			float scooter_len_ref_to_back = 0.4f;
			float scooter_len_rear_axle_to_front_axle = 1.0f;
			float scooter_error_bound = 0.5f;
			float scooter_pref_speed = 4.0f;

			AgentParams scooter = AgentParams(	scooter_position, scooter_neighborDist, scooter_maxNeighbors, scooter_timeHorizon, scooter_timeHorizonObst, scooter_radius, scooter_maxSpeed, scooter_velocity, scooter_tag, 
						scooter_max_tracking_angle, scooter_len_ref_to_front, scooter_len_ref_to_side, scooter_len_ref_to_back, scooter_len_rear_axle_to_front_axle, scooter_error_bound, scooter_pref_speed
						,bicycle_r_front_list, bicycle_r_rear_list, bicycle_res_dec_rate_list);
			
			lookup[scooter_tag]=scooter;

			// default setting for car
			RVO::Vector2 car_position = RVO::Vector2(0.0f, 0.0f);
			float car_neighborDist = 11.0f;
			int car_maxNeighbors = 5;
			float car_timeHorizon = 4.0f;
			float car_timeHorizonObst = 0.01f;
			float car_radius = 2.5f;
			float car_maxSpeed = 10.0f;
			RVO::Vector2 car_velocity = RVO::Vector2(0.0f, 0.0f);
			std::string car_tag = "Car";
			float car_max_tracking_angle = 45.0f;
			float car_len_ref_to_front = 3.8f;
			float car_len_ref_to_side = 1.0f;
			float car_len_ref_to_back = 1.0f;
			float car_len_rear_axle_to_front_axle = 2.7f;
			float car_error_bound = 0.5f;
			float car_pref_speed = 4.0f;

			AgentParams car = AgentParams(	car_position, car_neighborDist, car_maxNeighbors, car_timeHorizon, car_timeHorizonObst, car_radius, car_maxSpeed, car_velocity, car_tag, 
								car_max_tracking_angle, car_len_ref_to_front, car_len_ref_to_side, car_len_ref_to_back, car_len_rear_axle_to_front_axle, car_error_bound, car_pref_speed,
								veh_r_front_list, veh_r_rear_list, veh_res_dec_rate_list);
			
			lookup[car_tag]=car;


			// default setting for van
			RVO::Vector2 van_position = RVO::Vector2(0.0f, 0.0f);
			float van_neighborDist = 12.0f;
			int van_maxNeighbors = 5;
			float van_timeHorizon = 5.0f;
			float van_timeHorizonObst = 0.01f;
			float van_radius = 3.0f;
			float van_maxSpeed = 10.0f;
			RVO::Vector2 van_velocity = RVO::Vector2(0.0f, 0.0f);
			std::string van_tag = "Van";
			float van_max_tracking_angle = 45.0f;
			float van_len_ref_to_front = 4.2f;
			float van_len_ref_to_side = 1.1f;
			float van_len_ref_to_back = 1.1f;
			float van_len_rear_axle_to_front_axle = 3.91f;
			float van_error_bound = 0.5f;
			float van_pref_speed = 4.0f;

			AgentParams van = AgentParams(	van_position, van_neighborDist, van_maxNeighbors, van_timeHorizon, van_timeHorizonObst, van_radius, van_maxSpeed, van_velocity, van_tag, 
								van_max_tracking_angle, van_len_ref_to_front, van_len_ref_to_side, van_len_ref_to_back, van_len_rear_axle_to_front_axle, van_error_bound, van_pref_speed
								,veh_r_front_list, veh_r_rear_list, veh_res_dec_rate_list);
			
			lookup[van_tag]=van;


			// default setting for bus
			RVO::Vector2 bus_position = RVO::Vector2(0.0f, 0.0f);
			float bus_neighborDist = 15.0f;
			int bus_maxNeighbors = 5;
			float bus_timeHorizon = 5.0f;
			float bus_timeHorizonObst = 0.01f;
			float bus_radius = 5.0f;
			float bus_maxSpeed = 10.0f;
			RVO::Vector2 bus_velocity = RVO::Vector2(0.0f, 0.0f);
			std::string bus_tag = "Bus";
			float bus_max_tracking_angle = 45.0f;
			float bus_len_ref_to_front = 4.5f;
			float bus_len_ref_to_side = 1.5f;
			float bus_len_ref_to_back = 1.5f;
			float bus_len_rear_axle_to_front_axle = 4.0f;
			float bus_error_bound = 0.5f;
			float bus_pref_speed = 4.0f;

			AgentParams bus = AgentParams(	bus_position, bus_neighborDist, bus_maxNeighbors, bus_timeHorizon, bus_timeHorizonObst, bus_radius, bus_maxSpeed, bus_velocity, bus_tag, 
								bus_max_tracking_angle, bus_len_ref_to_front, bus_len_ref_to_side, bus_len_ref_to_back, bus_len_rear_axle_to_front_axle, bus_error_bound, bus_pref_speed
								,veh_r_front_list, veh_r_rear_list, veh_res_dec_rate_list);
			
			lookup[bus_tag]=bus;

			// default setting for Jeep
			RVO::Vector2 jeep_position = RVO::Vector2(0.0f, 0.0f);
			float jeep_neighborDist = 12.0f;
			int jeep_maxNeighbors = 5;
			float jeep_timeHorizon = 5.0f;
			float jeep_timeHorizonObst = 0.01f;
			float jeep_radius = 2.5f;
			float jeep_maxSpeed = 10.0f;
			RVO::Vector2 jeep_velocity = RVO::Vector2(0.0f, 0.0f);
			std::string jeep_tag = "Jeep";
			float jeep_max_tracking_angle = 45.0f;
			float jeep_len_ref_to_front = 4.2f;
			float jeep_len_ref_to_side = 1.1f;
			float jeep_len_ref_to_back = 1.1f;
			float jeep_len_rear_axle_to_front_axle = 3.3f;
			float jeep_error_bound = 0.5f;
			float jeep_pref_speed = 4.0f;

			AgentParams jeep = AgentParams( jeep_position, jeep_neighborDist, jeep_maxNeighbors, jeep_timeHorizon, jeep_timeHorizonObst, jeep_radius, jeep_maxSpeed, jeep_velocity, jeep_tag, 
								jeep_max_tracking_angle, jeep_len_ref_to_front, jeep_len_ref_to_side, jeep_len_ref_to_back, jeep_len_rear_axle_to_front_axle, jeep_error_bound, jeep_pref_speed
								,veh_r_front_list, veh_r_rear_list, veh_res_dec_rate_list);
			
			lookup[jeep_tag]=jeep;


			// default setting for bicycle
			RVO::Vector2 bicycle_position = RVO::Vector2(0.0f, 0.0f);
			float bicycle_neighborDist = 8.0f;
			int bicycle_maxNeighbors = 5;
			float bicycle_timeHorizon = 5.0f;
			float bicycle_timeHorizonObst = 0.01f;
			float bicycle_radius = 1.0f;
			float bicycle_maxSpeed = 5.0f;
			RVO::Vector2 bicycle_velocity = RVO::Vector2(0.0f, 0.0f);
			std::string bicycle_tag = "Bicycle";
			float bicycle_max_tracking_angle = 45.0f;
			float bicycle_len_ref_to_front = 1.25f;
			float bicycle_len_ref_to_side = 0.5f;
			float bicycle_len_ref_to_back = 0.4f;
			float bicycle_len_rear_axle_to_front_axle = 1.0f;
			float bicycle_error_bound = 0.5f;
			float bicycle_pref_speed = 2.7f;

			AgentParams bicycle = AgentParams(	bicycle_position, bicycle_neighborDist, bicycle_maxNeighbors, bicycle_timeHorizon, bicycle_timeHorizonObst, bicycle_radius, bicycle_maxSpeed, bicycle_velocity, bicycle_tag, 
								bicycle_max_tracking_angle, bicycle_len_ref_to_front, bicycle_len_ref_to_side, bicycle_len_ref_to_back, bicycle_len_rear_axle_to_front_axle, bicycle_error_bound, bicycle_pref_speed
								,bicycle_r_front_list, bicycle_r_rear_list, bicycle_res_dec_rate_list);
			
			lookup[bicycle_tag]=bicycle;

			// default setting for electric tricycle
			RVO::Vector2 electric_tricycle_position = RVO::Vector2(0.0f, 0.0f);
			float electric_tricycle_neighborDist = 8.0f;
			int electric_tricycle_maxNeighbors = 5;
			float electric_tricycle_timeHorizon = 5.0f;
			float electric_tricycle_timeHorizonObst = 0.01f;
			float electric_tricycle_radius = 1.5f;
			float electric_tricycle_maxSpeed = 8.0f;
			RVO::Vector2 electric_tricycle_velocity = RVO::Vector2(0.0f, 0.0f);
			std::string electric_tricycle_tag = "Electric_Tricycle";
			float electric_tricycle_max_tracking_angle = 45.0f;
			float electric_tricycle_len_ref_to_front = 1.25f;
			float electric_tricycle_len_ref_to_side = 0.8f;
			float electric_tricycle_len_ref_to_back = 0.4f;
			float electric_tricycle_len_rear_axle_to_front_axle = 1.0f;
			float electric_tricycle_error_bound = 0.5f;
			float electric_tricycle_pref_speed = 4.0f;

			AgentParams electric_tricycle = AgentParams(	electric_tricycle_position, electric_tricycle_neighborDist, electric_tricycle_maxNeighbors, electric_tricycle_timeHorizon, electric_tricycle_timeHorizonObst, electric_tricycle_radius, electric_tricycle_maxSpeed, electric_tricycle_velocity, electric_tricycle_tag, 
								electric_tricycle_max_tracking_angle, electric_tricycle_len_ref_to_front, electric_tricycle_len_ref_to_side, electric_tricycle_len_ref_to_back, electric_tricycle_len_rear_axle_to_front_axle, electric_tricycle_error_bound, electric_tricycle_pref_speed
								,bicycle_r_front_list, bicycle_r_rear_list, bicycle_res_dec_rate_list);
			
			lookup[electric_tricycle_tag]=electric_tricycle;


			// default setting for gyro_scooter
			RVO::Vector2 gyro_scooter_position = RVO::Vector2(0.0f, 0.0f);
			float gyro_scooter_neighborDist = 8.0f;
			int gyro_scooter_maxNeighbors = 5;
			float gyro_scooter_timeHorizon = 5.0f;
			float gyro_scooter_timeHorizonObst = 0.01f;
			float gyro_scooter_radius = 0.25f;
			float gyro_scooter_maxSpeed = 2.0f;
			RVO::Vector2 gyro_scooter_velocity = RVO::Vector2(0.0f, 0.0f);
			std::string gyro_scooter_tag = "Gyro_Scooter";
			float gyro_scooter_max_tracking_angle = 60.0f;
			float gyro_scooter_len_ref_to_front = 0.25f;
			float gyro_scooter_len_ref_to_side = 0.3f;
			float gyro_scooter_len_ref_to_back = 0.25f;
			float gyro_scooter_len_rear_axle_to_front_axle = 0.25f;
			float gyro_scooter_error_bound = 0.1f;
			float gyro_scooter_pref_speed = 2.0f;

			AgentParams gyro_scooter = AgentParams(	gyro_scooter_position, gyro_scooter_neighborDist, gyro_scooter_maxNeighbors, gyro_scooter_timeHorizon, gyro_scooter_timeHorizonObst, gyro_scooter_radius, gyro_scooter_maxSpeed, gyro_scooter_velocity, gyro_scooter_tag, 
								gyro_scooter_max_tracking_angle, gyro_scooter_len_ref_to_front, gyro_scooter_len_ref_to_side, gyro_scooter_len_ref_to_back, gyro_scooter_len_rear_axle_to_front_axle, gyro_scooter_error_bound, gyro_scooter_pref_speed
								,bicycle_r_front_list, bicycle_r_rear_list, bicycle_res_dec_rate_list);
			
			lookup[gyro_scooter_tag]=gyro_scooter;

			return lookup;
		};

		static std::unordered_map<std::string, AgentParams> lookup_ = create_lookup();

		return lookup_[tag];
	}
	
};


#endif