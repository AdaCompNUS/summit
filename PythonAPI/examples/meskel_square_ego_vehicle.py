#!/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob(os.path.abspath('%s/../../carla/dist/carla-*%d.%d-%s.egg' % (
        os.path.realpath(__file__),
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    pass

import carla
import math
import numpy as np
import argparse
import random
import cv2
import time
if sys.version_info.major == 2:
    from pathlib2 import Path
else:
    from pathlib import Path



''' ========== CONSTANTS ========== '''
DATA_PATH = Path(os.path.realpath(__file__)).parent.parent.parent/'Data'
DATASET = 'meskel_square'
SPAWN_POSITION = carla.Vector2D(360, 493)
MAX_SPEED = 8.0 # m/s
KP_THROTTLE = 0.4
KP_STEERING = 0.01



''' ========== HELPER CLASSES ========== '''
class SumoNetworkAgentPath:
    def __init__(self, route_points, min_points, interval):
        self.route_points = route_points
        self.min_points = min_points
        self.interval = interval

    def resize(self, sumo_network, rng=random):
        while len(self.route_points) < self.min_points:
            next_points = sumo_network.get_next_route_points(self.route_points[-1], self.interval)
            if len(next_points) == 0:
                return False
            self.route_points.append(rng.choice(next_points))
        return True

    def get_min_offset(self, sumo_network, position):
        min_offset = None
        for i in range(int(len(self.route_points) / 2)):
            route_point = self.route_points[i]
            offset = position - sumo_network.get_route_point_position(route_point)
            offset = offset.length() 
            if min_offset == None or offset < min_offset:
                min_offset = offset
        return min_offset

    def cut(self, sumo_network, position):
        cut_index = 0
        min_offset = None
        min_offset_index = None
        for i in range(int(len(self.route_points) / 2)):
            route_point = self.route_points[i]
            offset = position - sumo_network.get_route_point_position(route_point)
            offset = offset.length() 
            if min_offset == None or offset < min_offset:
                min_offset = offset
                min_offset_index = i
            if offset <= 1.0:
                cut_index = i + 1

        # Invalid path because too far away.
        if min_offset > 1.0:
            self.route_points = self.route_points[min_offset_index:]
        else:
            self.route_points = self.route_points[cut_index:]

    def get_position(self, sumo_network, index=0):
        return sumo_network.get_route_point_position(self.route_points[index])

    def get_yaw(self, sumo_network, index=0):
        pos = sumo_network.get_route_point_position(self.route_points[index])
        next_pos = sumo_network.get_route_point_position(self.route_points[index + 1])
        return np.rad2deg(math.atan2(next_pos.y - pos.y, next_pos.x - pos.x))



''' ========== HELPER FUNCTIONS ========== '''
def get_signed_angle_diff(vector1, vector2):
    theta = math.atan2(vector1.y, vector1.x) - math.atan2(vector2.y, vector2.x)
    theta = np.rad2deg(theta)
    if theta > 180:
        theta -= 360
    elif theta < -180:
        theta += 360
    return theta

def get_position(actor):
    pos3d = actor.get_location()
    return carla.Vector2D(pos3d.x, pos3d.y)

def get_forward_direction(actor):
    forward = actor.get_transform().get_forward_vector()
    return carla.Vector2D(forward.x, forward.y)

def get_velocity(actor):
    v = actor.get_velocity()
    return carla.Vector2D(v.x, v.y)

def get_bounding_box_corners(actor):
    bbox = actor.bounding_box
    loc = carla.Vector2D(bbox.location.x, bbox.location.y) + get_position(actor)
    forward_vec = get_forward_direction(actor).make_unit_vector()
    sideward_vec = forward_vec.rotate(np.deg2rad(90))
    half_y_len = bbox.extent.y
    half_x_len = bbox.extent.x
    corners = [loc - half_x_len * forward_vec + half_y_len * sideward_vec,
               loc + half_x_len * forward_vec + half_y_len * sideward_vec,
               loc + half_x_len * forward_vec - half_y_len * sideward_vec,
               loc - half_x_len * forward_vec - half_y_len * sideward_vec]
    return corners

def get_vehicle_bounding_box_corners(actor):
    bbox = actor.bounding_box
    loc = carla.Vector2D(bbox.location.x, bbox.location.y) + get_position(actor)
    forward_vec = get_forward_direction(actor).make_unit_vector()
    sideward_vec = forward_vec.rotate(np.deg2rad(90))
    half_y_len = bbox.extent.y + 0.3
    half_x_len_forward = bbox.extent.x + 1.0
    half_x_len_backward = bbox.extent.x + 0.1
    corners = [loc - half_x_len_backward * forward_vec + half_y_len * sideward_vec,
               loc + half_x_len_forward * forward_vec + half_y_len * sideward_vec,
               loc + half_x_len_forward * forward_vec - half_y_len * sideward_vec,
               loc - half_x_len_backward * forward_vec - half_y_len * sideward_vec]
    return corners

def get_pedestrian_bounding_box_corners(actor):
    bbox = actor.bounding_box
    loc = carla.Vector2D(bbox.location.x, bbox.location.y) + get_position(actor)
    forward_vec = get_forward_direction(actor).make_unit_vector()
    sideward_vec = forward_vec.rotate(np.deg2rad(90))
    # Hardcoded values for pedestrians.
    half_y_len = 0.25
    half_x_len = 0.25
    corners = [loc - half_x_len * forward_vec + half_y_len * sideward_vec,
               loc + half_x_len * forward_vec + half_y_len * sideward_vec,
               loc + half_x_len * forward_vec - half_y_len * sideward_vec,
               loc - half_x_len * forward_vec - half_y_len * sideward_vec]
    return corners
    
def get_lane_constraints(sidewalk, position, forward_vec):
    left_line_end = position + (1.5 + 2.0 + 0.8) * ((forward_vec.rotate(np.deg2rad(-90))).make_unit_vector())
    right_line_end = position + (1.5 + 2.0 + 0.8) * ((forward_vec.rotate(np.deg2rad(90))).make_unit_vector())
    left_lane_constrained_by_sidewalk = sidewalk.intersects(carla.Segment2D(position, left_line_end))
    right_lane_constrained_by_sidewalk = sidewalk.intersects(carla.Segment2D(position, right_line_end))
    return left_lane_constrained_by_sidewalk, right_lane_constrained_by_sidewalk

def is_car(actor):
    return isinstance(actor, carla.Vehicle) and int(actor.attributes['number_of_wheels']) > 2

def is_bike(actor):
    return isinstance(actor, carla.Vehicle) and int(actor.attributes['number_of_wheels']) == 2

def is_pedestrian(actor):
    return isinstance(actor, carla.Walker)



''' ========== MAIN FUNCTION ========== '''
def main(args):
    # Load context data.
    with (DATA_PATH/'{}.sim_bounds'.format(DATASET)).open('r') as f:
        bounds_min = carla.Vector2D(*[float(v) for v in f.readline().split(',')])
        bounds_max = carla.Vector2D(*[float(v) for v in f.readline().split(',')])
        bounds_occupancy = carla.OccupancyMap(bounds_min, bounds_max)
    sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(DATASET)))
    sumo_network_segments = sumo_network.create_segment_map()
    sumo_network_spawn_segments = sumo_network_segments.intersection(carla.OccupancyMap(bounds_min, bounds_max))
    sumo_network_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.network.wkt'.format(DATASET)))
    sidewalk = sumo_network_occupancy.create_sidewalk(1.5)

    # Connect to simulator.
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    # Generate path.
    spawn_point = sumo_network.get_nearest_route_point(SPAWN_POSITION)
    ego_path = SumoNetworkAgentPath([spawn_point], 200, 1.0)
    ego_path.resize(sumo_network)

    # Calculate transform for spawning.
    spawn_position = ego_path.get_position(sumo_network, 0)
    spawn_heading_yaw = ego_path.get_yaw(sumo_network, 0)
    spawn_transform = carla.Transform()
    spawn_transform.location.x = spawn_position.x
    spawn_transform.location.y = spawn_position.y
    spawn_transform.location.z = 1.0
    spawn_transform.rotation.yaw = spawn_heading_yaw

    # Spawn ego actor and spectator camera.
    ego_actor = world.spawn_actor(
            world.get_blueprint_library().filter('vehicle.audi.etron')[0],
            spawn_transform)

    # Main control loop.
    while True:
        # Prepare GAMMA instance.
        gamma = carla.RVOSimulator()

        # Add exo-agents to GAMMA.
        gamma_id = 0
        for actor in world.get_actors():
            if actor.id != ego_actor.id:
                if isinstance(actor, carla.Vehicle):
                    if is_bike(actor):
                        type_tag = 'Bicycle'
                    else:
                        type_tag = 'Car'
                    bounding_box_corners = get_vehicle_bounding_box_corners(actor)
                elif isinstance(actor, carla.Walker):
                    type_tag = 'People'
                    bounding_box_corners = get_pedestrian_bounding_box_corners(actor)
                else:
                    continue

                gamma.add_agent(carla.AgentParams.get_default(type_tag), gamma_id)
                gamma.set_agent_position(gamma_id, get_position(actor))
                gamma.set_agent_velocity(gamma_id, get_velocity(actor))
                gamma.set_agent_heading(gamma_id, get_forward_direction(actor))
                gamma.set_agent_bounding_box_corners(gamma_id, bounding_box_corners)
                gamma.set_agent_pref_velocity(gamma_id, get_velocity(actor))
                gamma_id += 1
        
        # Extend ego path.
        ego_path.cut(sumo_network, get_position(ego_actor))
        ego_path.resize(sumo_network)
       
        # Add ego-agent to GAMMA.
        ego_id = gamma_id
        gamma.add_agent(carla.AgentParams.get_default('Car'), ego_id)
        gamma.set_agent_position(ego_id, get_position(ego_actor))
        gamma.set_agent_velocity(ego_id, get_velocity(ego_actor))
        gamma.set_agent_heading(ego_id, get_forward_direction(ego_actor))
        gamma.set_agent_bounding_box_corners(ego_id, get_vehicle_bounding_box_corners(ego_actor))
        target_position = ego_path.get_position(sumo_network, 5)
        pref_vel = MAX_SPEED * (target_position - get_position(ego_actor)).make_unit_vector()
        path_forward = (ego_path.get_position(sumo_network,1) - 
                            ego_path.get_position(sumo_network, 0)).make_unit_vector()
        gamma.set_agent_pref_velocity(ego_id, pref_vel)
        gamma.set_agent_path_forward(ego_id, path_forward)
        left_line_end = get_position(ego_actor) + (1.5 + 2.0 + 0.8) * ((get_forward_direction(ego_actor).rotate(np.deg2rad(-90))).make_unit_vector())
        right_line_end = get_position(ego_actor) + (1.5 + 2.0 + 0.8) * ((get_forward_direction(ego_actor).rotate(np.deg2rad(90))).make_unit_vector())
        left_lane_constrained_by_sidewalk = sidewalk.intersects(carla.Segment2D(get_position(ego_actor), left_line_end))
        right_lane_constrained_by_sidewalk = sidewalk.intersects(carla.Segment2D(get_position(ego_actor), right_line_end))
        # Flip left-right -> right-left since GAMMA uses a different handed coordinate system.
        gamma.set_agent_lane_constraints(ego_id, right_lane_constrained_by_sidewalk, left_lane_constrained_by_sidewalk)  

        # Step GAMMA, and retrieve ego-agent's target velocity.
        gamma.do_step()
        target_vel = gamma.get_agent_velocity(ego_id)

        # Calculate ego-vehicle control.
        control = carla.VehicleControl()
        control.throttle = KP_THROTTLE * (target_vel.length() - get_velocity(ego_actor).length())
        control.steer = KP_STEERING * get_signed_angle_diff(target_vel, get_velocity(ego_actor))
        control.manual_gear_shift = True # Reduces transmission lag.
        control.gear = 1 # Reduces transmission lag.
        ego_actor.apply_control(control)

        time.sleep(0.1)
        

if __name__ == '__main__':
    # Parse arguments.
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()
    main(args)
