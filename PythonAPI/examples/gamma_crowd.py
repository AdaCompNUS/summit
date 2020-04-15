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

os.environ["PYRO_LOGFILE"] = "pyro.log"
os.environ["PYRO_LOGLEVEL"] = "DEBUG"

from collections import defaultdict
from multiprocessing import Process
from threading import RLock
import Pyro4
import argparse
import carla
import math
import numpy as np
import random
import time
if sys.version_info.major == 2:
    from pathlib2 import Path
else:
    from pathlib import Path



''' ========== CONSTANTS ========== '''
DATA_PATH = Path(os.path.realpath(__file__)).parent.parent.parent/'Data'
PATH_MIN_POINTS = 20
PATH_INTERVAL = 1.0

SPAWN_DESTROY_MAX_RATE = 15.0
GAMMA_MAX_RATE = 40.0
CONTROL_MAX_RATE = 20.0
COLLISION_STATISTICS_MAX_RATE = 5.0
SPAWN_DESTROY_REPETITIONS = 3

CAR_SPEED_KP = 1.2 * 0.8 # 1.5
CAR_SPEED_KI = 0.5 * 0.8
CAR_SPEED_KD = 0.2 * 0.8 # 0.005
CAR_STEER_KP = 1.5 # 2.5

BIKE_SPEED_KP = 1.2 * 0.8 # 1.5
BIKE_SPEED_KI = 0.5 * 0.8
BIKE_SPEED_KD = 0.2 * 0.8 # 0.005
BIKE_STEER_KP = 1.5 # 2.5

Pyro4.config.SERIALIZERS_ACCEPTED.add('serpent')
Pyro4.config.SERIALIZER = 'serpent'
Pyro4.util.SerializerBase.register_class_to_dict(
        carla.Vector2D, 
        lambda o: { 
            '__class__': 'carla.Vector2D',
            'x': o.x,
            'y': o.y
        })
Pyro4.util.SerializerBase.register_dict_to_class(
        'carla.Vector2D',
        lambda c, o: carla.Vector2D(o['x'], o['y']))
Pyro4.util.SerializerBase.register_class_to_dict(
        carla.SumoNetworkRoutePoint,    
        lambda o: {     
            '__class__': 'carla.SumoNetworkRoutePoint', 
            'edge': o.edge, 
            'lane': o.lane, 
            'segment': o.segment,   
            'offset': o.offset  
        })
def dict_to_sumo_network_route_point(c, o): 
    r = carla.SumoNetworkRoutePoint()   
    r.edge = str(o['edge']) # In python2, this is a unicode, so use str() to convert.
    r.lane = o['lane']  
    r.segment = o['segment']    
    r.offset = o['offset']  
    return r    
Pyro4.util.SerializerBase.register_dict_to_class(   
        'carla.SumoNetworkRoutePoint', dict_to_sumo_network_route_point)    
Pyro4.util.SerializerBase.register_class_to_dict(   
        carla.SidewalkRoutePoint,   
        lambda o: {     
            '__class__': 'carla.SidewalkRoutePoint',    
            'polygon_id': o.polygon_id, 
            'segment_id': o.segment_id, 
            'offset': o.offset  
        })  
def dict_to_sidewalk_route_point(c, o): 
    r = carla.SidewalkRoutePoint()  
    r.polygon_id = o['polygon_id']  
    r.segment_id = o['segment_id']  
    r.offset = o['offset']
    return r    
Pyro4.util.SerializerBase.register_dict_to_class(   
        'carla.SidewalkRoutePoint', dict_to_sidewalk_route_point)



''' ========== MESSAGE PASSING SERVICE ========== '''
@Pyro4.expose
@Pyro4.behavior(instance_mode="single")
class CrowdService():
    def __init__(self):
        self._simulation_bounds_min = None
        self._simulation_bounds_max = None
        self._simulation_bounds_lock = RLock()

        self._forbidden_bounds_min = None
        self._forbidden_bounds_max = None
        self._forbidden_bounds_lock = RLock()

        self._spawn_car = False
        self._new_cars = []
        self._new_cars_lock = RLock()

        self._spawn_bike = False
        self._new_bikes = []
        self._new_bikes_lock = RLock()

        self._spawn_pedestrian = False
        self._new_pedestrians = []
        self._new_pedestrians_lock = RLock()

        self._control_velocities = []
        self._control_velocities_lock = RLock()

        self._local_intentions = []
        self._local_intentions_lock = RLock()

        self._destroy_list = []
        self._destroy_list_lock = RLock()

    @property
    def simulation_bounds(self):
        self._simulation_bounds_lock.acquire()
        simulation_bounds_min = None if self._simulation_bounds_min is None else \
                carla.Vector2D(self._simulation_bounds_min.x, self._simulation_bounds_min.y)
        simulation_bounds_max = None if self._simulation_bounds_max is None else \
                carla.Vector2D(self._simulation_bounds_max.x, self._simulation_bounds_max.y)
        self._simulation_bounds_lock.release()
        return (simulation_bounds_min, simulation_bounds_max)

    @simulation_bounds.setter
    def simulation_bounds(self, bounds):
        self._simulation_bounds_lock.acquire()
        self._simulation_bounds_min = bounds[0]
        self._simulation_bounds_max = bounds[1]
        self._simulation_bounds_lock.release() 
   

    @property
    def forbidden_bounds(self):
        self._forbidden_bounds_lock.acquire()
        forbidden_bounds_min = None if self._forbidden_bounds_min is None else \
                carla.Vector2D(self._forbidden_bounds_min.x, self._forbidden_bounds_min.y)
        forbidden_bounds_max = None if self._forbidden_bounds_max is None else \
                carla.Vector2D(self._forbidden_bounds_max.x, self._forbidden_bounds_max.y)
        self._forbidden_bounds_lock.release()
        return (forbidden_bounds_min, forbidden_bounds_max)

    @forbidden_bounds.setter
    def forbidden_bounds(self, bounds):
        self._forbidden_bounds_lock.acquire()
        self._forbidden_bounds_min = bounds[0]
        self._forbidden_bounds_max = bounds[1]
        self._forbidden_bounds_lock.release() 


    @property
    def spawn_car(self):
        return self._spawn_car

    @spawn_car.setter
    def spawn_car(self, value):
        self._spawn_car = value

    @property
    def new_cars(self):
        return self._new_cars

    @new_cars.setter
    def new_cars(self, cars):
        self._new_cars = cars
    
    def append_new_cars(self, info):
        self._new_cars.append(info)

    def acquire_new_cars(self):
        self._new_cars_lock.acquire()

    def release_new_cars(self):
        try:
            self._new_cars_lock.release()
        except Exception as e:
            print(e)
            sys.stdout.flush()
   

    @property
    def spawn_bike(self):
        return self._spawn_bike

    @spawn_bike.setter
    def spawn_bike(self, value):
        self._spawn_bike = value

    @property
    def new_bikes(self):
        return self._new_bikes

    @new_bikes.setter
    def new_bikes(self, bikes):
        self._new_bikes = bikes
    
    def append_new_bikes(self, info):
        self._new_bikes.append(info)
    
    def acquire_new_bikes(self):
        self._new_bikes_lock.acquire()

    def release_new_bikes(self):
        try:
            self._new_bikes_lock.release()
        except Exception as e:
            print(e)
            sys.stdout.flush()
    
    
    @property
    def spawn_pedestrian(self):
        return self._spawn_pedestrian

    @spawn_pedestrian.setter
    def spawn_pedestrian(self, value):
        self._spawn_pedestrian = value

    @property
    def new_pedestrians(self):
        return self._new_pedestrians

    @new_pedestrians.setter
    def new_pedestrians(self, pedestrians):
        self._new_pedestrians = pedestrians
    
    def append_new_pedestrians(self, info):
        self._new_pedestrians.append(info)
    
    def acquire_new_pedestrians(self):
        self._new_pedestrians_lock.acquire()

    def release_new_pedestrians(self):
        try:
            self._new_pedestrians_lock.release()
        except Exception as e:
            print(e)
            sys.stdout.flush()


    @property
    def control_velocities(self):
        return self._control_velocities

    @control_velocities.setter
    def control_velocities(self, velocities):
        self._control_velocities = velocities

    def acquire_control_velocities(self):
        self._control_velocities_lock.acquire()

    def release_control_velocities(self):
        try:
            self._control_velocities_lock.release()
        except Exception as e:
            print(e)
            sys.stdout.flush()

    @property
    def local_intentions(self):
        return self._local_intentions

    @local_intentions.setter
    def local_intentions(self, velocities):
        self._local_intentions = velocities

    def acquire_local_intentions(self):
        self._local_intentions_lock.acquire()

    def release_local_intentions(self):
        try:
            self._local_intentions_lock.release()
        except Exception as e:
            print(e)
            sys.stdout.flush()
   

    @property
    def destroy_list(self):
        return self._destroy_list

    @destroy_list.setter
    def destroy_list(self, items):
        self._destroy_list = items
    
    def append_destroy_list(self, item):
        self._destroy_list.append(item)
    
    def extend_destroy_list(self, items):
        self._destroy_list.extend(items)

    def acquire_destroy_list(self):
        self._destroy_list_lock.acquire()

    def release_destroy_list(self):
        try:
            self._destroy_list_lock.release()
        except Exception as e:
            print(e)
            sys.stdout.flush()



''' ========== UTILITY FUNCTIONS AND CLASSES ========== '''
def get_signed_angle_diff(vector1, vector2):
    theta = math.atan2(vector1.y, vector1.x) - math.atan2(vector2.y, vector2.x)
    theta = np.rad2deg(theta)
    if theta > 180:
        theta -= 360
    elif theta < -180:
        theta += 360
    return theta

def get_steer_angle_range(actor):
    actor_physics_control = actor.get_physics_control()
    return (actor_physics_control.wheels[0].max_steer_angle + actor_physics_control.wheels[1].max_steer_angle) / 2

def get_position(actor):
    pos3d = actor.get_location()
    return carla.Vector2D(pos3d.x, pos3d.y)

def get_forward_direction(actor):
    forward = actor.get_transform().get_forward_vector()
    return carla.Vector2D(forward.x, forward.y)

def get_bounding_box(actor):
    return actor.bounding_box

def get_position_3d(actor):
    return actor.get_location()

def get_aabb(actor):
    bbox = actor.bounding_box
    loc = carla.Vector2D(bbox.location.x, bbox.location.y) + get_position(actor)
    forward_vec = get_forward_direction(actor).make_unit_vector()
    sideward_vec = forward_vec.rotate(np.deg2rad(90))
    corners = [loc - bbox.extent.x * forward_vec + bbox.extent.y * sideward_vec,
               loc + bbox.extent.x * forward_vec + bbox.extent.y * sideward_vec,
               loc + bbox.extent.x * forward_vec - bbox.extent.y * sideward_vec,
               loc - bbox.extent.x * forward_vec - bbox.extent.y * sideward_vec]
    return carla.AABB2D(
        carla.Vector2D(
            min(v.x for v in corners),
            min(v.y for v in corners)),
        carla.Vector2D(
            max(v.x for v in corners),
            max(v.y for v in corners)))

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

class SumoNetworkAgentPath:
    def __init__(self, route_points, min_points, interval):
        self.route_points = route_points
        self.min_points = min_points
        self.interval = interval

    @staticmethod
    def rand_path(sumo_network, min_points, interval, segment_map, rng=random):
        spawn_point = None
        route_paths = None
        while not spawn_point or len(route_paths) < 1:
            spawn_point = segment_map.rand_point()
            spawn_point = sumo_network.get_nearest_route_point(spawn_point)
            route_paths = sumo_network.get_next_route_paths(spawn_point, min_points - 1, interval)

        return SumoNetworkAgentPath(rng.choice(route_paths), min_points, interval)

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

class SidewalkAgentPath:
    def __init__(self, route_points, route_orientations, min_points, interval):
        self.min_points = min_points
        self.interval = interval
        self.route_points = route_points
        self.route_orientations = route_orientations

    @staticmethod
    def rand_path(sidewalk, min_points, interval, cross_probability, segment_map, rng=None):
        if rng is None:
            rng = random
    
        spawn_point = sidewalk.get_nearest_route_point(segment_map.rand_point())

        path = SidewalkAgentPath([spawn_point], [rng.choice([True, False])], min_points, interval)
        path.resize(sidewalk, cross_probability)
        return path

    def resize(self, sidewalk, cross_probability, rng=None):
        if rng is None:
            rng = random

        while len(self.route_points) < self.min_points:
            if rng.random() <= cross_probability:
                adjacent_route_point = sidewalk.get_adjacent_route_point(self.route_points[-1], 50.0)
                if adjacent_route_point is not None:
                    self.route_points.append(adjacent_route_point)
                    self.route_orientations.append(rng.randint(0, 1) == 1)
                    continue

            if self.route_orientations[-1]:
                self.route_points.append(
                        sidewalk.get_next_route_point(self.route_points[-1], self.interval))
                self.route_orientations.append(True)
            else:
                self.route_points.append(
                        sidewalk.get_previous_route_point(self.route_points[-1], self.interval))
                self.route_orientations.append(False)

        return True

    def cut(self, sidewalk, position):
        cut_index = 0
        min_offset = None
        min_offset_index = None
        for i in range(int(len(self.route_points) / 2)):
            route_point = self.route_points[i]
            offset = position - sidewalk.get_route_point_position(route_point)
            offset = offset.length()
            if min_offset is None or offset < min_offset:
                min_offset = offset
                min_offset_index = i
            if offset <= 1.0:
                cut_index = i + 1
        
        # Invalid path because too far away.
        if min_offset > 1.0:
            self.route_points = self.route_points[min_offset_index:]
            self.route_orientations = self.route_orientations[min_offset_index:]
        else:
            self.route_points = self.route_points[cut_index:]
            self.route_orientations = self.route_orientations[cut_index:]

    def get_position(self, sidewalk, index=0):
        return sidewalk.get_route_point_position(self.route_points[index])

    def get_yaw(self, sidewalk, index=0):
        pos = sidewalk.get_route_point_position(self.route_points[index])
        next_pos = sidewalk.get_route_point_position(self.route_points[index + 1])
        return np.rad2deg(math.atan2(next_pos.y - pos.y, next_pos.x - pos.x))

class Agent(object):
    def __init__(self, actor, type_tag, path, preferred_speed, steer_angle_range=0.0, rand=0):
        self.actor = actor
        self.type_tag = type_tag
        self.path = path
        self.preferred_speed = preferred_speed
        self.stuck_time = None
        self.control_velocity = carla.Vector2D(0, 0)
        self.steer_angle_range = steer_angle_range
        self.behavior_type = self.rand_agent_behavior_type(rand)

    def rand_agent_behavior_type(self, prob):
        prob_gamma_agent = 1.0
        prob_simplified_gamma_agent = 0.0
        prob_ttc_agent = 0.0

        if prob <= prob_gamma_agent:
            return carla.AgentBehaviorType.Gamma
        elif prob <= prob_gamma_agent + prob_simplified_gamma_agent:
            return carla.AgentBehaviorType.SimplifiedGamma
        else:
            return -1


class Context(object):
    def __init__(self, args):
        self.args = args
        self.rng = random.Random(args.seed)

        with (DATA_PATH/'{}.sim_bounds'.format(args.dataset)).open('r') as f:
            self.bounds_min = carla.Vector2D(*[float(v) for v in f.readline().split(',')])
            self.bounds_max = carla.Vector2D(*[float(v) for v in f.readline().split(',')])
            self.bounds_occupancy = carla.OccupancyMap(self.bounds_min, self.bounds_max)
        
        self.forbidden_bounds_min = None
        self.forbidden_bounds_max = None
        self.forbidden_bounds_occupancy = None

        self.sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(args.dataset)))
        self.sumo_network_segments = self.sumo_network.create_segment_map()
        self.sumo_network_spawn_segments = self.sumo_network_segments.intersection(carla.OccupancyMap(self.bounds_min, self.bounds_max))
        self.sumo_network_spawn_segments.seed_rand(self.rng.getrandbits(32))
        self.sumo_network_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.network.wkt'.format(args.dataset)))

        self.sidewalk = self.sumo_network_occupancy.create_sidewalk(1.5)
        self.sidewalk_segments = self.sidewalk.create_segment_map()
        self.sidewalk_spawn_segments = self.sidewalk_segments.intersection(carla.OccupancyMap(self.bounds_min, self.bounds_max))
        self.sidewalk_spawn_segments.seed_rand(self.rng.getrandbits(32))
        self.sidewalk_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.sidewalk.wkt'.format(args.dataset)))

        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.crowd_service = Pyro4.Proxy('PYRO:crowdservice.warehouse@localhost:{}'.format(args.pyroport))
    
        self.pedestrian_blueprints = self.world.get_blueprint_library().filter('walker.pedestrian.*')
        self.vehicle_blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        self.car_blueprints = [x for x in self.vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        self.car_blueprints = [x for x in self.car_blueprints if x.id not in ['vehicle.bmw.isetta']] # This dude moves too slow.
        self.bike_blueprints = [x for x in self.vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 2]

class Statistics(object):
    def __init__(self, log_file):
        self.start_time = None

        self.total_num_cars = 0
        self.total_num_bikes = 0
        self.total_num_pedestrians = 0

        self.stuck_num_cars = 0
        self.stuck_num_bikes = 0
        self.stuck_num_pedestrians = 0

        self.avg_speed_cars = 0
        self.avg_speed_bikes = 0
        self.avg_speed_pedestrians = 0

        self.log_file = log_file

    def write(self):
        self.log_file.write('{} {} {} {} {} {} {} {} {} {}\n'.format(
            time.time() - self.start_time,
            self.total_num_cars, 
            self.total_num_bikes, 
            self.total_num_pedestrians,
            self.stuck_num_cars, 
            self.stuck_num_bikes, 
            self.stuck_num_pedestrians,
            self.avg_speed_cars,
            self.avg_speed_bikes,
            self.avg_speed_pedestrians))
        self.log_file.flush()
        os.fsync(self.log_file)


''' ========== MAIN LOGIC FUNCTIONS ========== '''
def do_spawn(c):
    
    c.crowd_service.acquire_new_cars()
    spawn_car = c.crowd_service.spawn_car
    c.crowd_service.release_new_cars()
    
    c.crowd_service.acquire_new_bikes()
    spawn_bike = c.crowd_service.spawn_bike
    c.crowd_service.release_new_bikes()
    
    c.crowd_service.acquire_new_pedestrians()
    spawn_pedestrian = c.crowd_service.spawn_pedestrian
    c.crowd_service.release_new_pedestrians()

    if not spawn_car and not spawn_bike and not spawn_pedestrian:
        return

    # Find car spawn point.
    if spawn_car:
        aabb_occupancy = carla.OccupancyMap() if c.forbidden_bounds_occupancy is None else c.forbidden_bounds_occupancy
        for actor in c.world.get_actors():
            if isinstance(actor, carla.Vehicle) or isinstance(actor, carla.Walker):
                aabb = get_aabb(actor)
                aabb_occupancy = aabb_occupancy.union(carla.OccupancyMap(
                    carla.Vector2D(aabb.bounds_min.x - c.args.clearance_car, aabb.bounds_min.y - c.args.clearance_car), 
                    carla.Vector2D(aabb.bounds_max.x + c.args.clearance_car, aabb.bounds_max.y + c.args.clearance_car)))

        for _ in range(SPAWN_DESTROY_REPETITIONS):
            spawn_segments = c.sumo_network_spawn_segments.difference(aabb_occupancy)
            spawn_segments.seed_rand(c.rng.getrandbits(32))

            path = SumoNetworkAgentPath.rand_path(c.sumo_network, PATH_MIN_POINTS, PATH_INTERVAL, spawn_segments, rng=c.rng)
            position = path.get_position(c.sumo_network, 0)
            trans = carla.Transform()
            trans.location.x = position.x
            trans.location.y = position.y
            trans.location.z = 0.2
            trans.rotation.yaw = path.get_yaw(c.sumo_network, 0)

            actor = c.world.try_spawn_actor(c.rng.choice(c.car_blueprints), trans)
            if actor:
                actor.set_collision_enabled(c.args.collision)
                c.world.wait_for_tick(1.0)  # For actor to update pos and bounds, and for collision to apply.
                c.crowd_service.acquire_new_cars()
                c.crowd_service.append_new_cars((
                    actor.id, 
                    [p for p in path.route_points], # Convert to python list.
                    get_steer_angle_range(actor)))
                c.crowd_service.release_new_cars()
                aabb = get_aabb(actor)
                aabb_occupancy = aabb_occupancy.union(carla.OccupancyMap(
                    carla.Vector2D(aabb.bounds_min.x - c.args.clearance_car, aabb.bounds_min.y - c.args.clearance_car), 
                    carla.Vector2D(aabb.bounds_max.x + c.args.clearance_car, aabb.bounds_max.y + c.args.clearance_car)))


    # Find bike spawn point.
    if spawn_bike:
        aabb_occupancy = carla.OccupancyMap() if c.forbidden_bounds_occupancy is None else c.forbidden_bounds_occupancy
        for actor in c.world.get_actors():
            if isinstance(actor, carla.Vehicle) or isinstance(actor, carla.Walker):
                aabb = get_aabb(actor)
                aabb_occupancy = aabb_occupancy.union(carla.OccupancyMap(
                    carla.Vector2D(aabb.bounds_min.x - c.args.clearance_bike, aabb.bounds_min.y - c.args.clearance_bike), 
                    carla.Vector2D(aabb.bounds_max.x + c.args.clearance_bike, aabb.bounds_max.y + c.args.clearance_bike)))
        
        for _ in range(SPAWN_DESTROY_REPETITIONS):
            spawn_segments = c.sumo_network_spawn_segments.difference(aabb_occupancy)
            spawn_segments.seed_rand(c.rng.getrandbits(32))

            path = SumoNetworkAgentPath.rand_path(c.sumo_network, PATH_MIN_POINTS, PATH_INTERVAL, spawn_segments, rng=c.rng)
            position = path.get_position(c.sumo_network, 0)
            trans = carla.Transform()
            trans.location.x = position.x
            trans.location.y = position.y
            trans.location.z = 0.2
            trans.rotation.yaw = path.get_yaw(c.sumo_network, 0)

            actor = c.world.try_spawn_actor(c.rng.choice(c.bike_blueprints), trans)
            if actor:
                actor.set_collision_enabled(c.args.collision)
                c.world.wait_for_tick(1.0)  # For actor to update pos and bounds, and for collision to apply.
                c.crowd_service.acquire_new_bikes()
                c.crowd_service.append_new_bikes((
                    actor.id, 
                    [p for p in path.route_points], # Convert to python list.
                    get_steer_angle_range(actor)))
                c.crowd_service.release_new_bikes()
                aabb = get_aabb(actor)
                aabb_occupancy = aabb_occupancy.union(carla.OccupancyMap(
                    carla.Vector2D(aabb.bounds_min.x - c.args.clearance_bike, aabb.bounds_min.y - c.args.clearance_bike), 
                    carla.Vector2D(aabb.bounds_max.x + c.args.clearance_bike, aabb.bounds_max.y + c.args.clearance_bike)))


    if spawn_pedestrian:
        aabb_occupancy = carla.OccupancyMap() if c.forbidden_bounds_occupancy is None else c.forbidden_bounds_occupancy
        for actor in c.world.get_actors():
            if isinstance(actor, carla.Vehicle) or isinstance(actor, carla.Walker):
                aabb = get_aabb(actor)
                aabb_occupancy = aabb_occupancy.union(carla.OccupancyMap(
                    carla.Vector2D(aabb.bounds_min.x - c.args.clearance_pedestrian, aabb.bounds_min.y - c.args.clearance_pedestrian), 
                    carla.Vector2D(aabb.bounds_max.x + c.args.clearance_pedestrian, aabb.bounds_max.y + c.args.clearance_pedestrian)))
        
        for _ in range(SPAWN_DESTROY_REPETITIONS):
            spawn_segments = c.sidewalk_spawn_segments.difference(aabb_occupancy)
            spawn_segments.seed_rand(c.rng.getrandbits(32))

            path = SidewalkAgentPath.rand_path(c.sidewalk, PATH_MIN_POINTS, PATH_INTERVAL, c.args.cross_probability, c.sidewalk_spawn_segments, c.rng)
            position = path.get_position(c.sidewalk, 0)
            trans = carla.Transform()
            trans.location.x = position.x
            trans.location.y = position.y
            trans.location.z = 0.5
            trans.rotation.yaw = path.get_yaw(c.sidewalk, 0)
            actor = c.world.try_spawn_actor(c.rng.choice(c.pedestrian_blueprints), trans)
            if actor:
                actor.set_collision_enabled(c.args.collision)
                c.world.wait_for_tick(1.0)  # For actor to update pos and bounds, and for collision to apply.
                c.crowd_service.acquire_new_pedestrians()
                c.crowd_service.append_new_pedestrians((
                    actor.id, 
                    [p for p in path.route_points], # Convert to python list.
                    path.route_orientations))
                c.crowd_service.release_new_pedestrians()
                aabb = get_aabb(actor)
                aabb_occupancy = aabb_occupancy.union(carla.OccupancyMap(
                    carla.Vector2D(aabb.bounds_min.x - c.args.clearance_pedestrian, aabb.bounds_min.y - c.args.clearance_pedestrian), 
                    carla.Vector2D(aabb.bounds_max.x + c.args.clearance_pedestrian, aabb.bounds_max.y + c.args.clearance_pedestrian)))


def do_destroy(c):
    
    c.crowd_service.acquire_destroy_list()
    destroy_list = c.crowd_service.destroy_list
    c.crowd_service.destroy_list = []
    c.crowd_service.release_destroy_list()

    commands = [carla.command.DestroyActor(x) for x in destroy_list]

    c.client.apply_batch_sync(commands)
    c.world.wait_for_tick(1.0)


def pull_new_agents(c, car_agents, bike_agents, pedestrian_agents, statistics):
    
    new_car_agents = []
    new_bike_agents = []
    new_pedestrian_agents = []

    c.crowd_service.acquire_new_cars()
    for (actor_id, route_points, steer_angle_range) in c.crowd_service.new_cars:
        path = SumoNetworkAgentPath(route_points, PATH_MIN_POINTS, PATH_INTERVAL)
        new_car_agents.append(Agent(
            c.world.get_actor(actor_id), 'Car', path, 5.0 + c.rng.uniform(-0.5, 0.5),
            steer_angle_range, rand=c.rng.uniform(0.0, 1.0)))
    c.crowd_service.new_cars = []
    c.crowd_service.spawn_car = len(car_agents) < c.args.num_car
    c.crowd_service.release_new_cars()
    
    c.crowd_service.acquire_new_bikes()
    for (actor_id, route_points, steer_angle_range) in c.crowd_service.new_bikes:
        path = SumoNetworkAgentPath(route_points, PATH_MIN_POINTS, PATH_INTERVAL)
        new_bike_agents.append(Agent(
            c.world.get_actor(actor_id), 'Bicycle', path, 3.0 + c.rng.uniform(-0.5, 0.5),
            steer_angle_range, rand=c.rng.uniform(0.0, 1.0)))
    c.crowd_service.new_bikes = []
    c.crowd_service.spawn_bike = len(bike_agents) < c.args.num_bike
    c.crowd_service.release_new_bikes()
    
    c.crowd_service.acquire_new_pedestrians()
    for (actor_id, route_points, route_orientations) in c.crowd_service.new_pedestrians:
        path = SidewalkAgentPath(route_points, route_orientations, PATH_MIN_POINTS, PATH_INTERVAL)
        path.resize(c.sidewalk, c.args.cross_probability)
        new_pedestrian_agents.append(Agent(
            c.world.get_actor(actor_id), 'People', path, 1.0 + c.rng.uniform(-0.5, 0.5),
            rand=c.rng.uniform(0.0, 1.0)))
    c.crowd_service.new_pedestrians = []
    c.crowd_service.spawn_pedestrian = len(pedestrian_agents) < c.args.num_pedestrian
    c.crowd_service.release_new_pedestrians()

    statistics.total_num_cars += len(new_car_agents)
    statistics.total_num_bikes += len(new_bike_agents)
    statistics.total_num_pedestrians += len(new_pedestrian_agents)

    return (car_agents + new_car_agents, bike_agents + new_bike_agents, pedestrian_agents + new_pedestrian_agents, statistics)


def do_death(c, car_agents, bike_agents, pedestrian_agents, destroy_list, statistics):
   
    update_time = time.time()

    next_car_agents = []
    next_bike_agents = []
    next_pedestrian_agents = []
    new_destroy_list = []

    for (agents, next_agents) in zip([car_agents, bike_agents, pedestrian_agents], [next_car_agents, next_bike_agents, next_pedestrian_agents]):
        for agent in agents:
            delete = False
            if not delete and not c.bounds_occupancy.contains(get_position(agent.actor)):
                delete = True
            if not delete and get_position_3d(agent.actor).z < -10:
                delete = True
            if not delete and \
                    ((agent.type_tag in ['Car', 'Bicycle']) and not c.sumo_network_occupancy.contains(get_position(agent.actor))):
                delete = True
            if not delete and \
                    len(agent.path.route_points) < agent.path.min_points:
                delete = True
            if get_velocity(agent.actor).length() < c.args.stuck_speed:
                if agent.stuck_time is not None:
                    if update_time - agent.stuck_time >= c.args.stuck_duration:
                        if agents == car_agents:
                            statistics.stuck_num_cars += 1
                        elif agents == bike_agents:
                            statistics.stuck_num_bikes += 1
                        elif agents == pedestrian_agents:
                            statistics.stuck_num_pedestrians += 1
                        delete = True
                else:
                    agent.stuck_time = update_time
            else:
                agent.stuck_time = None
            
            if delete:
                new_destroy_list.append(agent.actor.id)
            else:
                next_agents.append(agent)

    return (next_car_agents, next_bike_agents, next_pedestrian_agents, destroy_list + new_destroy_list, statistics)


def do_speed_statistics(c, car_agents, bike_agents, pedestrian_agents, statistics):
    avg_speed_cars = 0.0
    avg_speed_bikes = 0.0
    avg_speed_pedestrians = 0.0

    for agent in car_agents:
        avg_speed_cars += get_velocity(agent.actor).length()

    for agent in bike_agents:
        avg_speed_bikes += get_velocity(agent.actor).length()

    for agent in pedestrian_agents:
        avg_speed_pedestrians += get_velocity(agent.actor).length()

    if len(car_agents) > 0:
        avg_speed_cars /= len(car_agents)

    if len(bike_agents) > 0:
        avg_speed_bikes /= len(bike_agents)
        
    if len(pedestrian_agents) > 0:
        avg_speed_pedestrians /= len(pedestrian_agents)

    statistics.avg_speed_cars = avg_speed_cars
    statistics.avg_speed_bikes = avg_speed_bikes
    statistics.avg_speed_pedestrians = avg_speed_pedestrians

    return statistics


def do_collision_statistics(c, timestamp, log_file):
    actors = c.world.get_actors()
    actors = [a for a in actors if is_car(a) or is_bike(a) or is_pedestrian(a)]
    bounding_boxes = [carla.OccupancyMap(get_bounding_box_corners(actor)) for actor in actors]
    collisions = [0 for _ in range(len(actors))]

    for i in range(len(actors) - 1):
        if collisions[i] == 1:
            continue
        for j in range(i + 1, len(actors)):
            if bounding_boxes[i].intersects(bounding_boxes[j]):
                collisions[i] = 1
                collisions[j] = 1

    log_file.write('{} {}\n'.format(timestamp, 0.0 if len(collisions) == 0 else float(sum(collisions)) / len(collisions)))
    log_file.flush()
    os.fsync(log_file)


def do_gamma(c, car_agents, bike_agents, pedestrian_agents, destroy_list):
    agents = car_agents + bike_agents + pedestrian_agents
    agents_lookup = {}
    for agent in agents:
        agents_lookup[agent.actor.id] = agent

    next_agents = []
    next_agent_gamma_ids = []
    new_destroy_list = []
    if len(agents) > 0:
        gamma = carla.RVOSimulator()
        
        gamma_id = 0

         # For external agents not tracked.
        for actor in c.world.get_actors():
            if actor.id not in agents_lookup:
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

        # For tracked agents.
        for agent in agents:
            actor = agent.actor

            # Declare variables.
            is_valid = True
            pref_vel = None
            path_forward = None
            bounding_box_corners = None
            lane_constraints = None

            # Update path, check validity, process variables.
            if agent.type_tag == 'Car' or agent.type_tag == 'Bicycle':
                position = get_position(actor)
                # Lane change if possible.
                if c.rng.uniform(0.0, 1.0) <= c.args.lane_change_probability:
                    new_path_candidates = c.sumo_network.get_next_route_paths(
                            c.sumo_network.get_nearest_route_point(position),
                            agent.path.min_points - 1, agent.path.interval)
                    if len(new_path_candidates) > 0:
                        new_path = SumoNetworkAgentPath(c.rng.choice(new_path_candidates)[0:agent.path.min_points], 
                                agent.path.min_points, agent.path.interval)
                        agent.path = new_path
                # Cut, resize, check.
                if not agent.path.resize(c.sumo_network):
                    is_valid = False
                else:
                    agent.path.cut(c.sumo_network, position)
                    if not agent.path.resize(c.sumo_network):
                        is_valid = False
                # Calculate variables.
                if is_valid:
                    target_position = agent.path.get_position(c.sumo_network, 5)  ## to check
                    velocity = (target_position - position).make_unit_vector()
                    pref_vel = agent.preferred_speed * velocity
                    path_forward = (agent.path.get_position(c.sumo_network, 1) - 
                            agent.path.get_position(c.sumo_network, 0)).make_unit_vector()
                    bounding_box_corners = get_vehicle_bounding_box_corners(actor)
                    lane_constraints = get_lane_constraints(c.sidewalk, position, path_forward)
            elif agent.type_tag == 'People':
                position = get_position(actor)
                # Cut, resize, check.
                if not agent.path.resize(c.sidewalk, c.args.cross_probability):
                    is_valid = False
                else:
                    agent.path.cut(c.sidewalk, position)
                    if not agent.path.resize(c.sidewalk, c.args.cross_probability):
                        is_valid = False
                # Calculate pref_vel.
                if is_valid:
                    target_position = agent.path.get_position(c.sidewalk, 0)
                    velocity = (target_position - position).make_unit_vector()
                    pref_vel = agent.preferred_speed * velocity
                    path_forward = carla.Vector2D(0, 0) # Irrelevant for pedestrian.
                    bounding_box_corners = get_pedestrian_bounding_box_corners(actor)
            
            # Add info to GAMMA.
            if pref_vel:
                gamma.add_agent(carla.AgentParams.get_default(agent.type_tag), gamma_id)
                gamma.set_agent_position(gamma_id, get_position(actor))
                gamma.set_agent_velocity(gamma_id, get_velocity(actor))
                gamma.set_agent_heading(gamma_id, get_forward_direction(actor))
                gamma.set_agent_bounding_box_corners(gamma_id, bounding_box_corners)
                gamma.set_agent_pref_velocity(gamma_id, pref_vel)
                gamma.set_agent_path_forward(gamma_id, path_forward)
                if lane_constraints is not None:
                    # Flip LR -> RL since GAMMA uses right-handed instead.
                    gamma.set_agent_lane_constraints(gamma_id, lane_constraints[1], lane_constraints[0])  
                if agent.behavior_type is not -1:
                    gamma.set_agent_behavior_type(gamma_id, agent.behavior_type)
                next_agents.append(agent)
                next_agent_gamma_ids.append(gamma_id)
                gamma_id += 1
            else:
                new_destroy_list.append(agent.actor.id)

            if agent.behavior_type is -1:
                agent.control_velocity = get_ttc_vel(agent, agents, pref_vel)

        start = time.time()        
        gamma.do_step()

        for (agent, gamma_id) in zip(next_agents, next_agent_gamma_ids):
            if agent.behavior_type is not -1 or agent.control_velocity is None:
                agent.control_velocity = gamma.get_agent_velocity(gamma_id)

    next_car_agents = [a for a in next_agents if a.type_tag == 'Car']
    next_bike_agents = [a for a in next_agents if a.type_tag == 'Bicycle']
    next_pedestrian_agents = [a for a in next_agents if a.type_tag == 'People']
    next_destroy_list = destroy_list + new_destroy_list

    c.crowd_service.acquire_control_velocities()
    c.crowd_service.control_velocities = [
            (agent.actor.id, agent.type_tag, agent.control_velocity, agent.preferred_speed, agent.steer_angle_range)
            for agent in next_agents]
    c.crowd_service.release_control_velocities()
   
    local_intentions = []
    for agent in next_agents:
        if agent.type_tag == 'People':
            local_intentions.append((agent.actor.id, agent.type_tag, agent.path.route_points[0], agent.path.route_orientations[0]))
        else:
            local_intentions.append((agent.actor.id, agent.type_tag, agent.path.route_points[0]))

    c.crowd_service.acquire_local_intentions()
    c.crowd_service.local_intentions = local_intentions
    c.crowd_service.release_local_intentions()

    return (next_car_agents, next_bike_agents, next_pedestrian_agents, next_destroy_list)


def get_ttc_vel(agent, agents, pref_vel):
    try:
        if agent:
            vel_to_exe = pref_vel
            if not vel_to_exe: # path is not ready.
                return None

            speed_to_exe = agent.preferred_speed
            for other_agent in agents:
                if other_agent and agent.actor.id != other_agent.actor.id:
                    s_f = get_velocity(other_agent.actor).length()
                    d_f = (get_position(other_agent.actor) - get_position(agent.actor)).length()
                    d_safe = 5.0
                    a_max = 3.0
                    s = max(0, s_f * s_f + 2 * a_max * (d_f - d_safe))**0.5
                    speed_to_exe = min(speed_to_exe, s)

            cur_vel = get_velocity(agent.actor)
            angle_diff = get_signed_angle_diff(vel_to_exe, cur_vel)
            if angle_diff > 30 or angle_diff < -30:
                vel_to_exe = 0.5 * (vel_to_exe + cur_vel)

            vel_to_exe = vel_to_exe.make_unit_vector() * speed_to_exe

            return vel_to_exe
    except Exception as e:
        print(e)

    return None



def do_control(c, pid_integrals, pid_last_errors, pid_last_update_time):
    start = time.time()

    c.crowd_service.acquire_control_velocities()
    control_velocities = c.crowd_service.control_velocities
    c.crowd_service.release_control_velocities()

    commands = []
    cur_time = time.time()
    if pid_last_update_time is None:
        dt = 0.0
    else:
        dt = cur_time - pid_last_update_time

    for (actor_id, type_tag, control_velocity, preferred_speed, steer_angle_range) in control_velocities:
        actor = c.world.get_actor(actor_id)
        if actor is None:
            if actor_id in pid_integrals:
                del pid_integrals[actor_id]
            if actor_id in pid_last_errors:
                del pid_last_errors[actor_id]
            continue

        cur_vel = get_velocity(actor)

        angle_diff = get_signed_angle_diff(control_velocity, cur_vel)
        if angle_diff > 30 or angle_diff < -30:
            target_speed = 0.5 * (control_velocity + cur_vel)

        if type_tag == 'Car' or type_tag == 'Bicycle':
            speed = get_velocity(actor).length()
            target_speed = control_velocity.length()
            control = actor.get_control()

            # Calculate error.
            speed_error = target_speed - speed

            # Add to integral.
            pid_integrals[actor_id] += speed_error * dt

            kp = CAR_SPEED_KP if type_tag == 'Car' else BIKE_SPEED_KP
            ki = CAR_SPEED_KI if type_tag == 'Car' else BIKE_SPEED_KI
            kd = CAR_SPEED_KD if type_tag == 'Car' else BIKE_SPEED_KD
            steer_kp = CAR_STEER_KP if type_tag == 'Car' else BIKE_STEER_KP

            # Calculate output.
            speed_control = kp * speed_error + ki * pid_integrals[actor_id]
            if pid_last_update_time is not None and actor_id in pid_last_errors:
                speed_control += kd * (speed_error - pid_last_errors[actor_id]) / dt
            
            # Update history.
            pid_last_errors[actor_id] = speed_error

            # Set control.
            if speed_control >= 0:
                control.throttle = speed_control
                control.brake = 0.0
                control.hand_brake = False
            else:
                control.throttle = 0.0
                control.brake = -speed_control
                control.hand_brake = False
            control.steer = np.clip(
                    np.clip(
                        steer_kp * get_signed_angle_diff(control_velocity, get_forward_direction(actor)), 
                        -45.0, 45.0) / steer_angle_range,
                    -1.0, 1.0)
            control.manual_gear_shift = True # DO NOT REMOVE: Reduces transmission lag.
            control.gear = 1 # DO NOT REMOVE: Reduces transmission lag.
            
            # Append to commands.
            commands.append(carla.command.ApplyVehicleControl(actor_id, control))

        elif type_tag == 'People':
            velocity = np.clip(control_velocity.length(), 0.0, preferred_speed) * control_velocity.make_unit_vector()
            control = carla.WalkerControl(carla.Vector3D(velocity.x, velocity.y), 1.0, False)
            commands.append(carla.command.ApplyWalkerControl(actor_id, control))

    c.client.apply_batch(commands)

    return cur_time # New pid_last_update_time.


def spawn_destroy_loop(args):
    try:
        # Wait for crowd service.
        time.sleep(3)
        c = Context(args)

        # Upload bounds.
        c.crowd_service.simulation_bounds = (c.bounds_min, c.bounds_max)

        last_bounds_update = None
        
        print('Spawn-destroy loop running.')
        
        while True:
            start = time.time()

            # Download bounds
            if last_bounds_update is None or start - last_bounds_update > 1.0:
                new_bounds = c.crowd_service.simulation_bounds
                if (new_bounds[0] is not None and new_bounds[0] != c.bounds_min) or \
                        (new_bounds[1] is not None and new_bounds[1] != c.bounds_max):
                    c.bounds_min = new_bounds[0]
                    c.bounds_max = new_bounds[1]
                    c.bounds_occupancy = carla.OccupancyMap(c.bounds_min, c.bounds_max)
                    c.sumo_network_spawn_segments = c.sumo_network_segments.intersection(carla.OccupancyMap(c.bounds_min, c.bounds_max))
                    c.sumo_network_spawn_segments.seed_rand(c.rng.getrandbits(32))
                    c.sidewalk_spawn_segments = c.sidewalk_segments.intersection(carla.OccupancyMap(c.bounds_min, c.bounds_max))
                    c.sidewalk_spawn_segments.seed_rand(c.rng.getrandbits(32))
                last_bounds_update = time.time()

            do_spawn(c)
            do_destroy(c)
            time.sleep(max(0, 1 / SPAWN_DESTROY_MAX_RATE - (time.time() - start)))
            # print('({}) Spawn-destroy rate: {} Hz'.format(os.getpid(), 1 / max(time.time() - start, 0.001)))
    except Pyro4.errors.ConnectionClosedError:
        pass


def control_loop(args):
    try:
        # Wait for crowd service.
        time.sleep(3)
        c = Context(args)
        print('Control loop running.')
                
        pid_integrals = defaultdict(float)
        pid_last_errors = defaultdict(float)
        pid_last_update_time = None 

        while True:
            start = time.time()
            pid_last_update_time = do_control(c, pid_integrals, pid_last_errors, pid_last_update_time)
            time.sleep(max(0, 1 / CONTROL_MAX_RATE - (time.time() - start))) # 20 Hz
            # print('({}) Control rate: {} Hz'.format(os.getpid(), 1 / max(time.time() - start, 0.001)))
    except Pyro4.errors.ConnectionClosedError:
        pass


def gamma_loop(args):
    try:
        # Wait for crowd service.
        time.sleep(3)
        c = Context(args)
        print('GAMMA loop running.')
        
        car_agents = []
        bike_agents = []
        pedestrian_agents = []
        statistics_file = open('statistics.log', 'w')
        statistics = Statistics(statistics_file)
        statistics.start_time = time.time()

        last_bounds_update = None

        rate_statistics_start = None
        rate_statistics_count = 0
        rate_statistics_done = False

        while True:
            destroy_list = []
            start = time.time()
            
            # Download bounds.
            # Required for death process.
            if last_bounds_update is None or start - last_bounds_update > 1.0:

                new_bounds = c.crowd_service.simulation_bounds
                if (new_bounds[0] is not None and new_bounds[0] != c.bounds_min) or \
                        (new_bounds[1] is not None and new_bounds[1] != c.bounds_max):
                    c.bounds_min = new_bounds[0]
                    c.bounds_max = new_bounds[1]
                    c.bounds_occupancy = carla.OccupancyMap(c.bounds_min, c.bounds_max)
                
                new_forbidden_bounds = c.crowd_service.forbidden_bounds
                if (new_forbidden_bounds[0] is not None and new_forbidden_bounds[0] != c.forbidden_bounds_min) or \
                        (new_forbidden_bounds[1] is not None and new_forbidden_bounds[1] != c.forbidden_bounds_max):
                    c.forbidden_bounds_min = new_forbidden_bounds[0]
                    c.forbidden_bounds_max = new_forbidden_bounds[1]
                    c.forbidden_bounds_occupancy = carla.OccupancyMap(c.forbidden_bounds_min, c.forbidden_bounds_max)

                last_bounds_update = time.time()

            # TODO: Maybe an functional-immutable interface wasn't the best idea...

            # Do this first if not new agents from pull_new_agents will affect avg. speed.
            (statistics) = \
                    do_speed_statistics(c, car_agents, bike_agents, pedestrian_agents, statistics)

            (car_agents, bike_agents, pedestrian_agents, statistics) = \
                    pull_new_agents(c, car_agents, bike_agents, pedestrian_agents, statistics)

            (car_agents, bike_agents, pedestrian_agents, destroy_list) = \
                    do_gamma(c, car_agents, bike_agents, pedestrian_agents, destroy_list)

            (car_agents, bike_agents, pedestrian_agents, destroy_list, statistics) = \
                    do_death(c, car_agents, bike_agents, pedestrian_agents, destroy_list, statistics)

            #statistics.write()

            c.crowd_service.acquire_destroy_list()
            c.crowd_service.extend_destroy_list(destroy_list)
            c.crowd_service.release_destroy_list()
            time.sleep(max(0, 1 / GAMMA_MAX_RATE - (time.time() - start))) # 40 Hz
            # print('({}) GAMMA rate: {} Hz'.format(os.getpid(), 1 / max(time.time() - start, 0.001)))

            '''
            if not rate_statistics_done:
                print(len(car_agents), len(bike_agents), len(pedestrian_agents), time.time() - statistics.start_time)
                if rate_statistics_start is None:
                    if time.time() - statistics.start_time > 180:
                        rate_statistics_start = time.time()
                else:
                    rate_statistics_count += 1
                    if time.time() - rate_statistics_start > 300:
                        print('Rate statistics = {:2f} Hz'.format(float(rate_statistics_count) / (time.time() - rate_statistics_start)))
                        rate_statistics_done = True
            '''
    except Pyro4.errors.ConnectionClosedError:
        pass


def collision_statistics_loop(args):
    try:
        # Wait for crowd service.
        time.sleep(3)
        c = Context(args)
        print('Collision statistics loop running.')
        
        statistics_file = open('statistics_collision.log', 'w')
        sim_start_time = time.time()

        while True:
            start = time.time()
            do_collision_statistics(c, time.time() - sim_start_time, statistics_file)
            time.sleep(max(0, 1 / COLLISION_STATISTICS_MAX_RATE - (time.time() - start))) # 40 Hz
            # print('({}) Collision statistics rate: {} Hz'.format(os.getpid(), 1 / max(time.time() - start, 0.001)))
    except Pyro4.errors.ConnectionClosedError:
        pass


def main(args):
    spawn_destroy_process = Process(target=spawn_destroy_loop, args=(args,))
    spawn_destroy_process.daemon = True
    spawn_destroy_process.start()

    control_process = Process(target=control_loop, args=(args,))
    control_process.daemon = True
    control_process.start()
    
    gamma_process = Process(target=gamma_loop, args=(args,))
    gamma_process.daemon = True
    gamma_process.start()

    '''
    collision_statistics_process = Process(target=collision_statistics_loop, args=(args,))
    collision_statistics_process.daemon = True
    collision_statistics_process.start()
    '''
    
    Pyro4.Daemon.serveSimple(
            {
                CrowdService: "crowdservice.warehouse"
            },
            port=args.pyroport,
            ns=False)

if __name__ == '__main__':
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
    argparser.add_argument(
        '-pyp', '--pyroport',
        default=8100,
        type=int,
        help='TCP port for pyro4 to listen to (default: 8100)')
    argparser.add_argument(
        '-d', '--dataset',
        default='meskel_square',
        help='Name of dataset (default: meskel_square)')
    argparser.add_argument(
        '-s', '--seed',
        default='-1',
        help='Value of random seed (default: -1)',
        type=int)
    argparser.add_argument(
        '--collision',
        help='Enables collision for controlled agents',
        action='store_true')
    argparser.add_argument(
        '--num-car',
        default='20',
        help='Number of cars to spawn (default: 20)',
        type=int)
    argparser.add_argument(
        '--num-bike',
        default='20',
        help='Number of bikes to spawn (default: 20)',
        type=int)
    argparser.add_argument(
        '--num-pedestrian',
        default='20',
        help='Number of pedestrians to spawn (default: 20)',
        type=int)
    argparser.add_argument(
        '--clearance-car',
        default='7.0',
        help='Minimum clearance (m) when spawning a car (default: 7.0)',
        type=float)
    argparser.add_argument(
        '--clearance-bike',
        default='7.0',
        help='Minimum clearance (m) when spawning a bike (default: 7.0)',
        type=float)
    argparser.add_argument(
        '--clearance-pedestrian',
        default='1.0',
        help='Minimum clearance (m) when spawning a pedestrian (default: 1.0)',
        type=float)
    argparser.add_argument(
        '--lane-change-probability',
        default='0.0',
        help='Probability of lane change for cars and bikes (default: 0.0)',
        type=float)
    argparser.add_argument(
        '--cross-probability',
        default='0.1',
        help='Probability of crossing road for pedestrians (default: 0.1)',
        type=float)
    argparser.add_argument(
        '--stuck-speed',
        default='0.2',
        help='Maximum speed (m/s) for an agent to be considered stuck (default: 0.2)',
        type=float)
    argparser.add_argument(
        '--stuck-duration',
        default='5.0',
        help='Minimum duration (s) for an agent to be considered stuck (default: 5)',
        type=float)
    args = argparser.parse_args()
    main(args)
