#!/usr/bin/env python3

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

from multiprocessing import Process
from pathlib import Path
from threading import RLock
import Pyro4
import argparse
import atexit
import carla
import math
import numpy as np
import random
import time

DATA_PATH = Path(os.path.realpath(__file__)).parent.parent.parent/'Data'

# This applies to all processes.
Pyro4.config.COMMTIMEOUT = 2.0
Pyro4.config.SERIALIZERS_ACCEPTED.add('serpent')
Pyro4.config.SERIALIZER = 'serpent'

# Serialization for carla.Vector2D.
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

# Serialization for carla.SumoNetworkRoutePoint.
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
    r.edge = o['edge']
    r.lane = o['lane']
    r.segment = o['segment']
    r.offset = o['offset']
    return r
Pyro4.util.SerializerBase.register_dict_to_class(
        'carla.SumoNetworkRoutePoint', dict_to_sumo_network_route_point)

# Serialization for carla.SidewalkRoutePoint.
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
    return r
Pyro4.util.SerializerBase.register_dict_to_class(
        'carla.SidewalkRoutePoint', dict_to_sidewalk_route_point)

            
bounds_center = carla.Vector2D(450, 400)
bounds_min = carla.Vector2D(300, 250)
bounds_max = carla.Vector2D(500, 550)
PATH_MIN_POINTS = 20
PATH_INTERVAL = 1.0

''' Crowd service class definition '''
@Pyro4.expose
@Pyro4.behavior(instance_mode="single")
class CrowdService():
    def __init__(self):
        self._car_agents = []
        self._car_agents_lock = RLock()
        self._bike_agents = []
        self._bike_agents_lock = RLock()
        self._pedestrian_agents = []
        self._pedestrian_agents_lock = RLock()
    
    @property
    def car_agents(self):
        return self._car_agents

    @car_agents.setter
    def car_agents(self, agents):
        self._car_agents = agents

    def acquire_car_agents(self):
        self._car_agents_lock.acquire()

    def release_car_agents(self):
        self._car_agents_lock.release()

    @property
    def bike_agents(self):
        return self._bike_agents

    @bike_agents.setter
    def bike_agents(self, agents):
        self._bike_agents = agents

    def acquire_bike_agents(self):
        self._bike_agents_lock.acquire()

    def release_bike_agents(self):
        self._bike_agents_lock.release()

    @property
    def pedestrian_agents(self):
        return self._pedestrian_agents
    
    @pedestrian_agents.setter
    def pedestrian_agents(self, agents):
        self._pedestrian_agents = agents

    def acquire_pedestrian_agents(self):
        self._pedestrian_agents_lock.acquire()

    def release_pedestrian_agents(self):
        self._pedestrian_agents_lock.release()



''' Utility functions. '''
def get_cars(world):
    return [a for a in world.get_actors().filter('vehicle.*') if int(a.attributes['number_of_wheels']) == 4]

def get_bikes(world):
    return [a for a in world.get_actors().filter('vehicle.*') if int(a.attributes['number_of_wheels']) == 2]

def get_pedestrians(world):
    return [a for a in world.get_actors().filter('walker.*')]
    
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
    
def get_lane_constraints(sidewalk, actor):
    position = get_position(actor)
    forward_vec = get_forward_direction(actor)
    left_line_end = position + (1.5 + 2.0 + 0.8) * ((forward_vec.rotate(np.deg2rad(-90))).make_unit_vector())
    right_line_end = position + (1.5 + 2.0 + 0.5) * ((forward_vec.rotate(np.deg2rad(90))).make_unit_vector())
    left_lane_constrained_by_sidewalk = sidewalk.intersects(position, left_line_end)
    right_lane_constrained_by_sidewalk =sidewalk.intersects(position, right_line_end)
    return left_lane_constrained_by_sidewalk, right_lane_constrained_by_sidewalk



''' Represents a path on the SUMO network. '''
class SumoNetworkAgentPath:
    def __init__(self, route_points, min_points, interval):
        self.route_points = route_points
        self.min_points = min_points
        self.interval = interval

    @staticmethod
    def rand_path(sumo_network, min_points, interval, segment_map, min_safe_points=None, rng=random):
        if min_safe_points is None:
            min_safe_points = min_points

        spawn_point = None
        route_paths = None
        while not spawn_point or len(route_paths) < 1:
            spawn_point = segment_map.rand_point()
            spawn_point = sumo_network.get_nearest_route_point(spawn_point)
            route_paths = sumo_network.get_next_route_paths(spawn_point, min_safe_points - 1, interval)

        return SumoNetworkAgentPath(rng.choice(route_paths)[0:min_points], min_points, interval)

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

# Serialization for SumoNetworkAgentPath.
Pyro4.util.SerializerBase.register_class_to_dict(
        SumoNetworkAgentPath,
        lambda o: { 
            '__class__': 'SumoNetworkAgentPath',
            'route_points': [Pyro4.util.SerializerBase.class_to_dict(p) for p in o.route_points],
            'min_points': o.min_points,
            'interval': o.interval
        })
Pyro4.util.SerializerBase.register_dict_to_class(
        'SumoNetworkAgentPath',
        lambda c, o: SumoNetworkAgentPath(
            [Pyro4.util.SerializerBase.dict_to_class(p) for p in o['route_points']],
            o['min_points'],
            o['interval'])
        )



''' Represents a path on the sidewalk. '''
class SidewalkAgentPath:
    def __init__(self, route_points, route_orientations, min_points, interval):
        self.min_points = min_points
        self.interval = interval
        self.route_points = route_points
        self.route_orientations = route_orientations

    @staticmethod
    def rand_path(sidewalk, min_points, interval, cross_probability, bounds_min, bounds_max, rng=None,):
        if rng is None:
            rng = random
    
        point = None
        while point is None or not (bounds_min.x <= point_position.x <= bounds_max.x and bounds_min.y <= point_position.y <= bounds_max.y):
            point = carla.Vector2D(rng.uniform(bounds_min.x, bounds_max.x), rng.uniform(bounds_min.y, bounds_max.y))
            point = sidewalk.get_nearest_route_point(point)
            point_position = sidewalk.get_route_point_position(point)

        path = SidewalkAgentPath([point], [rng.choice([True, False])], min_points, interval)
        path.resize(sidewalk, cross_probability)
        return path

    def resize(self, sidewalk, cross_probability, rng=None):
        if rng is None:
            rng = random

        while len(self.route_points) < self.min_points:
            if rng.random() <= cross_probability:
                adjacent_route_points = sidewalk.get_adjacent_route_points(self.route_points[-1], 50.0)
                if adjacent_route_points:
                    self.route_points.append(adjacent_route_points[0])
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

# Serialization for SidewalkAgentPath.
Pyro4.util.SerializerBase.register_class_to_dict(
        SidewalkAgentPath,
        lambda o: { 
            '__class__': 'SidewalkAgentPath',
            'route_points': [Pyro4.util.SerializerBase.class_to_dict(p) for p in o.route_points],
            'route_orientations': o.route_points,
            'min_points': o.min_points,
            'interval': o.interval
        })
Pyro4.util.SerializerBase.register_dict_to_class(
        'SidewalkAgentPath',
        lambda c, o: SidewalkAgentPath(
            [Pyro4.util.SerializerBase.dict_to_class(p) for p in o['route_points']],
            o['route_orientations'],
            o['min_points'],
            o['interval'])
        )



''' Represents an agent. '''
class Agent(object):
    def __init__(self, actor_id, type_tag, path, preferred_speed, stuck_time=None, control_velocity=carla.Vector2D(0, 0)):
        self.actor_id = actor_id
        self.type_tag = type_tag
        self.path = path
        self.preferred_speed = preferred_speed
        self.stuck_time = stuck_time
        self.control_velocity = control_velocity

# Serialization for Agent.
Pyro4.util.SerializerBase.register_class_to_dict(
        Agent, 
        lambda o: { 
            '__class__': 'Agent',
            'actor_id': o.actor_id,
            'type_tag': o.type_tag,
            'path': Pyro4.util.SerializerBase.class_to_dict(o.path),
            'preferred_speed': o.preferred_speed,
            'stuck_time': o.stuck_time,
            'control_velocity': o.control_velocity
        })
Pyro4.util.SerializerBase.register_dict_to_class(
        'Agent',
        lambda c, o: Agent(
            o['actor_id'], 
            o['type_tag'],
            Pyro4.util.SerializerBase.dict_to_class(o['path']),
            o['preferred_speed'], 
            o['stuck_time'], 
            o['control_velocity']))




def spawn_loop(args):
    sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(args.dataset)))
    sumo_network_segment_map = sumo_network.create_segment_map()
    sumo_network_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.network.wkt'.format(args.dataset)))
    sidewalk = sumo_network_occupancy.create_sidewalk(1.5)
    sidewalk_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.sidewalk.wkt'.format(args.dataset)))
    rng = random.Random(args.seed)
    crowd_service = Pyro4.Proxy('PYRO:crowdservice.warehouse@localhost:8100')
    print('Spawn loop connected.')

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    
    pedestrian_blueprints = world.get_blueprint_library().filter('walker.pedestrian.*')
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')
    car_blueprints = [x for x in vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 4]
    car_blueprints = [x for x in car_blueprints if x.id not in ['vehicle.mini.cooperst']]
    bike_blueprints = [x for x in vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 2]

    while True:    
        # Get segment map within ego range.
        spawn_segment_map = sumo_network_segment_map.intersection(
                carla.OccupancyMap(bounds_min, bounds_max))
        spawn_segment_map.seed_rand(rng.getrandbits(32))

        # Get AABB.
        aabb_map = carla.AABBMap(
            [get_aabb(actor) for actor in get_cars(world) + get_bikes(world) + get_pedestrians(world)])

        # Spawn at most one car.
        if len(crowd_service.car_agents) < args.num_car:
            path = SumoNetworkAgentPath.rand_path(sumo_network, PATH_MIN_POINTS, PATH_INTERVAL, spawn_segment_map, rng=rng)
            if not aabb_map.intersects(carla.AABB2D(
                    carla.Vector2D(path.get_position(sumo_network, 0).x - args.clearance_car,
                                   path.get_position(sumo_network, 0).y - args.clearance_car),
                    carla.Vector2D(path.get_position(sumo_network, 0).x + args.clearance_car,
                                   path.get_position(sumo_network, 0).y + args.clearance_car))):
                trans = carla.Transform()
                trans.location.x = path.get_position(sumo_network, 0).x
                trans.location.y = path.get_position(sumo_network, 0).y
                trans.location.z = 0.2
                trans.rotation.yaw = path.get_yaw(sumo_network, 0)

                actor = world.try_spawn_actor(rng.choice(car_blueprints), trans)
                if actor:
                    actor.set_collision_enabled(args.collision)
                    world.wait_for_tick(1.0)  # For actor to update pos and bounds, and for collision to apply.
                    aabb_map.insert(get_aabb(actor))
                    crowd_service.acquire_car_agents()
                    crowd_service.car_agents += [Agent(
                        actor.id, 'Car', path, 5.0 + rng.uniform(0.0, 0.5))]
                    crowd_service.release_car_agents()


        # Spawn at most one bike.
        if len(crowd_service.bike_agents) < args.num_bike:
            path = SumoNetworkAgentPath.rand_path(sumo_network, PATH_MIN_POINTS, PATH_INTERVAL, spawn_segment_map, rng=rng)
            if not aabb_map.intersects(carla.AABB2D(
                    carla.Vector2D(path.get_position(sumo_network, 0).x - args.clearance_bike,
                                   path.get_position(sumo_network, 0).y - args.clearance_bike),
                    carla.Vector2D(path.get_position(sumo_network, 0).x + args.clearance_bike,
                                   path.get_position(sumo_network, 0).y + args.clearance_bike))):
                trans = carla.Transform()
                trans.location.x = path.get_position(sumo_network, 0).x
                trans.location.y = path.get_position(sumo_network, 0).y
                trans.location.z = 0.2
                trans.rotation.yaw = path.get_yaw(sumo_network, 0)

                actor = world.try_spawn_actor(rng.choice(bike_blueprints), trans)
                if actor:
                    actor.set_collision_enabled(args.collision)
                    world.wait_for_tick(1.0)  # For actor to update pos and bounds, and for collision to apply.
                    aabb_map.insert(get_aabb(actor))
                    crowd_service.acquire_bike_agents()
                    crowd_service.bike_agents += [Agent(
                        actor.id, 'Bicycle', path, 3.0 + rng.uniform(0.0, 0.5))]
                    crowd_service.release_bike_agents()


        # Spawn at most one pedestrian.
        if len(crowd_service.pedestrian_agents) < args.num_pedestrians:
            path = SidewalkAgentPath.rand_path(sidewalk, PATH_MIN_POINTS, PATH_INTERVAL, args.probability_cross, bounds_min, bounds_max, rng)
            if not aabb_map.intersects(carla.AABB2D(
                    carla.Vector2D(path.get_position(sidewalk, 0).x - args.clearance_pedestrian,
                                   path.get_position(sidewalk, 0).y - args.clearance_pedestrian),
                    carla.Vector2D(path.get_position(sidewalk, 0).x + args.clearance_pedestrian,
                                   path.get_position(sidewalk, 0).y + args.clearance_pedestrian))):
                trans = carla.Transform()
                trans.location.x = path.get_position(sidewalk, 0).x
                trans.location.y = path.get_position(sidewalk, 0).y
                trans.location.z = 0.2
                trans.rotation.yaw = path.get_yaw(sidewalk, 0)
                actor = world.try_spawn_actor(rng.choice(pedestrian_blueprints), trans)
                if actor:
                    actor.set_collision_enabled(args.collision)
                    world.wait_for_tick(1.0)  # For actor to update pos and bounds, and for collision to apply.
                    aabb_map.insert(get_aabb(actor))
                    crowd_service.acquire_pedestrian_agents()
                    crowd_service.pedestrian_agents += [Agent(
                        actor.id, 'People', path, 0.5 + rng.uniform(0.0, 1.0))]
                    crowd_service.release_pedestrian_agents()


def destroy_loop(args):
    sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(args.dataset)))
    sumo_network_segment_map = sumo_network.create_segment_map()
    sumo_network_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.network.wkt'.format(args.dataset)))
    sidewalk = sumo_network_occupancy.create_sidewalk(1.5)
    sidewalk_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.sidewalk.wkt'.format(args.dataset)))
    rng = random.Random(args.seed)
    crowd_service = Pyro4.Proxy('PYRO:crowdservice.warehouse@localhost:8100')
    print('Spawn loop connected.')

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

def control_loop(args):
    sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(args.dataset)))
    sumo_network_segment_map = sumo_network.create_segment_map()
    sumo_network_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.network.wkt'.format(args.dataset)))
    sidewalk = sumo_network_occupancy.create_sidewalk(1.5)
    sidewalk_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.sidewalk.wkt'.format(args.dataset)))
    rng = random.Random(args.seed)
    crowd_service = Pyro4.Proxy('PYRO:crowdservice.warehouse@localhost:8100')
    print('Control loop connected.')

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

def gamma_loop(args):
    sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(args.dataset)))
    sumo_network_segment_map = sumo_network.create_segment_map()
    sumo_network_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.network.wkt'.format(args.dataset)))
    sidewalk = sumo_network_occupancy.create_sidewalk(1.5)
    sidewalk_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.sidewalk.wkt'.format(args.dataset)))
    rng = random.Random(args.seed)
    crowd_service = Pyro4.Proxy('PYRO:crowdservice.warehouse@localhost:8100')
    print('GAMMA loop connected.')

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
   
    while True:
        crowd_service.acquire_car_agents()
        crowd_service.acquire_bike_agents()
        crowd_service.acquire_pedestrian_agents()

        agents = crowd_service.car_agents + crowd_service.bike_agents + crowd_service.pedestrian_agents
        if len(agents) > 0:
            gamma = carla.RVOSimulator()
            
            for (i, agent) in enumerate(agents):
                actor = world.get_actor(agent.actor_id)
                gamma.add_agent(carla.AgentParams.get_default(agent.type_tag), i)

                is_valid = True
                pref_vel = None
                path_forward = None
                bounding_box_corners = None

                # Update path, check, proces variables.
                if isinstance(agent.path, SumoNetworkAgentPath):
                    position = get_position(actor)
                    # Lane change if possible.
                    if rng.uniform(0.0, 1.0) <= args.probability_lane_change:
                        new_path_candidates = sumo_network.get_next_route_paths(
                                sumo_network.get_nearest_route_point(position),
                                agent.path.min_points - 1, agent.path.interval)
                        if len(new_path_candidates) > 0:
                            new_path = SumoNetworkAgentPath(rng.choice(new_path_candidates)[0:agent.path.min_points], 
                                    agent.path.min_points, agent.path.interval)
                            agent.path = new_path
                    # Cut, resize, check.
                    if not agent.path.resize(sumo_network):
                        is_valid = False
                    else:
                        agent.path.cut(sumo_network, position)
                        if not agent.path.resize(sumo_network):
                            is_valid = False
                    # Calculate variables.
                    if is_valid:
                        target_position = agent.path.get_position(sumo_network, 5)  ## to check
                        velocity = (target_position - position).make_unit_vector()
                        pref_vel =  agent.preferred_speed * velocity
                        path_forward = (agent.path.get_position(sumo_network, 1) - 
                                agent.path.get_position(sumo_network, 0)).make_unit_vector()
                        bounding_box_corners = get_vehicle_bounding_box_corners(actor)
                elif isinstance(agent.path, SidewalkAgentPath):
                    position = get_position(actor)
                    # Cut, resize, check.
                    if not agent.path.resize(sidewalk, args.probability_cross):
                        is_valid = False
                    else:
                        agent.path.cut(sidewalk, position)
                        if not agent.path.resize(sidewalk, args.probability_cross):
                            is_valid = False
                    # Calculate pref_vel.
                    if is_valid:
                        target_position = agent.path.get_position(sidewalk, 0)
                        velocity = (target_position - position).make_unit_vector()
                        pref_vel = agent.preferred_speed * velocity
                        path_forward = carla.Vector2D(0, 0) # Irrelevant for pedestrian.
                        bounding_box_corners = get_pedestrian_bounding_box_corners(actor)


                if pref_vel:
                    gamma.set_agent_position(i, get_position(actor))
                    gamma.set_agent_velocity(i, get_velocity(actor))
                    gamma.set_agent_heading(i, get_forward_direction(actor))
                    gamma.set_agent_bounding_box_corners(i, bounding_box_corners)
                    gamma.set_agent_pref_velocity(i, pref_vel)
                    gamma.set_agent_path_forward(i, path_forward)
                    gamma.set_agent_lane_constraints(i, *get_lane_constraints(sidewalk, actor)) 
                else:
                    agents[i] = None # Ignore for rest of GAMMA loop.
                    self.gamma.set_agent_position(i, default_agent_pos)
                    self.gamma.set_agent_pref_velocity(i, carla.Vector2D(0, 0))
                    self.gamma.set_agent_velocity(i, carla.Vector2D(0, 0))
                    self.gamma.set_agent_bounding_box_corners(i, default_agent_bbox)

            gamma.do_step()

            for (i, agent) in enumerate(agents):
                if agent:
                    agent.control_velocity = gamma.get_agent_velocity(i)
       
        crowd_service.release_car_agents()
        crowd_service.release_bike_agents()
        crowd_service.release_pedestrian_agents()


def main():
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
        default='10',
        help='Number of cars to spawn (default: 10)',
        type=int)
    argparser.add_argument(
        '--num-bike',
        default='10',
        help='Number of bikes to spawn (default: 10)',
        type=int)
    argparser.add_argument(
        '--num-pedestrians',
        default='10',
        help='Number of pedestrians to spawn (default: 10)',
        type=int)
    argparser.add_argument(
        '--clearance-car',
        default='5.0',
        help='Minimum clearance in meters when spawning a car (default: 5.0)',
        type=float)
    argparser.add_argument(
        '--clearance-bike',
        default='5.0',
        help='Minimum clearance in meters when spawning a bike (default: 5.0)',
        type=float)
    argparser.add_argument(
        '--clearance-pedestrian',
        default='0.5',
        help='Minimum clearance in meters when spawning a pedestrian (default: 0.5)',
        type=float)
    argparser.add_argument(
        '--probability-lane-change',
        default='0.1',
        help='Probability of lane change for cars and bikes (default: 0.1)',
        type=float)
    argparser.add_argument(
        '--probability-cross',
        default='0.05',
        help='Probability of crossing road for pedestrians (default: 0.05)',
        type=float)
    args = argparser.parse_args()

    Process(target=spawn_loop, args=(args,), daemon=True).start()
    Process(target=destroy_loop, args=(args,), daemon=True).start()
    Process(target=control_loop, args=(args,), daemon=True).start()
    Process(target=gamma_loop, args=(args,), daemon=True).start()
    
    Pyro4.Daemon.serveSimple(
            {
                CrowdService: "crowdservice.warehouse"
            },
            port=8100,
            ns=False)

if __name__ == '__main__':
    main()
