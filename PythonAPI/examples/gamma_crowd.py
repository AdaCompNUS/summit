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
Pyro4.config.SERIALIZERS_ACCEPTED.add('pickle')
Pyro4.config.SERIALIZER = 'pickle'
            
bounds_center = carla.Vector2D(450, 400)
bounds_min = carla.Vector2D(350, 300)
bounds_max = carla.Vector2D(550, 500)
PATH_MIN_POINTS = 20
PATH_INTERVAL = 1.0

@Pyro4.expose
@Pyro4.behavior(instance_mode="single")
class CrowdService():
    def __init__(self):
        self._car_agents = []
        self._bike_agents = []
        self._pedestrian_agents = []

    @property
    def bike_agents(self):
        return self._bike_agents
    
    @property
    def car_agents(self):
        return self._bike_agents
    
    @property
    def pedestrian_agents(self):
        return self._pedestrian_agents

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

def get_aabb(actor):
    bbox = actor.bounding_box
    loc = carla.Vector2D(bbox.location.x, bbox.location.y) + get_position(actor)
    forward_vec = get_forward_direction(actor).make_unit_vector()  # the local x direction (left-handed coordinate system)
    sideward_vec = forward_vec.rotate(np.deg2rad(90))  # the local y direction
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

class SumoNetworkAgentPath:
    def __init__(self, sumo_network, min_points, interval):
        self.sumo_network = sumo_network
        self.min_points = min_points
        self.interval = interval
        self.route_points = []

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

        path = SumoNetworkAgentPath(sumo_network, min_points, interval)
        path.route_points = rng.choice(route_paths)[0:min_points]
        return path

    def resize(self, rng=random):
        while len(self.route_points) < self.min_points:
            next_points = self.sumo_network.get_next_route_points(self.route_points[-1], self.interval)
            if len(next_points) == 0:
                return False
            self.route_points.append(rng.choice(next_points))
        return True

    def get_min_offset(self, position):
        min_offset = None
        for i in range(len(self.route_points) / 2):
            route_point = self.route_points[i]
            offset = position - self.sumo_network.get_route_point_position(route_point)
            offset = offset.length() 
            if min_offset == None or offset < min_offset:
                min_offset = offset
        return min_offset

    def cut(self, position):
        cut_index = 0
        min_offset = None
        min_offset_index = None
        for i in range(len(self.route_points) / 2):
            route_point = self.route_points[i]
            offset = position - self.sumo_network.get_route_point_position(route_point)
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

    def get_position(self, index=0):
        return self.sumo_network.get_route_point_position(self.route_points[index])

    def get_yaw(self, index=0):
        pos = self.sumo_network.get_route_point_position(self.route_points[index])
        next_pos = self.sumo_network.get_route_point_position(self.route_points[index + 1])
        return np.rad2deg(math.atan2(next_pos.y - pos.y, next_pos.x - pos.x))

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
    
    walker_blueprints = world.get_blueprint_library().filter('walker.pedestrian.*')
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
                    carla.Vector2D(path.get_position(0).x - args.clearance_car,
                                   path.get_position(0).y - args.clearance_car),
                    carla.Vector2D(path.get_position(0).x + args.clearance_car,
                                   path.get_position(0).y + args.clearance_car))):
                trans = carla.Transform()
                trans.location.x = path.get_position(0).x
                trans.location.y = path.get_position(0).y
                trans.location.z = 0.2
                trans.rotation.yaw = path.get_yaw(0)

                actor = world.try_spawn_actor(rng.choice(car_blueprints), trans)
                if actor:
                    # actor.set_collision_enabled(False)
                    world.wait_for_tick(1.0)  # For actor to update pos and bounds, and for collision to apply.
                    aabb_map.insert(get_aabb(actor))
                    '''
                    self.network_car_agents_lock.acquire()
                    self.network_car_agents.append(CrowdNetworkCarAgent(
                        self, actor, path,
                        5.0 + self.rng.uniform(0.0, 0.5)))
                    self.network_car_agents_lock.release()
                    '''
        
        '''
        # Spawn at most one bike.
        self.network_bike_agents_lock.acquire()
        do_spawn = len(self.network_bike_agents) < self.num_network_bike_agents
        self.network_bike_agents_lock.release()
        if do_spawn:
            path = NetworkAgentPath.rand_path(self, self.path_min_points, self.path_interval, spawn_segment_map, rng=self.rng)
            if aabb_map.intersects(carla.AABB2D(
                    carla.Vector2D(path.get_position(0).x - args.clearance_car,
                                   path.get_position(0).y - args.clearance_car),
                    carla.Vector2D(path.get_position(0).x + args.clearance_car,
                                   path.get_position(0).y + args.clearance_car))) and \
                    not exo_aabb_map.intersects(carla.AABB2D(
                        carla.Vector2D(path.get_position(0).x - self.spawn_clearance_ego,
                                       path.get_position(0).y - self.spawn_clearance_ego),
                        carla.Vector2D(path.get_position(0).x + self.spawn_clearance_ego,
                                       path.get_position(0).y + self.spawn_clearance_ego))):
                trans = carla.Transform()
                trans.location.x = path.get_position(0).x
                trans.location.y = path.get_position(0).y
                trans.location.z = 0.2
                trans.rotation.yaw = path.get_yaw(0)
                actor = self.world.try_spawn_actor(self.rng.choice(self.bikes_blueprints), trans)
                if actor:
                    # actor.set_collision_enabled(False)
                    self.world.wait_for_tick(1.0)  # For actor to update pos and bounds, and for collision to apply.
                    aabb_map.insert(self.get_aabb(actor))
                    self.network_bike_agents_lock.acquire()
                    self.network_bike_agents.append(CrowdNetworkBikeAgent(
                        self, actor, path,
                        3.0 + self.rng.uniform(0, 0.5)))
                    self.network_bike_agents_lock.release()

        # Spawn at most one pedestrian.
        self.sidewalk_agents_lock.acquire()
        do_spawn = len(self.sidewalk_agents) < self.num_sidewalk_agents
        self.sidewalk_agents_lock.release()
        if do_spawn:
            path = SidewalkAgentPath.rand_path(self, self.path_min_points, self.path_interval, bounds_min, bounds_max, self.rng)
            if aabb_map.intersects(carla.AABB2D(
                    carla.Vector2D(path.get_position(0).x - self.spawn_clearance_person,
                                   path.get_position(0).y - self.spawn_clearance_person),
                    carla.Vector2D(path.get_position(0).x + self.spawn_clearance_person,
                                   path.get_position(0).y + self.spawn_clearance_person))) and \
                    not exo_aabb_map.intersects(carla.AABB2D(
                        carla.Vector2D(path.get_position(0).x - self.spawn_clearance_ego,
                                       path.get_position(0).y - self.spawn_clearance_ego),
                        carla.Vector2D(path.get_position(0).x + self.spawn_clearance_ego,
                                       path.get_position(0).y + self.spawn_clearance_ego))):
                trans = carla.Transform()
                trans.location.x = path.get_position(0).x
                trans.location.y = path.get_position(0).y
                trans.location.z = 0.2
                trans.rotation.yaw = path.get_yaw(0)
                actor = self.world.try_spawn_actor(self.rng.choice(self.walker_blueprints), trans)
                if actor:
                    # actor.set_collision_enabled(False)
                    self.world.wait_for_tick(1.0)  # For actor to update pos and bounds, and for collision to apply.
                    aabb_map.insert(self.get_aabb(actor))
                    self.sidewalk_agents_lock.acquire()
                    self.sidewalk_agents.append(CrowdSidewalkAgent(
                        self, actor, path,
                        0.5 + self.rng.uniform(0.0, 1.0)))
                    self.sidewalk_agents_lock.release()

        if not self.initialized and rospy.Time.now() - self.start_time > rospy.Duration.from_sec(5.0):
            self.agents_ready_pub.publish(True)
            self.initialized = True
        if rospy.Time.now() - self.start_time > rospy.Duration.from_sec(10.0):
            self.do_publish = True
        '''

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
        '--num-car',
        default='10',
        help='Number of cars to spawn (default: 10)',
        type=int)
    argparser.add_argument(
        '--clearance-car',
        default='5.0',
        help='Minimum clearance in meters when spawning a car (default: 5.0)',
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
