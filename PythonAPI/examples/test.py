import glob
import math
import os
import sys

sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import numpy as np
import carla
import random
import time

class CrowdWalker:
    def __init__(self, sidewalk, actor, max_speed):
        self.sidewalk = sidewalk
        self.actor = actor
        self.max_speed = max_speed
        self.path_route_points = []

    def get_position(self):
        pos3D = self.actor.get_location()
        return carla.Vector2D(pos3D.x, pos3D.y)

    def get_preferred_velocity(self):
        position = self.get_position()

        if len(self.path_route_points) == 0:
            self.add_closest_route_point_to_path()
        while len(self.path_route_points) < 20 and self.extend_path():
            pass
        if len(self.path_route_points) < 20:
            return None
            
        last_pos = self.sidewalk.get_route_point_position(self.path_route_points[-1])
        world.debug.draw_point(
            carla.Location(last_pos.x, last_pos.y),
            0.2,
            carla.Color(0, 0, 255),
            0.4)

        adjacent_route_points = self.sidewalk.get_adjacent_route_points(self.path_route_points[-1])
        for route_point in adjacent_route_points:
            pos = self.sidewalk.get_route_point_position(route_point)
            world.debug.draw_point(
                carla.Location(pos.x, pos.y),
                0.2,
                carla.Color(255, 0, 0),
                0.4)
        
        cut_index = 0
        for i in range(len(self.path_route_points) / 2):
            route_point = self.path_route_points[i]
            offset = position - self.sidewalk.get_route_point_position(route_point)
            offset = (offset.x**2 + offset.y**2)**0.5
            if offset < 1.0:
                cut_index = i + 1

        self.path_route_points = self.path_route_points[cut_index:]
        target_position = self.sidewalk.get_route_point_position(self.path_route_points[0])
    
        velocity = (target_position - position)
        velocity /= (velocity.x**2 + velocity.y**2)**0.5

        return self.max_speed * velocity

    def set_velocity(self, velocity):
        control = carla.WalkerControl(
                carla.Vector3D(velocity.x, velocity.y),
                1.0, False)
        self.actor.apply_control(control)

    def add_closest_route_point_to_path(self):
        self.path_route_points.append(self.sidewalk.get_nearest_route_point(self.get_position()))
    
    def extend_path(self):
        self.path_route_points.append(self.sidewalk.get_previous_route_point(self.path_route_points[-1], 1.0))
        return True

def in_bounds(position):
    return -200 <= position.x <= 200 and -200 <= position.y <= 200

NUM_WALKERS = 1

if __name__ == '__main__':
    lane_network = carla.LaneNetwork.load('../../Data/network.ln')
    occupancy_map = lane_network.create_occupancy_map()
    sidewalk = carla.Sidewalk(
            occupancy_map,
            carla.Vector2D(-200, -200), carla.Vector2D(200, 200),
            3.0, 0.1,
            10.0)
    sidewalk_occupancy_map = sidewalk.create_occupancy_map()

    gamma = carla.RVOSimulator()
    for i in range(NUM_WALKERS):
        gamma.add_agent(carla.AgentParams.get_default("People"), i)
    
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world();
    world.spawn_occupancy_map(
        occupancy_map, 
        '/Game/Carla/Static/GenericMaterials/Asphalt/M_Asphalt01')
    world.spawn_occupancy_map(
        sidewalk_occupancy_map,
        '/Game/Carla/Static/GenericMaterials/M_Red')
    world.wait_for_tick()

    walker_blueprints = world.get_blueprint_library().filter("walker.pedestrian.*")
    crowd_walkers = []

    while True:

        while len(crowd_walkers) < NUM_WALKERS:
            position = carla.Vector2D(random.uniform(-200, 200), random.uniform(-200, 200))
            route_point = sidewalk.get_nearest_route_point(position)
            position = sidewalk.get_route_point_position(route_point)
            if in_bounds(position):
                rot = carla.Rotation()
                loc = carla.Location(position.x, position.y, 50.0)
                trans = carla.Transform(loc, rot)
                actor = world.try_spawn_actor(
                    random.choice(walker_blueprints),
                    trans)
                if actor:
                    crowd_walkers.append(CrowdWalker(sidewalk, actor, 2.0))
        world.wait_for_tick()

        next_crowd_walkers = []
        for (i, crowd_walker) in enumerate(crowd_walkers):
            if not in_bounds(crowd_walker.get_position()):
                next_crowd_walkers.append(None)
                crowd_walker.actor.destroy()
                continue

            pref_vel = crowd_walker.get_preferred_velocity()
            if pref_vel:
                next_crowd_walkers.append(crowd_walker)
                gamma.set_agent_position(i, crowd_walker.get_position())
                gamma.set_agent_pref_velocity(i, pref_vel)
            else:
                next_crowd_walkers.append(None)
                gamma.set_agent_position(i, crowd_walker.get_position())
                gamma.set_agent_pref_velocity(i, carla.Vector2D(0, 0))
                crowd_walker.actor.destroy()
        crowd_walkers = next_crowd_walkers
        
        gamma.do_step()

        for (i, crowd_walker) in enumerate(crowd_walkers):
            if crowd_walker is not None:
                crowd_walker.set_velocity(gamma.get_agent_velocity(i))

        crowd_walkers = [w for w in crowd_walkers if w is not None]
