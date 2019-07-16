# Example showing usage of RouteMap API to create pedestrians
# that follow routes in the RouteMap.
#
# RouteMap is calculated in LibCarla (C++) from a LaneNetwork and
# exposed through the PythonAPI wrapper. Here, pedestrians are
# spawned and controlled in a loop to follow the RouteMap.
#
# In future iterations, a CrowdController API will be implemented
# in LibCarla (C++) and exposed through PythonAPI. Generally, the
# stuff in this example will be moved to LibCarla and done directly
# in LibCarla, together with the ORCA/GAMMA routines. For usage,
# the interface would be something like crowd_controller.start(num)
# or something like that.

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

class CrowdWalker:
    def __init__(self, route_map, actor, max_speed):
        self.route_map = route_map
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
        
        cut_index = 0
        for i in range(len(self.path_route_points) / 2):
            route_point = self.path_route_points[i]
            offset = position - route_map.get_position(route_point)
            offset = (offset.x**2 + offset.y**2)**0.5
            if offset < 1.0:
                cut_index = i + 1

        self.path_route_points = self.path_route_points[cut_index:]
        target_position = self.route_map.get_position(self.path_route_points[0])
    
        velocity = (target_position - position)
        velocity /= (velocity.x**2 + velocity.y**2)**0.5

        return self.max_speed * velocity

    def set_velocity(self, velocity):
        control = carla.WalkerControl(
                carla.Vector3D(velocity.x, velocity.y),
                1.0, False)
        self.actor.apply_control(control)

    def add_closest_route_point_to_path(self):
        self.path_route_points.append(self.route_map.get_nearest_route_point(self.get_position()))
    
    def extend_path(self):
        next_route_points = self.route_map.get_next_route_points(self.path_route_points[-1], 1.0)

        if len(next_route_points) == 0:
            return False

        self.path_route_points.append(random.choice(next_route_points))
        return True

def in_bounds(position):
    return -500 <= position.x <= 500 and -500 <= position.y <= 500


if __name__ == '__main__':
    lane_network = carla.LaneNetwork.load('../../Data/network.ln')
    
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(2.0)
    world = client.get_world();
    walker_blueprints = world.get_blueprint_library().filter("walker.pedestrian.*")
    crowd_walkers = []

    print(type(lane_network))
    route_map = carla.RouteMap(lane_network)
    while True:
        while len(crowd_walkers) < 100:
            position = carla.Vector2D(random.uniform(-500, 500), random.uniform(-500, 500))
            route_point = route_map.get_nearest_route_point(position)
            position = route_map.get_position(route_point)
            if in_bounds(position):
                rot = carla.Rotation()
                loc = carla.Location(position.x, position.y, 2.0)
                trans = carla.Transform(loc, rot)
                actor = world.try_spawn_actor(
                    random.choice(walker_blueprints),
                    trans)
                if actor:
                    crowd_walkers.append(CrowdWalker(route_map, actor, 2.0))

        next_crowd_walkers = []
        for crowd_walker in crowd_walkers:
            if not in_bounds(crowd_walker.get_position()):
                crowd_walker.actor.destroy()
                continue

            pref_vel = crowd_walker.get_preferred_velocity()
            if pref_vel:
                crowd_walker.set_velocity(pref_vel)
                next_crowd_walkers.append(crowd_walker)
            else:
                crowd_walker.set_velocity(carla.Vector2D(0.0, 0.0))
                crowd_walker.actor.destroy()
        crowd_walkers = next_crowd_walkers

