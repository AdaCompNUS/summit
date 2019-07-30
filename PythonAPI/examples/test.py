import glob
import math
import os
import sys

sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import carla

import numpy as np
import svgwrite
import random

def rotate(v, radians):
    c, s = np.cos(radians), np.sin(radians)
    return np.array([v[0] * c - v[1] * s, v[0] * s + v[1] * c]) 

if __name__ == '__main__':

    with open('../../Data/map2.net.xml', 'r') as file:
        data = file.read()
    
    dwg = svgwrite.Drawing(
            'apple.svg', 
            profile='full')
    dwg.add(dwg.rect(
        insert=(0, 0), 
        size=('100%', '100%'), 
        rx=None, 
        ry=None, 
        fill='white'))

    def add_line(start, end, **args):
        dwg.add(dwg.line(
            tsp(start),
            tsp(end),
            **args))
    def add_circle(point, r, **args):
        dwg.add(dwg.circle(tsp(point), r, **args))
    def add_arrowed_line(start, end, **args):
        start = tsp(start)
        end = tsp(end)
        direction = end - start
        direction /= np.linalg.norm(direction)
        normal = rotate(direction, math.pi / 2)
        mid = (start + end) / 2 
        dwg.add(dwg.line(start, end, **args))
        dwg.add(dwg.line(
            mid - 0.5 * direction + 0.5 * normal,
            mid + 0.5 * direction,
            **args))
        dwg.add(dwg.line(
            mid - 0.5 * direction - 0.5 * normal,
            mid + 0.5 * direction,
            **args))
    def tsp(pos):
        return np.array([pos.x, pos.y])

    network = carla.SumoNetwork.load(data)
    for entry in network.edges:
        edge = entry.data()
        for lane in edge.lanes:
            for i in range(len(lane.shape) - 1):
                add_arrowed_line(
                    lane.shape[i], 
                    lane.shape[i + 1],
                    stroke='black' if edge.function == carla.Function.Normal else 'blue',
                    stroke_width=0.25)
    
    for _ in range(10000):
        position = carla.Vector2D(random.uniform(-5000, 5000), random.uniform(-5000, 5000))
        route_point = network.get_nearest_route_point(position)
        position = network.get_route_point_position(route_point)

        next_route_points = network.get_next_route_points(route_point, 1.0)

        for next_route_point in next_route_points:
            next_position = network.get_route_point_position(next_route_point)
            print(str(position), str(next_position))
            add_line(position, next_position,
                stroke='red',
                stroke_width=1.0)
    
    dwg.save()
                
