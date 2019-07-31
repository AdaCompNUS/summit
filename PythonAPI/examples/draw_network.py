import glob
import math
import os
import sys

sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import carla

import cv2
import numpy as np
import random
import svgwrite

def rotate(v, radians):
    c, s = np.cos(radians), np.sin(radians)
    return np.array([v[0] * c - v[1] * s, v[0] * s + v[1] * c]) 

def unit_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v - v
    else:
        return v / norm

if __name__ == '__main__':

    with open('../../Data/map.net.xml', 'r') as file:
        data = file.read()

    print('Loading map...')
    network = carla.SumoNetwork.load(data)
    occupancy_map = network.create_occupancy_map()

    print('Drawing topology...')
    dwg = svgwrite.Drawing('map.svg', profile='full')
    dwg.add(dwg.rect(size=('100%', '100%'), fill='white'))
    def add_arrowed_line(start, end, **args):
        direction = unit_vector(end - start)
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

    lanes_with_connections = set()
    lanes_with_connections.update(network.edges[c.from_edge].lanes[c.from_lane].id for c in network.connections)
    
    for entry in network.edges:
        edge = entry.data()
        for lane in edge.lanes:
            for i in range(len(lane.shape) - 1):
                stroke = 'black'
                stroke_width = 0.25
                if edge.function != carla.Function.Normal:
                    stroke = 'blue'
                if i == len(lane.shape) - 2 and lane.id not in lanes_with_connections:
                    stroke = 'blue'
                    stroke_width = 0.25

                add_arrowed_line(
                    np.array([
                        lane.shape[i].y - occupancy_map.bounds_min.y, 
                        occupancy_map.bounds_max.x - lane.shape[i].x]),
                    np.array([
                        lane.shape[i + 1].y - occupancy_map.bounds_min.y, 
                        occupancy_map.bounds_max.x - lane.shape[i + 1].x]),
                        stroke=stroke,
                        stroke_width=stroke_width)

    #rand_position = carla.Vector2D(
    #    random.uniform(occupancy_map.bounds_min.x, occupancy_map.bounds_max.x),
    #    random.uniform(occupancy_map.bounds_min.y, occupancy_map.bounds_max.y))
    #rand_route_point = network.get_nearest_route_point(rand_position)
    #rand_position = network.get_route_point_position(rand_route_point)
    #
    #for path in network.get_next_route_paths(rand_route_point, 500.0, 5.0):
    #    path = [network.get_route_point_position(rp) for rp in path]
    #    for i in range(len(path) - 1):
    #        add_arrowed_line(
    #            np.array([
    #                path[i].x - occupancy_map.bounds_min.x, 
    #                occupancy_map.bounds_max.y - path[i].y]),
    #            np.array([
    #                path[i + 1].x - occupancy_map.bounds_min.x, 
    #                occupancy_map.bounds_max.y - path[i + 1].y]),
    #            stroke='magenta',
    #            stroke_width=1.0)

    dwg.save()
                
    print('Drawing occupancy grid...')
    occupancy_grid = occupancy_map.create_occupancy_grid(
        occupancy_map.bounds_min,
        occupancy_map.bounds_max,
        1.0)
    cv2.imwrite('map.bmp', occupancy_grid.data)
   
