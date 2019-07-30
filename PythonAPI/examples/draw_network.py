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
import svgwrite

def rotate(v, radians):
    c, s = np.cos(radians), np.sin(radians)
    return np.array([v[0] * c - v[1] * s, v[0] * s + v[1] * c]) 

if __name__ == '__main__':

    with open('../../Data/map2.net.xml', 'r') as file:
        data = file.read()

    print('Loading map...')
    network = carla.SumoNetwork.load(data)
    occupancy_map = network.create_occupancy_map()

    print('Drawing occupancy grid...')
    occupancy_grid = occupancy_map.create_occupancy_grid(
        occupancy_map.bounds_min,
        occupancy_map.bounds_max,
        0.2)
    cv2.imwrite('map2.bmp', cv2.transpose(occupancy_grid.data[::-1,::-1]))
   
    print('Drawing topology...')
    dwg = svgwrite.Drawing('map2.svg', profile='full')
    dwg.add(dwg.rect(size=('100%', '100%'), fill='white'))
    def add_arrowed_line(start, end, **args):
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

    for entry in network.edges:
        edge = entry.data()
        for lane in edge.lanes:
            for i in range(len(lane.shape) - 1):
                add_arrowed_line(
                    np.array([
                        lane.shape[i].x - occupancy_map.bounds_min.x, 
                        occupancy_map.bounds_max.y - lane.shape[i].y]),
                    np.array([
                        lane.shape[i + 1].x - occupancy_map.bounds_min.x,
                        occupancy_map.bounds_max.y - lane.shape[i + 1].y]),
                    stroke='blue' if edge.function == carla.Function.Normal else 'red',
                    stroke_width=0.25)
    dwg.save()
                
