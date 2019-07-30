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
            mid - 0.4 * direction + 0.4 * normal,
            mid + 0.4 * direction,
            **args))
        dwg.add(dwg.line(
            mid - 0.4 * direction - 0.4 * normal,
            mid + 0.4 * direction,
            **args))
    def tsp(pos):
        return np.array([pos.x, pos.y])

    m = carla.SumoNetwork.load(data)
    for entry in m.edges:
        edge = entry.data()
        for lane in edge.lanes:
            for i in range(len(lane.shape) - 1):
                add_arrowed_line(
                    lane.shape[i], 
                    lane.shape[i + 1],
                    stroke='red' if edge.function == carla.Function.Normal else 'blue',
                    stroke_width=0.5)
    
    dwg.save()
                
