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

import svgwrite

if __name__ == '__main__':

    with open('../../Data/map2.xodr', 'r') as file:
        data = file.read()

    topology = carla.Map('map', data).get_topology()
    waypoints = []
    for pair in topology:
        waypoints.extend(pair)

    x_avg = sum(wp.transform.location.x for wp in waypoints) / len(waypoints)
    y_avg = sum(wp.transform.location.y for wp in waypoints) / len(waypoints)
    x_min = x_avg - 400
    x_max = x_avg + 400
    y_min = y_avg - 400
    y_max = y_avg + 400
    
    def tsp(pos):
        return (pos[0] - x_min, pos[1] - y_min)

    dwg = svgwrite.Drawing(
            'map.svg', 
            width=int(y_max - y_min),
            height=int(x_max - x_min),
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

    waypoints = carla.Map('map', data).generate_waypoints(1.0)
    
    for wp in waypoints:
        a = (wp.transform.location.x, wp.transform.location.y)
        if not (x_min <= a[0] <= x_max and y_min <= a[1] <= y_max): continue

        #add_circle(a, 0.2, stroke='red', stroke_width=0.2)
        
        for next_wp in wp.next(1.0):
            b = (next_wp.transform.location.x, next_wp.transform.location.y)
            if not (x_min <= b[0] <= x_max and y_min <= b[1] <= y_max): continue
            add_line(a, b, stroke='red', stroke_width=0.2)

    dwg.save()
