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
from adacomp.lane_network import *
from adacomp.util import *

def from_segment(start, end, width):
    direction = end - start
    direction /= np.linalg.norm(direction)
    normal = rotate(direction, math.pi / 2) 

    v1 = start + normal * width / 2.0
    v2 = start - normal * width / 2.0
    v3 = end + normal * width / 2.0
    v4 = end - normal * width / 2.0
    yield (v1, v2, v3)
    yield (v4, v3, v2)

    for i in range(16):
        v1 = start
        v2 = start + rotate(normal, math.pi / 16.0 * i) * width / 2.0
        v3 = start + rotate(normal, math.pi / 16.0 * (i + 1)) * width / 2.0
        yield (v1, v2, v3)

        v1 = end
        v2 = end + rotate(normal, -math.pi / 16.0 * (i + 1)) * width / 2.0
        v3 = end + rotate(normal, -math.pi / 16.0 * i) * width / 2.0
        yield (v1, v2, v3)

if __name__ == '__main__':
    #client = carla.Client('127.0.0.1', 2000)
    #client.set_timeout(2.0)
    #world = client.get_world();
    #world.spawn_mesh([])

    lane_network = LaneNetwork.load('/home/leeyiyuan/Projects/osm-convert/network.ln')

    triangles = []

    for lane in lane_network.lanes.values():
        triangles.extend(from_segment(
            lane_network.get_lane_start(lane, 0),
            lane_network.get_lane_end(lane, 0),
            lane_network.lane_width))
    
    triangles = [carla.Triangle(*[carla.Vector2D(v[0], v[1]) for v in t]) for t in triangles]

    print('Creating index...')
    triangle_index = carla.TriangleIndex(triangles)
    print(len(triangle_index))

    print('In bounds = ')
    results = triangle_index.query_intersect(carla.Vector2D(-100, -100), carla.Vector2D(100, 100))
    print(len(results))
