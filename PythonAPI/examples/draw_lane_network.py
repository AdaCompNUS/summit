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

import cv2

if __name__ == '__main__':
    lane_network = carla.LaneNetwork.load('/home/leeyiyuan/Projects/osm-convert/network.ln')
    occupancy_map = lane_network.create_occupancy_map()
    polygon_table = occupancy_map.create_polygon_table(
            carla.Vector2D(-500, -500),
            carla.Vector2D(500, 500),
            100,
            0.1)

    occupancy_grid = occupancy_map.create_occupancy_grid(
            carla.Vector2D(-500, -500),
            carla.Vector2D(500, 500),
            0.1)
    
    img = cv2.cvtColor(occupancy_grid.data, cv2.COLOR_GRAY2BGR)
    
    for r in range(polygon_table.rows):
        for c in range(polygon_table.columns):
            for p in polygon_table.get(r, c):
                print(r, c, len(polygon_table.get(r, c)))
                for i in range(len(p) - 1):
                    v1 = p[i]
                    v2 = p[i + 1]
                    cv2.arrowedLine(img, 
                        (int((v1.y - (-500)) / 0.1), int((500 - v1.x) / 0.1)),
                        (int((v2.y - (-500)) / 0.1), int((500 - v2.x) / 0.1)),
                        (0, 0, 255),
                        3)

    cv2.imwrite('test.bmp', img)
