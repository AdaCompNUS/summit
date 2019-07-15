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
    
    occupancy_grid = occupancy_map.create_occupancy_grid(
            carla.Vector2D(-1000, -1000),
            carla.Vector2D(1000, 1000),
            0.1)
    cv2.imwrite('test.bmp', occupancy_grid.data)
