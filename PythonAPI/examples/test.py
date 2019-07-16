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
    
    #client = carla.Client('127.0.0.1', 2000)
    #client.set_timeout(2.0)
    #client.get_world().spawn_occupancy_map(occupancy_map)

    occupancy_grid = occupancy_map.create_occupancy_grid(
            carla.Vector2D(-500, -500),
            carla.Vector2D(500, 500),
            0.1)
    
    img, contours, hierarchy = cv2.findContours(
            cv2.bitwise_not(occupancy_grid.data), 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE)
    
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(img, contours, -1, (255, 0, 0), 3)
    cv2.imwrite('test.bmp', img)
