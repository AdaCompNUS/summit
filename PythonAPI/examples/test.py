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

if __name__ == '__main__':
    lane_network = carla.LaneNetwork.load('../../Data/network.ln')
    occupancy_map = lane_network.create_occupancy_map()
    sidewalk = carla.Sidewalk(occupancy_map, 
            carla.Vector2D(-200, -200), carla.Vector2D(200, 200),
            2.0, 0.1)
    sidewalk_occupancy_map = sidewalk.create_occupancy_map()
    
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(2.0)
    client.get_world().spawn_occupancy_map(occupancy_map)
    client.get_world().spawn_occupancy_map(sidewalk_occupancy_map)
