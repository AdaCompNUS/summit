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

if __name__ == '__main__':
    lane_network = carla.LaneNetwork.load('/home/leeyiyuan/Projects/osm-convert/network.ln')
    print(len(lane_network.nodes()))
    print(len(lane_network.roads()))
    print(len(lane_network.lanes()))
    print(len(lane_network.lane_connections()))

