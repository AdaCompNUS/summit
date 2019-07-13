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
    grid = carla.OccupancyGrid(4, 5)
    data = grid.data

    print(grid)
    print(data)

    grid.set(2, 3, 100)
    print(data)

    data[1][2] = 200
    print(grid.get(1, 2))
