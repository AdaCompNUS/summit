'''
Process the geometry of a given SUMO network located in Data/ to produce the
following in the same folder:
- Occupancy map of SUMO network.
- Occupancy mesh triangles of SUMO network.
- Occupancy map of SUMO network's sidewalk.
- Occupancy mesh triangles of SUMO network's sidewalk.

Usage:
  python3 process_geometry.py <dataset>
  
  Processes the SUMO network located at Data/<dataset>.net.xml to produce
  the respective occupancy map (<dataset>.wkt, <dataset>.sidewalk.wkt) and 
  occupancy mesh triangles (<dataset>.mesh, <dataset>.sidewalk.mesh) files
  in Data/.

Example:
  python3 process_geometry.py meskel_square
'''

#!/usr/bin/env python3

import glob
import math
import os
import sys

sys.path.append(glob.glob('../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import carla

from pathlib import Path

DATA_PATH = Path(os.path.realpath(__file__)).parent.parent/'Data'

if __name__ == '__main__':

    data = sys.argv[1]

    print('Loading SUMO network...')
    network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(data)))
    
    print('Testing network segments...')
    segment_map = network.create_segment_map()

    print('Calculating SUMO network occupancy map...')
    network_occupancy_map = network.create_occupancy_map()

    print('Writing SUMO network occupancy map...')
    network_occupancy_map.save(str(DATA_PATH/'{}.wkt'.format(data)))

    print('Test loading SUMO network occupancy map...')
    carla.OccupancyMap.load(str(DATA_PATH/'{}.wkt'.format(data)))

    print('Writing SUMO network occupancy map mesh triangles...')
    with (DATA_PATH/'{}.mesh'.format(data)).open('w') as file:
        file.write(','.join('{},{},{}'.format(v.x, v.y, v.z) for v in network_occupancy_map.get_mesh_triangles()))

    print('Calculating sidewalk...')
    sidewalk = network_occupancy_map.create_sidewalk(1.5)
    
    print('Calculating sidewalk occupancy map...')
    sidewalk_occupancy_map = sidewalk.create_occupancy_map(3)
    
    print('Writing sidewalk occupancy map...')
    sidewalk_occupancy_map.save(str(DATA_PATH/'{}.sidewalk.wkt'.format(data)))

    print('Test loading sidewalk occupancy map...')
    carla.OccupancyMap.load(str(DATA_PATH/'{}.sidewalk.wkt'.format(data)))
    
    print('Writing sidewalk occupancy map mesh triangles...')
    with (DATA_PATH/'{}.sidewalk.mesh'.format(data)).open('w') as file:
        file.write(','.join('{},{},{}'.format(v.x, v.y, v.z) for v in sidewalk_occupancy_map.get_mesh_triangles()))

    print('Geometry processing complete!')
