import glob
import math
import os
import sys

sys.path.append(glob.glob('../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import carla

data = 'beijing'

if __name__ == '__main__':

    print('Loading SUMO network...')
    network = carla.SumoNetwork.load('../../Data/' + data + '.net.xml')
    
    print('Testing network segments...')
    segment_map = network.create_segment_map()

    print('Calculating SUMO network occupancy map...')
    network_occupancy_map = network.create_occupancy_map()

    print('Writing SUMO network occupancy map...')
    network_occupancy_map.save('../../Data/' + data + '.wkt')

    print('Test loading SUMO network occupancy map...')
    carla.OccupancyMap.load('../../Data/' + data + '.wkt')

    print('Writing SUMO network occupancy map mesh triangles...')
    with open('../../Data/' + data + '.mesh', 'w') as file:
        file.write(','.join('{},{},{}'.format(v.x, v.y, v.z) for v in network_occupancy_map.get_mesh_triangles()))

    print('Calculating sidewalk...')
    sidewalk = network_occupancy_map.create_sidewalk(1.5)
    
    print('Calculating sidewalk occupancy map...')
    sidewalk_occupancy_map = sidewalk.create_occupancy_map(3)
    
    print('Writing sidewalk occupancy map...')
    sidewalk_occupancy_map.save('../../Data/' + data + '.sidewalk.wkt')

    print('Test loading sidewalk occupancy map...')
    carla.OccupancyMap.load('../../Data/' + data + '.sidewalk.wkt')
    
    print('Writing sidewalk occupancy map mesh triangles...')
    with open('../../Data/' + data + '.sidewalk.mesh', 'w') as file:
        file.write(','.join('{},{},{}'.format(v.x, v.y, v.z) for v in sidewalk_occupancy_map.get_mesh_triangles()))
