import glob
import math
import os
import sys

sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import carla

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(10.0)

landmark_map = carla.LandmarkMap.load(
        '/home/leeyiyuan/carla/Data/map.osm',
        carla.Vector2D(-11551102.28, -143022.13))
client.get_world().spawn_dynamic_mesh(
        landmark_map.get_mesh_triangles(20),
        '/Game/Carla/Static/GenericMaterials/Asphalt/M_Asphalt01')
        
with open('/home/leeyiyuan/carla/Data/map.net.xml', 'r') as file:
    map_data = file.read()
network = carla.SumoNetwork.load(map_data)
network_occupancy_map = network.create_occupancy_map()
client.get_world().spawn_dynamic_mesh(
        network_occupancy_map.get_mesh_triangles(),
        '/Game/Carla/Static/GenericMaterials/Asphalt/M_Asphalt01')
