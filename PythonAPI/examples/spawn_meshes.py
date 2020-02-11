#!/usr/bin/env python3

"""Spawns meshes extracted from a dataset located in (<summit_root>/Data/)."""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
from pathlib import Path
import random

DATA_PATH = Path(os.path.realpath(__file__)).parent.parent.parent/'Data'   
WALL_MAT = [
        '/Game/Carla/Static/Buildings/aa_wall_mat',
        '/Game/Carla/Static/Buildings/Block08/Block08Wall_1',
        '/Game/Carla/Static/Buildings/Block08/Block08Wall_2',
        '/Game/Carla/Static/Buildings/Apartment02/M_Apartment02_Wall_1',
        '/Game/Carla/Static/Buildings/Apartment02/M_Apartment02Wall_2',
        '/Game/Carla/Static/Buildings/Block01/M_Block01OuterWall',
        '/Game/Carla/Static/Buildings/Block02/M_Block02BaseWall_1',
        '/Game/Carla/Static/Buildings/Block02/M_Block02Wall_2',
        '/Game/Carla/Static/Buildings/Block02/M_Block02WallRoof_1',
        '/Game/Carla/Static/Buildings/Block02/M_Block02WallRoof_2',
        '/Game/Carla/Static/Buildings/Block04/M_Block04Wall_1',
        '/Game/Carla/Static/Buildings/Block04/M_Block04Wall_2',
        '/Game/Carla/Static/Buildings/Block04/M_Block04Wall_3',
        '/Game/Carla/Static/Buildings/Block06/M_Block06StairsWall_1',
        '/Game/Carla/Static/Buildings/Block06/M_Block06Stairswall_2',
        '/Game/Carla/Static/Buildings/House01/M_House01StairsWall_3',
        '/Game/Carla/Static/Buildings/House01/M_House01Wall_1',
        '/Game/Carla/Static/Buildings/House01/M_House01Wall_2',
        '/Game/Carla/Static/Buildings/House12/M_House12Wall_4',
        '/Game/Carla/Static/Buildings/House12/M_House12Wall_6',
        '/Game/Carla/Static/Buildings/TerracedHouse02/M_TerracedHouse02Wall',
        '/Game/Carla/Static/Buildings/TerracedHouse02/M_TerracedHouse02Wall_4',
        '/Game/Carla/Static/Walls/WallTunnel01/M_WallTunnel01',
        '/Game/Carla/Static/Walls/Wall15/T_Wall15']

def spawn_meshes(client, dataset):
    sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(dataset)))
    sumo_network_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.network.wkt'.format(dataset)))
    roadmark_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.roadmark.wkt'.format(dataset)))
    sidewalk_occupancy = carla.OccupancyMap.load(str(DATA_PATH/'{}.sidewalk.wkt'.format(dataset)))
    landmark_occupancies = []
    for p in (DATA_PATH/'{}.landmarks'.format(dataset)).glob('*.landmark.wkt'):
        landmark_occupancies.append(carla.OccupancyMap.load(str(p)))

    commands = []
    
    # Roadmark mesh.
    commands.append(carla.command.SpawnDynamicMesh(
        roadmark_occupancy.get_mesh_triangles(),
        '/Game/Carla/Static/GenericMaterials/LaneMarking/M_MarkingLane_W',
        6)) # 6 = Road line

    # SUMO network mesh.
    commands.append(carla.command.SpawnDynamicMesh(
        sumo_network_occupancy.get_mesh_triangles(),
        '/Game/Carla/Static/GenericMaterials/Masters/LowComplexity/M_Road1',
        7)) # 7 = Road
    
    # Sidewalk mesh.
    commands.append(carla.command.SpawnDynamicMesh(
        sidewalk_occupancy.get_mesh_triangles(),
        '/Game/Carla/Static/GenericMaterials/Ground/GroundWheatField_Mat',
        8)) # 8 = Sidewalk
    
    # Landmark meshes.
    for landmark_occupancy in landmark_occupancies:
        commands.append(carla.command.SpawnDynamicMesh(
            landmark_occupancy.get_mesh_triangles(20), # Ceiling
            random.choice(WALL_MAT),
            1)) # 1 = Building
        commands.append(carla.command.SpawnDynamicMesh(
            landmark_occupancy.get_mesh_triangles(0), # Ground
            random.choice(WALL_MAT),
            1)) # 1 = Building
        commands.append(carla.command.SpawnDynamicMesh(
            landmark_occupancy.get_wall_mesh_triangles(20), # Walls
            random.choice(WALL_MAT),
            1)) # 1 = Building

    client.apply_batch_sync(commands)

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-d', '--dataset',
        metavar='D',
        default='meskel_square',
        help='Name of dataset (default: meskel_square)')
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    spawn_meshes(client, args.dataset)

if __name__ == '__main__':
    main()
