#!/usr/bin/env python3

'''
Processes a dataset (using OSM and SUMO network data) located in (<summit_root>/Data/) to produce the respective mesh files in the same folder (<summit_root>/Data/).
'''

import glob
import math
import os
import sys

sys.path.append(glob.glob('../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import carla

import argparse
from pathlib import Path
import shutil

DATA_PATH = Path(os.path.realpath(__file__)).parent.parent/'Data'

if __name__ == '__main__':

    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-d', '--dataset',
        metavar='D',
        default='meskel_square',
        help='Name of dataset (default: meskel_square)')
    args = argparser.parse_args()

    print('Reading from OSM file and SUMO network...')
    sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(args.dataset)))
    landmark_occupancies = carla.Landmark.load(
            str(DATA_PATH/'{}.osm'.format(args.dataset)), sumo_network.offset)
   
    print('Creating occupancy data...')
    sumo_network_occupancy = sumo_network.create_occupancy_map()
    roadmark_occupancy = sumo_network.create_roadmark_occupancy_map()
    sidewalk_occupancy = sumo_network_occupancy.create_sidewalk(1.5).create_occupancy_map(3.0)
    landmark_occupancies = [l.difference(sumo_network_occupancy).difference(sidewalk_occupancy) 
            for l in landmark_occupancies]
    landmark_occupancies = [l for l in landmark_occupancies if not l.is_empty]
    sumo_network_occupancy = sumo_network_occupancy.difference(roadmark_occupancy)

    print('Writing occupancy data...')
    sumo_network_occupancy.save(str(DATA_PATH/'{}.network.wkt'.format(args.dataset)))
    roadmark_occupancy.save(str(DATA_PATH/'{}.roadmark.wkt'.format(args.dataset)))
    sidewalk_occupancy.save(str(DATA_PATH/'{}.sidewalk.wkt'.format(args.dataset)))
    shutil.rmtree(str((DATA_PATH/'{}.landmarks'.format(args.dataset))), ignore_errors=True)
    (DATA_PATH/'{}.landmarks'.format(args.dataset)).mkdir(exist_ok=True)
    for (i, landmark_occupancy) in enumerate(landmark_occupancies):
        landmark_occupancy.save(
                str(DATA_PATH/'{}.landmarks'.format(args.dataset)/'{:06d}.landmark.wkt'.format(i)))

    print('Occupancy extraction complete!')
