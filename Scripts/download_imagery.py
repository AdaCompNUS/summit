#!/usr/bin/env python3

'''
Downloads imagery for a map (using its OSM file located in <summit_root>/Data/) and stores 
the downloaded image tiles into (<summit_root>/Data/imagery/).
'''

#/usr/bin/env python3

import glob
import os
import math
import sys

sys.path.append(glob.glob('../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

import carla

import argparse
import requests
from pathlib import Path
from multiprocessing import Pool

DATA_PATH = Path(os.path.realpath(__file__)).parent.parent/'Data'
IMAGERY_PATH = DATA_PATH/'imagery'

TILE_URL = 'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{0}/{1}/{2}' # z/y/x
CHECK_URL = 'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tilemap/{0}/{1}/{2}'
ZOOM_LEVEL = 18
    
# https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
def deg2num(zoom, lat_deg, lon_deg):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (zoom, ytile, xtile) # zoom/row/column

def check_tile(zoom, row, column):
    r = requests.get(CHECK_URL.format(zoom, row, column))
    return r.json()['data'][0] == 1

def download_tile(zoom, row, column):
    r = requests.get(TILE_URL.format(zoom, row, column))
    return r.content

def process_tile(tile):
    (zoom, row, column) = tile

    has_tile = check_tile(zoom, row, column)

    if has_tile:
        tile = download_tile(zoom, row, column)
        if not (IMAGERY_PATH/str(zoom)).exists():
            (IMAGERY_PATH/str(zoom)).mkdir(parents=True, exist_ok=True)
        with (IMAGERY_PATH/str(zoom)/'{}_{}.jpeg'.format(row, column)).open('wb') as f:
            f.write(tile)

def get_tiles(zoom, min_lat, min_lon, max_lat, max_lon, jobs):
    bottom_left_id = deg2num(zoom, min_lat, min_lon)
    top_right_id = deg2num(zoom, max_lat, max_lon)
    top_left_id = (zoom, top_right_id[1], bottom_left_id[2])
    bottom_right_id = (zoom, bottom_left_id[1], top_right_id[2])

    if bottom_right_id[1] >= top_left_id[1]:
        height = bottom_right_id[1] - top_left_id[1] + 1
    else:
        height = (top_left_id[1] + 1) + (2**zoom - bottom_right_id[1])

    if bottom_right_id[2] >= top_left_id[2]:
        width = bottom_right_id[2] - top_left_id[2] + 1
    else:
        width = (top_left_id[2] + 1) + (2**zoom - bottom_right_id[2])

    tiles = []
    for row in range(top_left_id[1], bottom_right_id[1] + 1):
        for column in range(top_left_id[2], bottom_right_id[2] + 1):
            tiles.append((zoom, row, column))

    p = Pool(jobs)

    for i, _ in enumerate(p.imap_unordered(process_tile, tiles), 1):
        print('Completed {} / {}'.format(i, len(tiles)))

if __name__ == '__main__':

    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-d', '--dataset',
        metavar='D',
        default='meskel_square',
        help='Name of dataset (default: meskel_square)')
    argparser.add_argument(
        '-j', '--jobs',
        metavar='J',
        default='4',
        help='Number of jobs to run at once (default: 4)')
    args = argparser.parse_args()

    sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(args.dataset)))

    # Get geographical bounds in LatLon (XY).
    bounds_min = (sumo_network.original_bounds_min.x, sumo_network.original_bounds_min.y)
    bounds_max = (sumo_network.original_bounds_max.x, sumo_network.original_bounds_max.y)

    get_tiles(ZOOM_LEVEL, *bounds_min, *bounds_max, int(args.jobs))
