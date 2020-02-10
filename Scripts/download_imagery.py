'''
Downloads the imagery for a given SUMO network located in Data/ and stores the
downloaded image tiles into Data/imagery.

Note the Data/imagery folder is shared across all datasets, since the image tiles
are global.

Usage:
  python3 download_imagery.py <dataset>

  Downloads the respective imagery for the SUMO network located at 
  Data/<dataset>.net.xml and stores the tiles in Data/imagery.

Example:
  python3 download_imagery.py meskel_square 
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

import requests
from pathlib import Path

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

def get_tile(zoom, row, column):
    r = requests.get(TILE_URL.format(zoom, row, column))
    return r.content

def get_tiles(zoom, min_lat, min_lon, max_lat, max_lon, output_dir):
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

    for row in range(top_left_id[1], bottom_right_id[1] + 1):
        for column in range(top_left_id[2], bottom_right_id[2] + 1):
            has_tile = check_tile(zoom, row, column)

            print('{}: {} / {} -> {}'.format(
                zoom, 
                (column - top_left_id[2]) + (row - top_left_id[1]) * width + 1, 
                height * width,
                1 if has_tile else 0))

            if has_tile:
                tile = get_tile(zoom, row, column)
                if not os.path.exists('{}/{}'.format(output_dir, zoom)):
                    os.makedirs('{}/{}'.format(output_dir, zoom)) 
                with open('{}/{}/{}_{}.jpeg'.format(output_dir, zoom, row, column), 'w+') as f:
                    f.write(tile)


if __name__ == '__main__':

    data = sys.argv[1]

    print('Loading SUMO network...')
    network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(data)))

    # Extract (x, y) from SUMO network and convert into LatLon.
    bounds_min = (network.original_bounds_min.y, network.original_bounds_min.x)
    bounds_max = (network.original_bounds_max.y, network.original_bounds_max.x)

    get_tiles(ZOOM_LEVEL, *bounds_min, *bounds_max, str(IMAGERY_PATH))
