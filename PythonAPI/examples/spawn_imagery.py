#!/usr/bin/env python

"""Spawns downloaded imagery for a dataset located in (<summit_root>/Data/)."""

import glob
import os
import sys

try:
    sys.path.append(glob.glob(os.path.abspath('%s/../../carla/dist/carla-*%d.%d-%s.egg' % (
        os.path.realpath(__file__),
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    pass

import carla

import argparse
import random
import math
if sys.version_info.major == 2:
    from pathlib2 import Path
else:
    from pathlib import Path

DATA_PATH = Path(os.path.realpath(__file__)).parent.parent.parent/'Data'   
IMAGERY_PATH = DATA_PATH/'imagery'
ZOOM_LEVEL = 18
VERTICAL_OFFSET = -0.1

def deg2num(zoom, lat_deg, lon_deg):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (zoom, ytile, xtile)

def num2deg(zoom, ytile, xtile):
  n = 2.0 ** zoom
  lon_deg = xtile / n * 360.0 - 180.0
  lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
  lat_deg = math.degrees(lat_rad)
  return (lat_deg, lon_deg)

EARTH_RADIUS = 6378137.0 # in meters on the equator

def lat2y(a):
    return math.log(math.tan(math.pi / 4 + math.radians(a) / 2)) * EARTH_RADIUS

def lon2x(a):
    return math.radians(a) * EARTH_RADIUS

def project(a): # LatLon -> CARLA coordinates.
    return carla.Vector2D(lat2y(a[0]), lon2x(a[1]))

def spawn_imagery(client, zoom, min_lat, min_lon, max_lat, max_lon, offset):
    bottom_left_id = deg2num(zoom, min_lat, min_lon)
    top_right_id = deg2num(zoom, max_lat, max_lon)
    top_left_id = (zoom, top_right_id[1], bottom_left_id[2])
    bottom_right_id = (zoom, bottom_left_id[1], top_right_id[2])

    if bottom_right_id[1] >= top_left_id[1]:
        height = bottom_right_id[1] - top_left_id[1] + 1
    else:
        height = (top_left_id[1] + 1) + (2 ** zoom - bottom_right_id[1])

    if bottom_right_id[2] >= top_left_id[2]:
        width = bottom_right_id[2] - top_left_id[2] + 1
    else:
        width = (top_left_id[2] + 1) + (2 ** zoom - bottom_right_id[2])

    for row in range(top_left_id[1], bottom_right_id[1] + 1):
        for column in range(top_left_id[2], bottom_right_id[2] + 1):

            path = IMAGERY_PATH/'{}/{}_{}.jpeg'.format(zoom, row, column)

            if not path.exists():
                print('Path not found! {}/{}_{}'.format(zoom, row, column))
                continue

            with path.open('rb') as f:
                data = [ord(b) if isinstance(b, str) else b for b in f.read()]

            bounds_min = project(num2deg(zoom, row + 1, column)) + offset
            bounds_min = carla.Vector3D(bounds_min.x, bounds_min.y, VERTICAL_OFFSET)
            bounds_max = project(num2deg(zoom, row, column + 1)) + offset
            bounds_max = carla.Vector3D(bounds_max.x, bounds_max.y, VERTICAL_OFFSET)

            client.get_world().spawn_dynamic_tile_mesh(bounds_min, bounds_max, data, 9)

def main(args):
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    
    # Extract (x, y) from SUMO network and convert into LatLon.
    sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format(args.dataset)))
    bounds_min = (sumo_network.original_bounds_min.x, sumo_network.original_bounds_min.y)
    bounds_max = (sumo_network.original_bounds_max.x, sumo_network.original_bounds_max.y)

    spawn_imagery(client, ZOOM_LEVEL, 
            sumo_network.original_bounds_min.x, sumo_network.original_bounds_min.y,
            sumo_network.original_bounds_max.x, sumo_network.original_bounds_max.y,
            sumo_network.offset)

if __name__ == '__main__':
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

    main(args)
