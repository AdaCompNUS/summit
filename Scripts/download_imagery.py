import os
import math
import numpy as np
import requests

TILE_URL = 'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{0}/{1}/{2}' # z/y/x
CHECK_URL = 'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tilemap/{0}/{1}/{2}'
ZOOM_MIN = 18
ZOOM_MAX = 19


map_locations = ['map', 'meskel_square', 'magic', 'highway', 'chandni_chowk', 'shi_men_er_lu', 'beijing']
# map_locations = ['shi_men_er_lu', 'beijing']


def set_map_bounds(map_location):
    if map_location is 'map':
        (BOUNDS_MIN, BOUNDS_MAX) = ((1.2894000, 103.7669000), (1.3088000, 103.7853000))

    if map_location is 'meskel_square':
        (BOUNDS_MIN, BOUNDS_MAX) = ((9.00802, 38.76009), (9.01391, 38.76603))

    if map_location is 'magic':
        (BOUNDS_MIN, BOUNDS_MAX) = ((51.5621800, -1.7729100), (51.5633900, -1.7697300))

    if map_location is 'highway':
        (BOUNDS_MIN, BOUNDS_MAX) = ((1.2983800, 103.7777000), (1.3003700, 103.7814900))

    if map_location is 'chandni_chowk':
        (BOUNDS_MIN, BOUNDS_MAX) = ((28.653888, 77.223296), (28.660295, 77.236850))

    if map_location is 'shi_men_er_lu':
        (BOUNDS_MIN, BOUNDS_MAX) = ((31.229828, 121.438702), (31.242810,121.464944))

    if map_location is 'beijing':
        (BOUNDS_MIN, BOUNDS_MAX) = ((39.8992818, 116.4099687), (39.9476116, 116.4438916))

    return BOUNDS_MIN, BOUNDS_MAX
    
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

def get_tiles(zoom, (min_lat, min_lon), (max_lat, max_lon), output_dir):
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

            # TODO: Determine if output is PNG, JPEG or something else.
            if has_tile:
                tile = get_tile(zoom, row, column)
                if not os.path.exists('{}/{}'.format(output_dir, zoom)):
                    os.makedirs('{}/{}'.format(output_dir, zoom)) 
                with open('{}/{}/{}_{}.jpeg'.format(output_dir, zoom, row, column), 'w+') as f:
                    f.write(tile)


if __name__ == '__main__':
    cur_path = os.path.dirname(os.path.abspath(__file__))
    imagery_path = os.path.join(cur_path, "../../Data/imagery")
    print("Data folder {}".format(imagery_path))

    for map_location in map_locations:
        BOUNDS_MIN, BOUNDS_MAX = set_map_bounds(map_location)
        print("Parsing map for {} from {} to {}".format(map_location, BOUNDS_MIN, BOUNDS_MAX))

        for i in range(ZOOM_MIN, ZOOM_MAX + 1):
            get_tiles(i, BOUNDS_MIN, BOUNDS_MAX, imagery_path)
